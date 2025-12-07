"""
Content ingestion script for populating the vector database.

Processes markdown files and ingests them into Qdrant.
"""

import os
import gc
import logging
import uuid
from typing import List, Dict, Any
from pathlib import Path

import openai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance

from config import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

import requests
import time
from huggingface_hub import InferenceClient

# Initialize Qdrant client
if settings.qdrant_api_key:
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )
else:
    qdrant_client = QdrantClient(url=settings.qdrant_url)

# Initialize HF Client
hf_client = InferenceClient(token=settings.hf_token, timeout=30.0)


def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """
    Split text into overlapping chunks.
    
    Args:
        text: Text to chunk
        chunk_size: Maximum size of each chunk
        overlap: Number of characters to overlap between chunks
        
    Returns:
        List of text chunks
    """
    if len(text) <= chunk_size:
        return [text]
    
    chunks = []
    start = 0
    
    while start < len(text):
        end = min(start + chunk_size, len(text))
        
        # Try to break at sentence boundary
        if end < len(text):
            # Look for sentence endings
            for break_char in ['. ', '\n\n', '\n']:
                last_break = text.rfind(break_char, start, end)
                if last_break != -1:
                    # Ensure we don't break too close to start causing infinite loop with overlap
                    if (last_break + len(break_char)) > (start + overlap + 10):
                        end = last_break + len(break_char)
                        break
        
        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)
        
        # Move forward
        # Calculate next step
        step = end - overlap
        
        # CRITICAL FIX: Ensure we always advance by at least 1 character
        # If the chunk was smaller than overlap, just move to end
        if step <= start:
            step = start + max(1, len(chunk) // 2) # Fallback progress
            
        start = max(step, end - overlap) # Normal case
        
        # Safety check: if end reached max, break
        if end >= len(text):
            break
            
    return chunks


def get_embedding(text: str) -> List[float]:
    """
    Get embedding for text using HuggingFace Inference Client.
    
    Args:
        text: Text to embed
        
    Returns:
        List of floats representing the embedding
    """
    try:
        # Retry logic handled partially by client, but loop for loading state
        for attempt in range(3):
            try:
                # feature_extraction returns ndarray
                response = hf_client.feature_extraction(text, model=settings.hf_embedding_model)
                if response is not None:
                     return response.tolist()
            except Exception as e:
                # Check for loading error in string representation
                if "loading" in str(e).lower():
                    logger.info(f"Model loading... (Attempt {attempt+1})")
                    time.sleep(5)
                    continue
                else:
                    raise e
        
        raise ValueError("Failed to get embedding (Model loading or other error)")

    except Exception as e:
        logger.error(f"Error getting embedding: {e}", exc_info=True)
        raise


def ingest_chunks(
    chunks: List[str],
    metadata: Dict[str, Any],
    document_id: str
) -> int:
    """
    Ingest chunks into Qdrant.
    
    Args:
        chunks: List of text chunks
        metadata: Metadata for the chunks
        document_id: Document ID
        
    Returns:
        Number of chunks ingested
    """
    points_batch = []
    
    for idx, chunk in enumerate(chunks):
        try:
            # Get embedding
            embedding = get_embedding(chunk)
            
            # Deterministic ID for Deduplication: uuid5(doc_id + chunk_index)
            # This ensures if we re-run, we overwrite efficiently
            point_id = str(uuid.uuid5(uuid.NAMESPACE_URL, f"{document_id}_{idx}"))
            
            # Create point
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": chunk,
                    "metadata": {
                        **metadata,
                        "document_id": document_id,
                        "chunk_index": idx,
                        "total_chunks": len(chunks)
                    }
                }
            )
            points_batch.append(point)
            
            # Upsert every 5 chunks to save memory
            if len(points_batch) >= 5:
                qdrant_client.upsert(
                    collection_name=settings.qdrant_collection_name,
                    points=points_batch
                )
                logger.info(f"Upserted 5 chunks (Progress: {idx+1}/{len(chunks)})")
                points_batch = [] # Clear memory
        
        except Exception as e:
            logger.error(f"Error processing chunk {idx}: {e}", exc_info=True)
            continue
            
    # Upsert remaining
    if points_batch:
        try:
            qdrant_client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=points_batch
            )
            logger.info(f"Upserted remaining {len(points_batch)} chunks")
        except Exception as e:
             logger.error(f"Error upserting final batch: {e}", exc_info=True)

    return len(chunks)


def process_markdown_file(file_path: Path) -> Dict[str, Any]:
    """
    Process a markdown file and extract content and metadata.
    
    Args:
        file_path: Path to markdown file
        
    Returns:
        Dictionary with content and metadata
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract frontmatter if present
        metadata = {}
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                frontmatter = parts[1]
                content = parts[2]
                
                # Parse frontmatter (simple YAML parsing)
                for line in frontmatter.split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        metadata[key.strip()] = value.strip().strip('"').strip("'")
        
        # Extract title from first heading
        lines = content.split('\n')
        title = None
        for line in lines[:10]:  # Check first 10 lines
            if line.startswith('# '):
                title = line[2:].strip()
                break
        
        return {
            "content": content,
            "metadata": {
                **metadata,
                "file_path": str(file_path),
                "file_name": file_path.name,
                "title": title or file_path.stem,
                "type": "markdown"
            }
        }
    
    except Exception as e:
        logger.error(f"Error processing file {file_path}: {e}", exc_info=True)
        raise


def extract_metadata_from_path(file_path: Path) -> Dict[str, str]:
    """
    Extract metadata like difficulty and language from file path/name.
    Expected format: {difficulty}_{language}.md (e.g., easy_urdu.md)
    """
    meta = {}
    try:
        # Check filename pattern
        stem = file_path.stem  # e.g., "easy_urdu"
        parts = stem.split('_')
        
        if len(parts) == 2:
            difficulty, language = parts
            if difficulty in ["easy", "medium", "hard"] and language in ["english", "urdu", "roman"]:
                meta["difficulty"] = difficulty
                meta["language"] = language
                return meta
                
    except Exception:
        pass
        
    # Default fallback if not found in filename
    return {
        "difficulty": "medium", 
        "language": "english"
    }


def ingest_directory(directory: Path, pattern: str = "*.md"):
    """
    Ingest all markdown files from a directory, including subdirectories.
    
    Args:
        directory: Directory path
        pattern: File pattern to match
    """
    # Also look in generated-content if we are scanning docs (backwards compatibility)
    paths_to_scan = [directory]
    
    # Check for parallel "generated-content" directory
    generated_dir = directory.parent / "generated-content"
    if generated_dir.exists():
        logger.info(f"Also scanning generated content at {generated_dir}")
        paths_to_scan.append(generated_dir)

    all_files = []
    for d in paths_to_scan:
        all_files.extend(list(d.rglob(pattern)))
        
    logger.info(f"Found {len(all_files)} files to ingest")
    
    for file_path in all_files:
        try:
            logger.info(f"Processing {file_path}")
            
            # Process file
            file_data = process_markdown_file(file_path)
            
            # Enrich metadata from filename/path
            path_meta = extract_metadata_from_path(file_path)
            combined_metadata = {**file_data["metadata"], **path_meta}
            
            # Generate document ID (deterministic based on path)
            document_id = str(uuid.uuid5(
                uuid.NAMESPACE_URL,
                str(file_path)
            ))
            
            # Chunk content
            chunks = chunk_text(file_data["content"])
            
            # Free memory of raw content immediately
            del file_data
            
            # Ingest chunks
            chunks_created = ingest_chunks(
                chunks=chunks,
                metadata=combined_metadata,
                document_id=document_id
            )
            
            logger.info(f"Successfully ingested {chunks_created} chunks from {file_path} (Meta: {path_meta})")
            
            # Explicit Cleanup
            del chunks
            del combined_metadata
            gc.collect()
        
        except MemoryError:
            logger.error(f"❌ MEMORY ERROR processing {file_path}. Skipping.")
            gc.collect()
        except Exception as e:
            logger.error(f"Error processing {file_path}: {str(e)[:200]}") # Truncate error log to save memory
            continue


def main():
    """Main ingestion function."""
    # Get docs directory
    docs_dir = Path(__file__).parent.parent / "docs"
    
    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        return
    
    logger.info(f"Ingesting content from {docs_dir}")
    
    # Ensure collection exists
    try:
        qdrant_client.get_collection(settings.qdrant_collection_name)
        logger.info(f"Collection '{settings.qdrant_collection_name}' exists.")
    except Exception as e:
        # Check if error is actually "Not Found" or something else
        if "not found" in str(e).lower() or "404" in str(e):
             logger.info(f"Collection '{settings.qdrant_collection_name}' not found. Creating...")
             try:
                qdrant_client.create_collection(
                    collection_name=settings.qdrant_collection_name,
                    vectors_config=VectorParams(size=384, distance=Distance.COSINE)
                )
                logger.info(f"Created collection '{settings.qdrant_collection_name}' (Size: 384)")
             except Exception as create_error:
                 logger.warning(f"Collection creation might have raced: {create_error}")
        else:
            logger.warning(f"Error checking collection (might exist): {e}")
    
    # Ingest all markdown files
    ingest_directory(docs_dir)
    
    logger.info("Ingestion complete!")


if __name__ == "__main__":
    main()

