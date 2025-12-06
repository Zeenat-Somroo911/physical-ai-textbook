"""
Content ingestion script for populating the vector database.

Processes markdown files and ingests them into Qdrant.
"""

import os
import logging
import uuid
from typing import List, Dict, Any
from pathlib import Path

import openai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct

from config import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize OpenAI
openai.api_key = settings.openai_api_key

# Initialize Qdrant client
if settings.qdrant_api_key:
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )
else:
    qdrant_client = QdrantClient(url=settings.qdrant_url)


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
        end = start + chunk_size
        
        # Try to break at sentence boundary
        if end < len(text):
            # Look for sentence endings
            for break_char in ['. ', '\n\n', '\n']:
                last_break = text.rfind(break_char, start, end)
                if last_break != -1:
                    end = last_break + len(break_char)
                    break
        
        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)
        
        start = end - overlap
    
    return chunks


def get_embedding(text: str) -> List[float]:
    """
    Get embedding for text using OpenAI.
    
    Args:
        text: Text to embed
        
    Returns:
        List of floats representing the embedding
    """
    try:
        response = openai.embeddings.create(
            model=settings.openai_embedding_model,
            input=text
        )
        return response.data[0].embedding
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
    points = []
    
    for idx, chunk in enumerate(chunks):
        try:
            # Get embedding
            embedding = get_embedding(chunk)
            
            # Create point
            point = PointStruct(
                id=str(uuid.uuid4()),
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
            points.append(point)
        
        except Exception as e:
            logger.error(f"Error processing chunk {idx}: {e}", exc_info=True)
            continue
    
    if not points:
        logger.warning("No points to ingest")
        return 0
    
    try:
        # Upsert points in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            qdrant_client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=batch
            )
            logger.info(f"Ingested batch {i // batch_size + 1}")
        
        logger.info(f"Successfully ingested {len(points)} chunks")
        return len(points)
    
    except Exception as e:
        logger.error(f"Error ingesting chunks: {e}", exc_info=True)
        raise


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


def ingest_directory(directory: Path, pattern: str = "*.md"):
    """
    Ingest all markdown files from a directory.
    
    Args:
        directory: Directory path
        pattern: File pattern to match
    """
    files = list(directory.rglob(pattern))
    logger.info(f"Found {len(files)} files to ingest")
    
    for file_path in files:
        try:
            logger.info(f"Processing {file_path}")
            
            # Process file
            file_data = process_markdown_file(file_path)
            
            # Generate document ID
            document_id = str(uuid.uuid5(
                uuid.NAMESPACE_URL,
                str(file_path)
            ))
            
            # Chunk content
            chunks = chunk_text(file_data["content"])
            
            # Ingest chunks
            chunks_created = ingest_chunks(
                chunks=chunks,
                metadata=file_data["metadata"],
                document_id=document_id
            )
            
            logger.info(f"Successfully ingested {chunks_created} chunks from {file_path}")
        
        except Exception as e:
            logger.error(f"Error processing {file_path}: {e}", exc_info=True)
            continue


def main():
    """Main ingestion function."""
    # Get docs directory
    docs_dir = Path(__file__).parent.parent / "docs"
    
    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        return
    
    logger.info(f"Ingesting content from {docs_dir}")
    
    # Ingest all markdown files
    ingest_directory(docs_dir)
    
    logger.info("Ingestion complete!")


if __name__ == "__main__":
    main()

