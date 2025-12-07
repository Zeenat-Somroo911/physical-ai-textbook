import os
import math
from pathlib import Path
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv(override=True)

# Config
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "textbook_embeddings"
CHUNK_SIZE = 1000
OVERLAP = 200

def count_chunks_in_file(file_path):
    """
    Simulates ingest.py chunking to count expected chunks.
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            text = f.read()
            
        if not text:
            return 0
            
        if len(text) <= CHUNK_SIZE:
            return 1
            
        # Simplified calculation matching the sliding window logic
        # Count how many windows of CHUNK_SIZE fit with OVERLAP
        count = 0
        start = 0
        while start < len(text):
            count += 1
            start = start + CHUNK_SIZE - OVERLAP
            
        return count
    except Exception:
        return 0

def scan_directory(directory):
    if not os.path.exists(directory):
        return 0, 0
    
    files = list(Path(directory).rglob("*.md"))
    total_chunks = 0
    
    print(f"Scanning {directory}...")
    for f in files:
        total_chunks += count_chunks_in_file(f)
        
    return len(files), total_chunks

def main():
    print("\n📊 PRECISE Ingestion Progress Tracker")
    print("=======================================")
    
    # 1. Calculate EXPECTED Chunks from Local Files
    docs_dir = Path("../docs")
    gen_dir = Path("../generated-content")
    
    docs_files, docs_chunks = scan_directory(docs_dir)
    gen_files, gen_chunks = scan_directory(gen_dir)
    
    total_files = docs_files + gen_files
    total_expected_chunks = docs_chunks + gen_chunks
    
    print(f"\n📂 Local Data Analysis:")
    print(f"   - Original Docs:     {docs_files} files -> ~{docs_chunks} chunks")
    print(f"   - Generated Content: {gen_files} files -> ~{gen_chunks} chunks")
    print(f"   -------------------------------------------")
    print(f"   🎯 TARGET TOTAL:     {total_expected_chunks} Chunks")
    
    # 2. Check Actual Qdrant Count
    try:
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        count_result = client.count(
            collection_name=COLLECTION_NAME,
            exact=True
        )
        actual_chunks = count_result.count
        
        progress = (actual_chunks / total_expected_chunks * 100) if total_expected_chunks > 0 else 0
        
        print(f"\n☁️  Qdrant Real-time Status:")
        print(f"   ✅ Uploaded:  {actual_chunks} / {total_expected_chunks} Chunks")
        print(f"   📊 Progress:  [{'#' * int(progress/5)}{'.' * (20 - int(progress/5))}] {progress:.1f}%")
        
        if actual_chunks >= total_expected_chunks:
            print("\n🎉 INGESTION COMPLETE! (Or very close)")
        else:
            print(f"\n⏳ Remaining: {total_expected_chunks - actual_chunks} chunks to go...")
            
    except Exception as e:
        print(f"\n❌ Qdrant Connection Error: {e}")

if __name__ == "__main__":
    main()
