import os
import logging
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType
from dotenv import load_dotenv

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("SETUP_INDEXES")

load_dotenv(override=True)

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "textbook_embeddings"

def setup_indexes():
    logger.info("Connecting to Qdrant...")
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    
    # Check collection
    try:
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]
        logger.info(f"Available collections: {collection_names}")
        
        if COLLECTION_NAME not in collection_names:
            logger.error(f"Collection '{COLLECTION_NAME}' does not exist! Run ingest.py first.")
            return
        
        logger.info(f"Collection '{COLLECTION_NAME}' found.")
    except Exception as e:
        logger.error(f"Failed to check collections: {e}")
        return

    # Create Indexes
    # We filter by 'difficulty' and 'language' in main.py, so we MUST index them.
    # The error "Index required... types: [keyword]" confirms this.
    
    fields_to_index = [
        ("metadata.difficulty", PayloadSchemaType.KEYWORD),
        ("metadata.language", PayloadSchemaType.KEYWORD),
        ("metadata.document_id", PayloadSchemaType.KEYWORD)
    ]

    for field, schema_type in fields_to_index:
        logger.info(f"Creating index for field: {field}")
        try:
            client.create_payload_index(
                collection_name=COLLECTION_NAME,
                field_name=field,
                field_schema=schema_type
            )
            logger.info(f"✅ Index created for {field}")
        except Exception as e:
            logger.warning(f"Could not create index for {field} (might already exist): {e}")

    logger.info("Index setup complete!")

if __name__ == "__main__":
    setup_indexes()
