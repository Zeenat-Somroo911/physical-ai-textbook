
import asyncio
import logging
import os
from db import init_db_pool, close_db_pool, get_db_connection

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def init_tables():
    """Initialize database tables from schema.sql."""
    try:
        # Initialize DB pool
        await init_db_pool()
        
        async with get_db_connection() as conn:
            # Enable UUID extension
            logger.info("Enabling uuid-ossp extension...")
            await conn.execute('CREATE EXTENSION IF NOT EXISTS "uuid-ossp";')

            # Targeted migration for Personalizations table
            logger.info("Creating personalizations table...")
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS personalizations (
                    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
                    user_id UUID NOT NULL,
                    chapter_id VARCHAR(255) NOT NULL,
                    chapter_path VARCHAR(500),
                    content TEXT NOT NULL,
                    content_type VARCHAR(50) DEFAULT 'note',
                    metadata JSONB DEFAULT '{}'::jsonb,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    CONSTRAINT unique_user_chapter_type UNIQUE (user_id, chapter_id, content_type)
                );
                
                CREATE INDEX IF NOT EXISTS idx_personalizations_user_id ON personalizations(user_id);
                CREATE INDEX IF NOT EXISTS idx_personalizations_chapter_id ON personalizations(chapter_id);
                CREATE INDEX IF NOT EXISTS idx_personalizations_content_type ON personalizations(content_type);
            """)
            
        logger.info("Personalizations table created successfully!")
        
    except Exception as e:
        logger.error(f"Error initializing tables: {e}")
        raise
    finally:
        await close_db_pool()

if __name__ == "__main__":
    asyncio.run(init_tables())
