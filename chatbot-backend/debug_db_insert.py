import asyncio
import logging
import uuid
import json
from db import init_db_pool, close_db_pool, create_user
from config import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_insert():
    print("Testing DB Connection and Insert...")
    try:
        # Initialize pool
        await init_db_pool()
        
        # Random email to avoid conflict
        email = f"debug_user_{uuid.uuid4()}@example.com"
        password_hash = "debug_hash_123"
        full_name = "Debug User"
        preferences = {"theme": "dark", "test": True}
        
        print(f"Attempting to create user: {email}")
        
        # Try creating user
        user = await create_user(
            email=email,
            password_hash=password_hash,
            full_name=full_name,
            preferences=preferences
        )
        
        print("SUCCESS! User created:")
        print(user)
        
    except Exception as e:
        print("\n!!! ERROR OCCURRED !!!\n")
        print(f"Type: {type(e)}")
        print(f"Message: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await close_db_pool()

if __name__ == "__main__":
    # Force mock mode off just in case, though config should load from env
    # settings.mock_mode = False 
    asyncio.run(test_insert())
