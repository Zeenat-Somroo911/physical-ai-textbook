from config import settings
import os

print(f"DEBUG CONFIG CHECK")
print(f"------------------")
print(f"Environment MOCK_MODE: {os.environ.get('MOCK_MODE')}")
print(f"Settings MOCK_MODE: {settings.mock_mode}")
print(f"Settings DATABASE_URL: {settings.database_url}")
print(f"Settings QDRANT_URL: {settings.qdrant_url}")
