from dotenv import load_dotenv
import os

load_dotenv(override=True)

keys = ["GEMINI_API_KEY", "QDRANT_API_KEY", "QDRANT_URL", "HF_TOKEN"]

print("--- ENV CHECK ---")
for key in keys:
    val = os.getenv(key)
    if val:
        masked = val[:4] + "..." + val[-4:] if len(val) > 8 else "****"
        print(f"{key}: Found ({masked})")
    else:
        print(f"{key}: MISSING")
