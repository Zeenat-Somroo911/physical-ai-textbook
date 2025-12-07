import os
from pathlib import Path

env_file = Path(".env")
model_line = 'GEMINI_MODEL="models/gemini-flash-latest"'

# Read current content
if env_file.exists():
    with open(env_file, 'r') as f:
        lines = f.readlines()
    
    # Check if GEMINI_MODEL already exists
    has_model = any('GEMINI_MODEL' in line for line in lines)
    
    if not has_model:
        # Add it
        with open(env_file, 'a') as f:
            f.write(f"\n{model_line}\n")
        print(f"✓ Added {model_line} to .env")
    else:
        print("✓ GEMINI_MODEL already in .env")
else:
    print("✗ .env file not found")
