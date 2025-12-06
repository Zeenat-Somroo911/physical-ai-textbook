import os
import google.generativeai as genai
from dotenv import load_dotenv

def list_models():
    load_dotenv(override=True)
    api_key = os.getenv("GEMINI_API_KEY")
    genai.configure(api_key=api_key)
    
    print("Checking available models...")
    try:
        for m in genai.list_models():
            if 'generateContent' in m.supported_generation_methods:
                print(f"- {m.name}")
    except Exception as e:
        print(f"Error listing models: {e}")

if __name__ == "__main__":
    list_models()
