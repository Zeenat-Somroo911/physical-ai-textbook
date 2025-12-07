
import os
import requests
import json
from dotenv import load_dotenv

# We need to manually make a REST request to see the headers
# The python SDK hides them.

def check_headers():
    load_dotenv(override=True)
    api_key = os.getenv("GEMINI_API_KEY")
    # Using the exact same model string we verified
    model_name = "gemini-flash-latest"
    
    url = f"https://generativelanguage.googleapis.com/v1beta/models/{model_name}:generateContent?key={api_key}"
    
    headers = {'Content-Type': 'application/json'}
    data = {
        "contents": [{
            "parts": [{"text": "Hello"}]
        }]
    }
    
    print(f"Testing URL: {url.split('?')[0]}...")
    
    try:
        response = requests.post(url, headers=headers, json=data)
        
        print(f"\nStatus Code: {response.status_code}")
        print("-" * 40)
        print("ALL HEADERS:")
        for k, v in response.headers.items():
            print(f"{k}: {v}")
        print("-" * 40)
        
        if response.status_code != 200:
            print("Response Body:")
            print(response.text)
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    check_headers()
