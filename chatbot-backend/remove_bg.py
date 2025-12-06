from rembg import remove
from PIL import Image
import sys
import os

def remove_background(input_path, output_path):
    print(f"Processing: {input_path}")
    if not os.path.exists(input_path):
        print(f"Error: File not found at {input_path}")
        return
    
    try:
        input_image = Image.open(input_path)
        output_image = remove(input_image)
        output_image.save(output_path)
        print(f"Success! Saved to {output_path}")
    except Exception as e:
        print(f"Error removing background: {e}")

if __name__ == "__main__":
    # Default paths
    input_path = r"d:\ai-robotics\static\img\logo.png"
    output_path = r"d:\ai-robotics\static\img\logo.png"
    
    if len(sys.argv) > 1:
        input_path = sys.argv[1]
    
    remove_background(input_path, output_path)
