"""
Generate all content versions for the Physical AI Textbook.

Creates 9 versions of each markdown file:
- 3 difficulty levels: easy, medium, hard
- 3 languages: english, urdu, roman urdu
- Total: 69 files × 9 versions = 621 generated files

Usage:
    python generate_all_versions.py
"""

import os
import sys
# Force unbuffered output
sys.stdout.reconfigure(encoding='utf-8', line_buffering=True)
print("DEBUG: Script initializing...", flush=True)

import asyncio
from pathlib import Path
import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv(override=True)

# Configuration
DOCS_DIR = Path("../docs")
OUTPUT_DIR = Path("../generated-content")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
# GEMINI_MODEL = os.getenv("GEMINI_MODEL", "models/gemini-flash-latest")
GEMINI_MODEL = "models/gemini-flash-latest" # Force verified model (ignoring .env)

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY not found in .env")

# Configure Gemini
genai.configure(api_key=GEMINI_API_KEY)

# Content variations
DIFFICULTIES = ["easy", "medium", "hard"]
LANGUAGES = ["english", "urdu", "roman"]

# Prompts for each difficulty
DIFFICULTY_PROMPTS = {
    "easy": """Rewrite this content for COMPLETE BEGINNERS with these requirements:

1. Use VERY SIMPLE words and SHORT sentences
2. Add REAL-WORLD ANALOGIES for every technical concept (like comparing to everyday objects)
3. Explain ALL technical terms in parentheses when first used
4. Add step-by-step examples with detailed explanations
5. Make it FRIENDLY and ENCOURAGING in tone
6. Add "💡 Tip:" boxes for helpful hints
7. Make it AT LEAST 40% LONGER than the original

Think of explaining to someone who has NEVER seen robotics before.""",

    "medium": """Rewrite this content for INTERMEDIATE learners:

1. Use proper technical terminology
2. Include practical code examples
3. Balance theory and hands-on practice
4. Assume basic programming knowledge
5. Keep a professional but approachable tone
6. Maintain similar length to original""",

    "hard": """Rewrite this content for ADVANCED learners:

1. Assume strong prior knowledge
2. Focus on optimizations, edge cases, and advanced patterns
3. Include performance considerations
4. Add references to research papers or advanced topics
5. Use concise, technical language
6. Skip basic explanations - go straight to advanced concepts"""
}

# Prompts for each language
LANGUAGE_PROMPTS = {
    "english": "Keep the content in English.",
    
    "urdu": """Translate the ENTIRE content to Urdu:
- Use proper Urdu script (اردو)
- Keep technical terms in English in parentheses
- Maintain right-to-left text flow
- Use appropriate Urdu technical vocabulary where it exists""",
    
    "roman": """Translate the ENTIRE content to Roman Urdu:
- Write Urdu using English/Latin script (Roman alphabet)
- This is Urdu language but written in English letters
- Example: "Yeh robotics ka basic concept hai"
- Keep technical terms in English
- Make it natural for Pakistani readers who read Roman Urdu"""
}

# Safety settings for Gemini
SAFETY_SETTINGS = {
    "HARM_CATEGORY_HARASSMENT": "BLOCK_NONE",
    "HARM_CATEGORY_HATE_SPEECH": "BLOCK_NONE",
    "HARM_CATEGORY_SEXUALLY_EXPLICIT": "BLOCK_NONE",
    "HARM_CATEGORY_DANGEROUS_CONTENT": "BLOCK_NONE",
}


async def generate_version(content: str, difficulty: str, language: str) -> str:
    """
    Generate one version of content using Gemini API.
    
    Args:
        content: Original markdown content
        difficulty: easy, medium, or hard
        language: english, urdu, or roman
        
    Returns:
        Generated content as string
    """
    prompt = f"""{DIFFICULTY_PROMPTS[difficulty]}

{LANGUAGE_PROMPTS[language]}

IMPORTANT: Return ONLY the rewritten content. DO NOT include meta-commentary or explanations about what you did.

ORIGINAL CONTENT:
{content}

REWRITTEN CONTENT:"""
    
    for attempt in range(5):
        try:
            model = genai.GenerativeModel(GEMINI_MODEL)
            response = await asyncio.to_thread(
                model.generate_content,
                prompt
            )
            return response.text.strip()
        except Exception as e:
            if "429" in str(e):
                wait_time = (attempt + 1) * 20 # 20s, 40s, 60s...
                print(f"\n    ⏳ GOOGLE SPEED LIMIT HIT (Normal). Pausing {wait_time}s...", end="", flush=True)
                for _ in range(wait_time):
                    await asyncio.sleep(1)
                    print(".", end="", flush=True)
                print(" Resuming!")
            else:
                print(f"    ❌ Error generation failed: {e}")
                return None
    return None


async def process_file(file_path: Path, file_index: int, total_files: int):
    """
    Process one markdown file - generate all 9 versions.
    
    Args:
        file_path: Path to original markdown file
        file_index: Current file number (for progress display)
        total_files: Total number of files to process
    """
    # Read original content
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print(f"❌ Error reading {file_path}: {e}")
        return
    
    # Calculate relative path for output directory
    try:
        rel_path = file_path.relative_to(DOCS_DIR)
    except ValueError:
        print(f"❌ File not in docs directory: {file_path}")
        return
    
    # Create output directory: generated-content/path/to/file_stem/
    output_dir = OUTPUT_DIR / rel_path.parent / rel_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n[{file_index}/{total_files}] Processing: {rel_path}")
    
    # Generate all 9 combinations (3 difficulties × 3 languages)
    for difficulty in DIFFICULTIES:
        for language in LANGUAGES:
            output_file = output_dir / f"{difficulty}_{language}.md"
            
            # Skip if already exists
            if output_file.exists():
                print(f"  ✓ Skip (exists): {difficulty}_{language}.md")
                continue
            
            print(f"  → Generating: {difficulty}_{language}.md...")
            
            # Generate content
            generated = await generate_version(content, difficulty, language)
            
            if generated is None:
                print(f"    ❌ Failed to generate")
                continue
            
            # Save to file
            try:
                with open(output_file, 'w', encoding='utf-8') as f:
                    f.write(generated)
                print(f"  ✅ Saved: {difficulty}_{language}.md ({len(generated)} chars)")
            except Exception as e:
                print(f"    ❌ Error saving: {e}")
            
            # Rate limiting - wait 4 seconds (15 RPM limit)
            await asyncio.sleep(4)


async def main():
    """Main function to process all markdown files."""
    print("=" * 60)
    print("Physical AI Textbook - Content Generation")
    print("=" * 60)
    print(f"Docs directory: {DOCS_DIR.absolute()}")
    print(f"Output directory: {OUTPUT_DIR.absolute()}")
    print(f"Model: {GEMINI_MODEL}")
    print("=" * 60)
    
    # Find all markdown files
    md_files = sorted(DOCS_DIR.rglob("*.md"))
    
    # Filter out any generated content if it somehow got in docs
    md_files = [f for f in md_files if "generated-content" not in str(f)]
    
    print(f"\nFound {len(md_files)} markdown files to process")
    print(f"Will generate {len(md_files) * 9} total files")
    print(f"Estimated time: ~{len(md_files) * 9 * 1.5 / 60:.0f} minutes\n")
    
    # input("Press Enter to start generation...")
    print("Starting generation...")
    
    # Process each file
    for i, file_path in enumerate(md_files, 1):
        await process_file(file_path, i, len(md_files))
    
    print("\n" + "=" * 60)
    print("✅ ALL DONE!")
    print(f"Generated content saved to: {OUTPUT_DIR.absolute()}")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
