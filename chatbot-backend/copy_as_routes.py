"""
Copy Generated Content as Docusaurus Routes

Yeh script generated-content files ko docs folder mein copy karti hai
as separate routes so Docusaurus naturally serve kar sake.

Structure:
generated-content/intro/easy_english.md → docs/intro/easy-english.md

URLs:
/docs/intro           → Original
/docs/intro/easy-english  → Easy English version
/docs/intro/medium-urdu   → Medium Urdu version
"""

import os
import shutil
from pathlib import Path

def copy_as_routes():
    """Copy generated content as Docusaurus routes"""
    
    project_root = Path(__file__).parent.parent
    source_dir = project_root / "generated-content"
    docs_dir = project_root / "docs"
    
    print("=" * 60)
    print("Creating Route-Based Personalization")
    print("=" * 60)
    print(f"Source: {source_dir}")
    print(f"Docs: {docs_dir}")
    print()
    
    if not source_dir.exists():
        print(f"❌ Error: {source_dir} not found")
        return False
    
    copied = 0
    skipped = 0
    
    # Walk through generated-content
    for chapter_folder in source_dir.iterdir():
        if not chapter_folder.is_dir():
            continue
            
        chapter_name = chapter_folder.name
        print(f"\n📁 Processing: {chapter_name}")
        
        # Create chapter-specific subfolder in docs
        # e.g., docs/intro/ for personalized versions
        target_chapter_folder = docs_dir / chapter_name
        target_chapter_folder.mkdir(parents=True, exist_ok=True)
        
        # Copy each version
        for version_file in chapter_folder.glob("*.md"):
            # Convert filename: easy_english.md → easy-english.md (URL friendly)
            version_name = version_file.stem.replace('_', '-')
            target_file = target_chapter_folder / f"{version_name}.md"
            
            # Read source file
            content = version_file.read_text(encoding='utf-8')
            
            # Add frontmatter to hide from sidebar
            if not content.startswith('---'):
                content = f"""---
sidebar_class_name: hidden
displayed_sidebar: null
pagination_prev: null
pagination_next: null
---

{content}"""
            
            # Write to target
            target_file.write_text(content, encoding='utf-8')
            copied += 1
            print(f"  ✅ {version_name}.md")
    
    print()
    print("=" * 60)
    print(f"✅ Copied {copied} route files")
    print()
    print("Generated routes (examples):")
    print("  /docs/intro/easy-english")
    print("  /docs/intro/medium-urdu")
    print("  /docs/getting-started/hard-roman")
    print("=" * 60)
    
    return True

if __name__ == "__main__":
    success = copy_as_routes()
    exit(0 if success else 1)
