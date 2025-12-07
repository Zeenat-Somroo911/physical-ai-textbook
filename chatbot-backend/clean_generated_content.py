"""
Clean Generated Content - Remove Broken JSX Tags

Removes incomplete JSX tags from generated markdown files
that cause MDX build errors.
"""

import os
import re
from pathlib import Path

def clean_jsx_tags(content):
    """Remove broken JSX tags from markdown content"""
    
    # Remove incomplete JSX opening tags
    content = re.sub(r'<PersonalizedContent[^>]*(?:>)?', '', content)
    content = re.sub(r'<PersonalizeButton[^>]*(?:>)?', '', content)
    content = re.sub(r'</PersonalizedContent>', '', content)
    content = re.sub(r'</PersonalizeButton>', '', content)
    
    # Remove any other incomplete JSX tags (< followed by capital letter without closing >)
    content = re.sub(r'<[A-Z][a-zA-Z]*(?:\s|$)', '', content)
    
    return content

def clean_all_files():
    """Clean all generated markdown files"""
    
    project_root = Path(__file__).parent.parent
    generated_dir = project_root / "generated-content"
    
    if not generated_dir.exists():
        print(f"❌ {generated_dir} not found")
        return
    
    cleaned = 0
    errors = 0
    
    print("=" * 60)
    print("Cleaning Generated Content Files")
    print("=" * 60)
    
    for md_file in generated_dir.rglob("*.md"):
        try:
            # Read original
            original = md_file.read_text(encoding='utf-8')
            
            # Clean JSX tags
            cleaned_content = clean_jsx_tags(original)
            
            # Only write if changed
            if cleaned_content != original:
                md_file.write_text(cleaned_content, encoding='utf-8')
                cleaned += 1
                print(f"✅ Cleaned: {md_file.relative_to(generated_dir)}")
            
        except Exception as e:
            errors += 1
            print(f"❌ Error: {md_file.name}: {e}")
    
    print()
    print("=" * 60)
    print(f"✅ Cleaned {cleaned} files")
    if errors > 0:
        print(f"❌ {errors} errors")
    print("=" * 60)

if __name__ == "__main__":
    clean_all_files()
