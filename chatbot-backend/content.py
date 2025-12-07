"""
Content serving endpoint for pre-generated versions.

Simple GET endpoint that serves content based on language and difficulty.
No authentication required - this is for public content access.
"""

import logging
from pathlib import Path
from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import PlainTextResponse

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/content", tags=["content"])


@router.get("/{chapter_path:path}")
async def get_content(
    chapter_path: str,
    lang: str = Query("english", description="Language: english, urdu, roman_urdu"),
    diff: str = Query("medium", description="Difficulty: easy, medium, hard")
):
    """
    Serve pre-generated content or fallback to original docs.
    
    Args:
        chapter_path: Path to chapter (e.g., "module-01-ros2/01-introduction")
        lang: Language preference
        diff: Difficulty level
        
    Returns:
        Content as plain text (markdown)
    """
    try:
        # Map language to file naming convention
        language_map = {
            "english": "english",
            "urdu": "urdu",
            "roman_urdu": "roman"
        }
        
        lang_code = language_map.get(lang, "english")
        
        # Get project root
        backend_dir = Path(__file__).parent
        project_root = backend_dir.parent
        
        # Try pre-generated content first
        generated_file = project_root / "generated-content" / chapter_path / f"{diff}_{lang_code}.md"
        
        logger.info(f"Looking for: {generated_file}")
        
        if generated_file.exists():
            logger.info(f"✅ Serving pre-generated: {generated_file}")
            with open(generated_file, 'r', encoding='utf-8') as f:
                content = f.read()
            return PlainTextResponse(content, headers={
                "X-Content-Source": "pre-generated",
                "X-Language": lang,
                "X-Difficulty": diff
            })
        
        # Fallback to original docs
        docs_file = project_root / "docs" / f"{chapter_path}.md"
        
        logger.info(f"Pre-generated not found, trying: {docs_file}")
        
        if docs_file.exists():
            logger.info(f"📄 Serving original docs: {docs_file}")
            with open(docs_file, 'r', encoding='utf-8') as f:
                content = f.read()
            return PlainTextResponse(content, headers={
                "X-Content-Source": "original-docs",
                "X-Language": "english",
                "X-Difficulty": "original"
            })
        
        # Nothing found
        raise HTTPException(
            status_code=404,
            detail=f"Content not found for: {chapter_path}"
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error serving content: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Failed to load content: {str(e)}"
        )
