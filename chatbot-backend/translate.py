"""
Translation Endpoints

Translates chapter content to Urdu using OpenAI API.
Falls back to Google Translate API (free) if OpenAI fails.
Caches translations to save API costs.
"""

import logging
from typing import Optional
from datetime import datetime
import hashlib

from fastapi import APIRouter, HTTPException, Depends, status, Query
from pydantic import BaseModel, Field
import openai
import requests

from config import settings
from auth import get_current_user
from db import (
    save_translation,
    get_translation,
    delete_expired_translations,
    get_user_by_id
)

logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/translate", tags=["translation"])

# Initialize OpenAI
openai.api_key = settings.openai_api_key


# ============================================================================
# Pydantic Models
# ============================================================================

class TranslateRequest(BaseModel):
    """Translation request model."""
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_path: Optional[str] = Field(None, description="Full path to chapter")
    content: str = Field(..., description="Content to translate", min_length=1)
    target_language: str = Field(default="ur", description="Target language code (ur for Urdu)")


class TranslateResponse(BaseModel):
    """Translation response model."""
    translated_content: str = Field(..., description="Translated content")
    chapter_id: str = Field(..., description="Chapter identifier")
    target_language: str = Field(..., description="Target language")
    cached: bool = Field(False, description="Whether translation was from cache")
    translation_method: str = Field("openai", description="Method used: openai or google")


class TranslationCheckResponse(BaseModel):
    """Translation check response."""
    has_translation: bool = Field(..., description="Whether translation exists")
    content: Optional[str] = Field(None, description="Cached translation if exists")


# ============================================================================
# Helper Functions
# ============================================================================

def get_content_hash(content: str) -> str:
    """
    Generate hash for content to detect changes.
    
    Args:
        content: Content string
        
    Returns:
        SHA-256 hash
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def translate_with_openai(content: str, target_lang: str = "ur") -> str:
    """
    Translate content using OpenAI API.
    
    Args:
        content: Content to translate
        target_lang: Target language code
        
    Returns:
        Translated content
    """
    try:
        # Map language codes to names
        lang_names = {
            "ur": "Urdu",
            "en": "English",
            "ar": "Arabic",
            "hi": "Hindi"
        }
        target_lang_name = lang_names.get(target_lang, "Urdu")
        
        prompt = f"""Translate the following technical content to {target_lang_name}. 

IMPORTANT INSTRUCTIONS:
1. Maintain ALL markdown formatting (headings, code blocks, lists, etc.)
2. Keep code blocks in English (do NOT translate code)
3. Keep technical terms in English if commonly used (e.g., ROS, API, Python)
4. Translate explanations and descriptions naturally
5. Preserve all special characters and formatting
6. Maintain the same structure and hierarchy

CONTENT TO TRANSLATE:
{content}

Return ONLY the translated content with all formatting preserved."""

        response = openai.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {
                    "role": "system",
                    "content": f"You are an expert translator specializing in technical content translation to {target_lang_name}. You maintain formatting and technical accuracy."
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            temperature=0.3,  # Lower temperature for more consistent translations
            max_tokens=4000,
        )
        
        translated = response.choices[0].message.content.strip()
        
        # Remove markdown code blocks if present
        if translated.startswith("```"):
            lines = translated.split("\n")
            if len(lines) > 2:
                translated = "\n".join(lines[1:-1])
        
        return translated
    
    except Exception as e:
        logger.error(f"OpenAI translation error: {e}", exc_info=True)
        raise


def translate_with_google(content: str, target_lang: str = "ur") -> str:
    """
    Translate content using Google Translate API (free).
    This is a fallback method.
    
    Note: This uses the free Google Translate web interface.
    For production, consider using the official Google Cloud Translation API.
    
    Args:
        content: Content to translate
        target_lang: Target language code
        
    Returns:
        Translated content
    """
    try:
        # Using Google Translate web API (free, but rate-limited)
        url = "https://translate.googleapis.com/translate_a/single"
        params = {
            "client": "gtx",
            "sl": "en",  # Source language: English
            "tl": target_lang,  # Target language
            "dt": "t",
            "q": content
        }
        
        response = requests.get(url, params=params, timeout=10)
        response.raise_for_status()
        
        result = response.json()
        
        # Extract translated text from nested JSON structure
        if result and len(result) > 0 and len(result[0]) > 0:
            translated_parts = [item[0] for item in result[0] if item[0]]
            translated = "".join(translated_parts)
            return translated
        else:
            raise ValueError("Invalid response from Google Translate")
    
    except Exception as e:
        logger.error(f"Google Translate error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}"
        )


def translate_content(content: str, target_lang: str = "ur") -> tuple[str, str]:
    """
    Translate content using OpenAI first, fallback to Google Translate.
    
    Args:
        content: Content to translate
        target_lang: Target language code
        
    Returns:
        Tuple of (translated_content, method_used)
    """
    # Try OpenAI first
    try:
        translated = translate_with_openai(content, target_lang)
        return translated, "openai"
    except Exception as e:
        logger.warning(f"OpenAI translation failed, trying Google Translate: {e}")
        
        # Fallback to Google Translate
        try:
            translated = translate_with_google(content, target_lang)
            return translated, "google"
        except Exception as e2:
            logger.error(f"Both translation methods failed: {e2}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Translation failed with both methods: {str(e2)}"
            )


# ============================================================================
# Endpoints
# ============================================================================

@router.post("", response_model=TranslateResponse)
async def translate_content_endpoint(
    request: TranslateRequest,
    current_user: dict = Depends(get_current_user)
):
    """
    Translate chapter content to target language.
    
    Caches translations to save API costs.
    """
    user_id = current_user.get("id")
    chapter_id = request.chapter_id
    target_lang = request.target_language or "ur"
    
    # Check cache first
    content_hash = get_content_hash(request.content)
    existing = await get_translation(
        chapter_id=chapter_id,
        language_code=target_lang
    )
    
    if existing:
        existing_hash = existing.get("original_content_hash")
        if existing_hash == content_hash:
            # Return cached translation
            logger.info(f"Returning cached translation for chapter {chapter_id}")
            return TranslateResponse(
                translated_content=existing["translated_content"],
                chapter_id=chapter_id,
                target_language=target_lang,
                cached=True,
                translation_method=existing.get("translation_model", "openai")
            )
    
    # Generate translation
    logger.info(f"Translating content for chapter {chapter_id} to {target_lang}")
    try:
        translated_content, method = translate_content(request.content, target_lang)
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Translation error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to translate content: {str(e)}"
        )
    
    # Save to database
    await save_translation(
        chapter_id=chapter_id,
        translated_content=translated_content,
        language_code=target_lang,
        chapter_path=request.chapter_path,
        original_content_hash=content_hash,
        translation_model=method
    )
    
    return TranslateResponse(
        translated_content=translated_content,
        chapter_id=chapter_id,
        target_language=target_lang,
        cached=False,
        translation_method=method
    )


@router.get("/check", response_model=TranslationCheckResponse)
async def check_translation(
    chapter_id: str = Query(..., description="Chapter identifier"),
    target_lang: str = Query(default="ur", description="Target language"),
    current_user: Optional[dict] = Depends(get_current_user)
):
    """
    Check if translation exists for a chapter.
    """
    try:
        translation = await get_translation(
            chapter_id=chapter_id,
            language_code=target_lang
        )
        
        if translation:
            return TranslationCheckResponse(
                has_translation=True,
                content=translation["translated_content"]
            )
        else:
            return TranslationCheckResponse(
                has_translation=False,
                content=None
            )
    except Exception as e:
        logger.error(f"Error checking translation: {e}", exc_info=True)
        return TranslationCheckResponse(
            has_translation=False,
            content=None
        )


@router.delete("")
async def delete_translation_endpoint(
    chapter_id: str = Query(..., description="Chapter identifier"),
    target_lang: str = Query(default="ur", description="Target language"),
    current_user: dict = Depends(get_current_user)
):
    """
    Delete cached translation.
    Note: This deletes expired translations. For specific deletion, 
    we would need to add a delete_translation function to db.py.
    """
    try:
        # For now, we'll just clean expired translations
        deleted_count = await delete_expired_translations()
        return {
            "message": "Expired translations cleaned",
            "deleted_count": deleted_count
        }
    except Exception as e:
        logger.error(f"Error deleting translation: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete translation: {str(e)}"
        )


@router.get("/list")
async def list_translations(
    target_lang: str = Query(default="ur", description="Target language"),
    current_user: dict = Depends(get_current_user)
):
    """
    List all translations.
    Note: This is a simplified version. For full implementation,
    we would need a get_all_translations function in db.py.
    """
    try:
        # For now, return a message indicating this needs implementation
        return {
            "message": "Translation listing requires additional database function",
            "target_language": target_lang
        }
    except Exception as e:
        logger.error(f"Error listing translations: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to list translations: {str(e)}"
        )

