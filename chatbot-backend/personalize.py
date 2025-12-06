

import logging
from typing import Optional
from datetime import datetime

from fastapi import APIRouter, HTTPException, Depends, status, Query
from pydantic import BaseModel, Field
import openai
import hashlib

from config import settings
from auth import get_current_user
from db import (
    save_personalization,
    get_personalizations,
    delete_personalization,
    get_user_by_id
)

logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/personalize", tags=["personalization"])

# Initialize OpenAI
openai.api_key = settings.openai_api_key


# ============================================================================
# Pydantic Models
# ============================================================================

class PersonalizeRequest(BaseModel):
    """Personalization request model."""
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_path: Optional[str] = Field(None, description="Full path to chapter")
    content: str = Field(..., description="Original chapter content", min_length=1)


class PersonalizeResponse(BaseModel):
    """Personalization response model."""
    personalized_content: str = Field(..., description="Personalized content")
    chapter_id: str = Field(..., description="Chapter identifier")
    cached: bool = Field(False, description="Whether content was from cache")


class CheckPersonalizedRequest(BaseModel):
    """Check personalized content request."""
    chapter_id: str


class CheckPersonalizedResponse(BaseModel):
    """Check personalized content response."""
    has_personalized: bool
    content: Optional[str] = None


class ResetPersonalizeRequest(BaseModel):
    """Reset personalization request."""
    chapter_id: str


# ============================================================================
# Helper Functions
# ============================================================================

def get_user_profile_context(user: dict) -> str:
    """
    Build context string from user profile for personalization.
    
    Args:
        user: User data dictionary
        
    Returns:
        Context string for OpenAI prompt
    """
    preferences = user.get("preferences", {})
    
    # Check for direct background field (from new AuthModal)
    background = preferences.get("background", "")
    
    # Fallback to legacy fields if specific background not found
    software_exp = preferences.get("software_experience", "intermediate")
    hardware_exp = preferences.get("hardware_experience", "none")
    languages = preferences.get("programming_languages", [])
    
    context_parts = []
    
    # 1. Handle "Background" (The main selector we added)
    if background:
        context_parts.append(f"User Identity: {background}")
        
        if background == "Student":
            context_parts.append("Level: Beginner/Learner")
            context_parts.append("Goal: wants to understand basics clear concepts.")
            context_parts.append("Preference: Use simple analogies, step-by-step explanations, and avoid overwhelming jargon.")
        elif background == "Engineer":
            context_parts.append("Level: Professional")
            context_parts.append("Goal: wants practical implementation details.")
            context_parts.append("Preference: Focus on efficiency, code structure, and industry standards. Skip basic definitions.")
        elif background == "Researcher":
            context_parts.append("Level: Academic/Advanced")
            context_parts.append("Goal: wants deep theoretical understanding.")
            context_parts.append("Preference: Cite papers/methods, focus on algorithms and mathematical foundations.")
        elif background == "Hobbyist":
            context_parts.append("Level: Maker/Enthusiast")
            context_parts.append("Goal: wants to build things.")
            context_parts.append("Preference: Focus on hardware integration, practical wiring, and 'how-to' guides.")
            
    # 2. Legacy/Detailed fields (keep for backward compatibility or future detailed forms)
    else:
        # Software experience
        if software_exp == "beginner":
            context_parts.append("User is a BEGINNER in software development (0-1 years)")
        elif software_exp == "intermediate":
            context_parts.append("User is INTERMEDIATE in software development (2-5 years)")
        elif software_exp == "advanced":
            context_parts.append("User is ADVANCED in software development (5+ years)")
        
        # Hardware experience
        if hardware_exp == "none":
            context_parts.append("User has NO hardware/robotics experience")
        elif hardware_exp == "beginner":
            context_parts.append("User is a BEGINNER in hardware/robotics")
        elif hardware_exp == "intermediate":
            context_parts.append("User is INTERMEDIATE in hardware/robotics")
        elif hardware_exp == "advanced":
            context_parts.append("User is ADVANCED in hardware/robotics")
    
    # Programming languages (always useful if present)
    if languages:
        context_parts.append(f"User knows: {', '.join(languages)}")
    
    return "\n".join(context_parts)


def generate_personalization_prompt(content: str, user_context: str) -> str:
    """
    Generate prompt for Gemini to personalize content with VISIBLE changes.
    """
    prompt = f"""You are an expert educational content personalizer. Your task is to COMPLETELY REWRITE this robotics textbook content to make it much more beginner-friendly and detailed.

USER PROFILE:
{user_context}

CRITICAL REQUIREMENTS - YOU MUST:
1. Make the content AT LEAST 40% LONGER than the original
2. Add 2-3 REAL-WORLD EXAMPLES for every major concept
3. Add ANALOGIES that relate technical concepts to everyday life
4. Explain EVERY technical term in simple language (in parentheses after first use)
5. Add step-by-step breakdowns for complex processes
6. Use simple, conversational language like explaining to a friend

EXAMPLE OF GOOD PERSONALIZATION:
Original: "ROS2 uses a publish-subscribe architecture."
Personalized: "ROS2 uses a publish-subscribe architecture (think of it like YouTube - some people create videos and 'publish' them, while others 'subscribe' to watch them). In robotics, this means one part of your robot can send out information (like sensor data), and other parts that need that information can subscribe to receive it automatically. For example, your robot's camera might publish images, and the navigation system subscribes to those images to know where to go."

ORIGINAL CONTENT:
{content}

REWRITTEN CONTENT (make it OBVIOUSLY different, much more detailed, with examples and analogies):"""
    
    return prompt


async def personalize_content(content: str, user: dict) -> str:
    """
    Personalize content using OpenAI based on user profile.
    
    Args:
        content: Original content
        user: User data dictionary
        
    Returns:
        Personalized content
    """
    try:
        user_context = get_user_profile_context(user)
        prompt = generate_personalization_prompt(content, user_context)
        
        # Check for Gemini first (User requested free tier)
        if settings.gemini_api_key:
            import asyncio
            import google.generativeai as genai
            
            genai.configure(api_key=settings.gemini_api_key)
            model = genai.GenerativeModel(settings.gemini_model)
            
            logger.info(f"Sending request to Gemini ({settings.gemini_model})... (Prompt length: {len(prompt)} chars)")
            
            # Configure safety settings for educational content
            safety_settings = {
                'HARM_CATEGORY_HARASSMENT': 'BLOCK_NONE',
                'HARM_CATEGORY_HATE_SPEECH': 'BLOCK_NONE',
                'HARM_CATEGORY_SEXUALLY_EXPLICIT': 'BLOCK_NONE',
                'HARM_CATEGORY_DANGEROUS_CONTENT': 'BLOCK_NONE',
            }
            
            # Run the blocking Gemini call in a thread pool to avoid blocking the event loop
            def _generate_sync():
                return model.generate_content(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        temperature=0.7,
                        max_output_tokens=2048,
                    ),
                    safety_settings=safety_settings
                )
            
            try:
                response = await asyncio.to_thread(_generate_sync)
                logger.info("Gemini response received successfully.")
                
                # Check if response was blocked
                if not response.text:
                    logger.warning(f"Gemini blocked response. Finish reason: {response.candidates[0].finish_reason if response.candidates else 'unknown'}")
                    # Return original content if blocked
                    return content
                    
                personalized = response.text.strip()
            except Exception as e:
                logger.error(f"Gemini generation error: {e}")
                # Return original content on error
                return content
            
        elif settings.openai_api_key:
            # Fallback to OpenAI if configured
            response = openai.chat.completions.create(
                model=settings.openai_model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are an expert educational content personalizer. Adjust technical content to match user's experience level while maintaining accuracy and completeness."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.7,
                max_tokens=4000,
            )
            personalized = response.choices[0].message.content.strip()
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="No AI service configured (Gemini or OpenAI missing)"
            )
        
        # Remove markdown code blocks if present
        if personalized.startswith("```"):
            lines = personalized.split("\n")
            if len(lines) > 2:
                personalized = "\n".join(lines[1:-1])
        
        return personalized
    
    except Exception as e:
        logger.error(f"Error personalizing content: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to personalize content: {str(e)}"
        )


def get_content_hash(content: str) -> str:
    """
    Generate hash for content to detect changes.
    
    Args:
        content: Content string
        
    Returns:
        SHA-256 hash
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


# ============================================================================
# Endpoints
# ============================================================================

@router.post("", response_model=PersonalizeResponse)
async def personalize_content_endpoint(
    request: PersonalizeRequest,
    current_user: dict = Depends(get_current_user)
):
    """
    Personalize chapter content based on user profile.
    
    Args:
        request: Personalization request with chapter content
        current_user: Current authenticated user
        
    Returns:
        Personalized content
    """
    try:
        user_id = str(current_user["id"])
        chapter_id = request.chapter_id
        
        # Check if personalized content already exists
        existing = await get_personalizations(
            user_id=user_id,
            chapter_id=chapter_id,
            content_type="personalized"
        )
        
        if existing:
            # Check if content hash matches (content hasn't changed)
            content_hash = get_content_hash(request.content)
            existing_hash = existing[0].get("metadata", {}).get("content_hash")
            
            if existing_hash == content_hash:
                # Return cached content
                logger.info(f"Returning cached personalized content for chapter {chapter_id}")
                return PersonalizeResponse(
                    personalized_content=existing[0]["content"],
                    chapter_id=chapter_id,
                    cached=True
                )
        
        # Get full user data (including preferences)
        full_user = await get_user_by_id(user_id)
        if not full_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )
        
        # Generate personalized content
        logger.info(f"Personalizing content for chapter {chapter_id}, user {user_id}")
        personalized_content = await personalize_content(request.content, full_user)
        
        # Calculate content hash
        content_hash = get_content_hash(request.content)
        
        # Save to database (serialize metadata to JSON)
        import json
        await save_personalization(
            user_id=user_id,
            chapter_id=chapter_id,
            content=personalized_content,
            content_type="personalized",
            chapter_path=request.chapter_path,
            metadata=json.dumps({
                "content_hash": content_hash,
                "original_length": len(request.content),
                "personalized_length": len(personalized_content),
                "personalized_at": datetime.utcnow().isoformat(),
            })
        )
        
        logger.info(f"Personalized content saved for chapter {chapter_id}")
        
        return PersonalizeResponse(
            personalized_content=personalized_content,
            chapter_id=chapter_id,
            cached=False
        )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Personalization error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to personalize content: {str(e)}"
        )


@router.get("/check", response_model=CheckPersonalizedResponse)
async def check_personalized(
    chapter_id: str = Query(..., description="Chapter identifier"),
    current_user: dict = Depends(get_current_user)
):
    """
    Check if personalized content exists for a chapter.
    
    Args:
        chapter_id: Chapter identifier
        current_user: Current authenticated user
        
    Returns:
        Whether personalized content exists and the content if available
    """
    try:
        user_id = str(current_user["id"])
        
        personalizations = await get_personalizations(
            user_id=user_id,
            chapter_id=chapter_id,
            content_type="personalized"
        )
        
        if personalizations and len(personalizations) > 0:
            return CheckPersonalizedResponse(
                has_personalized=True,
                content=personalizations[0]["content"]
            )
        
        return CheckPersonalizedResponse(
            has_personalized=False,
            content=None
        )
    
    except Exception as e:
        logger.error(f"Error checking personalized content: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to check personalized content: {str(e)}"
        )


@router.post("/reset")
async def reset_personalization(
    request: ResetPersonalizeRequest,
    current_user: dict = Depends(get_current_user)
):
    """
    Reset personalized content to original.
    
    Args:
        request: Reset request with chapter ID
        current_user: Current authenticated user
        
    Returns:
        Success message
    """
    try:
        user_id = str(current_user["id"])
        chapter_id = request.chapter_id
        
        # Get personalizations
        personalizations = await get_personalizations(
            user_id=user_id,
            chapter_id=chapter_id,
            content_type="personalized"
        )
        
        # Delete personalized content
        for personalization in personalizations:
            await delete_personalization(personalization["id"])
        
        logger.info(f"Reset personalization for chapter {chapter_id}, user {user_id}")
        
        return {"message": "Personalization reset successfully"}
    
    except Exception as e:
        logger.error(f"Error resetting personalization: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to reset personalization: {str(e)}"
        )


@router.get("/list")
async def list_personalizations(current_user: dict = Depends(get_current_user)):
    """
    List all personalized chapters for current user.
    
    Args:
        current_user: Current authenticated user
        
    Returns:
        List of personalized chapters
    """
    try:
        user_id = str(current_user["id"])
        
        personalizations = await get_personalizations(
            user_id=user_id,
            content_type="personalized"
        )
        
        return {
            "personalizations": [
                {
                    "chapter_id": p["chapter_id"],
                    "chapter_path": p.get("chapter_path"),
                    "updated_at": p["updated_at"].isoformat() if isinstance(p["updated_at"], datetime) else p["updated_at"],
                }
                for p in personalizations
            ]
        }
    
    except Exception as e:
        logger.error(f"Error listing personalizations: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to list personalizations: {str(e)}"
        )

