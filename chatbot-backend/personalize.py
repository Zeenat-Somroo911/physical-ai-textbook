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


class PersonalizeRequest(BaseModel):
    """Personalization request model."""
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_path: Optional[str] = Field(None, description="Full path to chapter")
    content: Optional[str] = Field(None, description="Original chapter content (not used for pre-generated)")
    language: str = Field("english", description="Language: english, urdu, roman_urdu")
    difficulty: str = Field("medium", description="Difficulty: easy, medium, hard")


class PersonalizeResponse(BaseModel):
    """Personalization response model."""
    personalized_content: str = Field(..., description="Personalized content")
    chapter_id: str = Field(..., description="Chapter identifier")
    cached: bool = Field(False, description="Whether content was from cache")
    source: str = Field("pre-generated", description="Content source: pre-generated or original-docs")
    language: Optional[str] = Field(None, description="Language used")
    difficulty: Optional[str] = Field(None, description="Difficulty used")


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


def get_user_profile_context(user: dict) -> str:
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
    Serve pre-generated personalized content based on language and difficulty.
    
    Args:
        request: Personalization request with chapter_id, language, difficulty
        current_user: Current authenticated user
        
    Returns:
        Personalized content from generated-content folder
    """
    try:
        user_id = str(current_user["id"])
        chapter_id = request.chapter_id
        language = request.language
        difficulty = request.difficulty
        
        # Map language to file naming convention
        language_map = {
            "english": "english",
            "urdu": "urdu",
            "roman_urdu": "roman"
        }
        
        lang_code = language_map.get(language, "english")

        from pathlib import Path
        import os
        
        # Get the project root (where chatbot-backend folder is)
        backend_dir = Path(__file__).parent
        project_root = backend_dir.parent
        generated_content_dir = project_root / "generated-content"
        
        # Build path to the specific version
        content_file = generated_content_dir / chapter_id / f"{difficulty}_{lang_code}.md"
        
        logger.info(f"Looking for pre-generated content at: {content_file}")
        
        # Track which source we're using
        content_source = "pre-generated"
        
        # Check if file exists
        if not content_file.exists():
            logger.warning(f"Pre-generated content not found: {content_file}")
            content_source = "original-docs"
            
            # If not found, try to read original docs file as fallback
            docs_file = project_root / "docs" / f"{chapter_id}.md"
            if docs_file.exists():
                logger.info(f"Falling back to original docs: {docs_file}")
                with open(docs_file, 'r', encoding='utf-8') as f:
                    personalized_content = f.read()
            else:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail=f"Content not found for {chapter_id} with {language}/{difficulty}"
                )
        else:
            # Read pre-generated content
            with open(content_file, 'r', encoding='utf-8') as f:
                personalized_content = f.read()
        
        logger.info(f"Loaded content ({len(personalized_content)} chars) for {chapter_id}")
        
        # Save to database for caching (optional, for tracking)
        import json
        await save_personalization(
            user_id=user_id,
            chapter_id=chapter_id,
            content=personalized_content,
            content_type="personalized",
            chapter_path=request.chapter_path,
            metadata=json.dumps({
                "language": language,
                "difficulty": difficulty,
                "source": content_source,
                "file_path": str(content_file if content_source == "pre-generated" else docs_file),
                "personalized_at": datetime.utcnow().isoformat(),
            })
        )
        
        logger.info(f"Served {content_source} content for chapter {chapter_id}")
        
        return PersonalizeResponse(
            personalized_content=personalized_content,
            chapter_id=chapter_id,
            cached=False,
            source=content_source,
            language=language,
            difficulty=difficulty
        )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Personalization error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to load personalized content: {str(e)}"
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

