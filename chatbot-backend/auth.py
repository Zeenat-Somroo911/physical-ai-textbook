"""
Authentication endpoints for the chatbot backend.

Handles user registration, login, and profile management.
"""

import logging
from datetime import datetime, timedelta
from typing import Optional
from secrets import token_urlsafe

from fastapi import APIRouter, HTTPException, Depends, status, Header
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr, Field
import bcrypt
import jwt

from config import settings
from db import (
    create_user,
    get_user_by_email,
    get_user_by_id,
    update_user,
    update_last_login
)

logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/auth", tags=["authentication"])

# Security
security = HTTPBearer()

# JWT Configuration
JWT_SECRET = settings.openai_api_key[:32] if settings.openai_api_key else token_urlsafe(32)
JWT_ALGORITHM = "HS256"
JWT_EXPIRATION_HOURS = 24 * 7  # 7 days


# ============================================================================
# Pydantic Models
# ============================================================================

class RegisterRequest(BaseModel):
    """User registration request."""
    email: EmailStr
    password: str = Field(..., min_length=8, description="Password must be at least 8 characters")
    full_name: str = Field(..., min_length=1)
    preferences: dict = Field(default_factory=dict)

    class Config:
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "securepassword123",
                "full_name": "John Doe",
                "preferences": {
                    "software_experience": "intermediate",
                    "hardware_experience": "beginner",
                    "programming_languages": ["Python", "JavaScript"],
                    "robotics_background": "Some experience with Arduino"
                }
            }
        }


class LoginRequest(BaseModel):
    """User login request."""
    email: EmailStr
    password: str


class UserResponse(BaseModel):
    """User response model."""
    id: str
    email: str
    full_name: Optional[str]
    avatar_url: Optional[str]
    bio: Optional[str]
    preferences: dict
    is_verified: bool
    created_at: datetime


class AuthResponse(BaseModel):
    """Authentication response."""
    access_token: str
    token_type: str = "bearer"
    user: UserResponse


class ProfileUpdateRequest(BaseModel):
    """Profile update request."""
    full_name: Optional[str] = None
    avatar_url: Optional[str] = None
    bio: Optional[str] = None
    preferences: Optional[dict] = None


# ============================================================================
# Helper Functions
# ============================================================================

def hash_password(password: str) -> str:
    """
    Hash password using bcrypt.
    
    Args:
        password: Plain text password
        
    Returns:
        Hashed password
    """
    salt = bcrypt.gensalt()
    return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')


def verify_password(password: str, hashed: str) -> bool:
    """
    Verify password against hash.
    
    Args:
        password: Plain text password
        hashed: Hashed password
        
    Returns:
        True if password matches
    """
    return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))


def create_access_token(user_id: str, email: str) -> str:
    """
    Create JWT access token.
    
    Args:
        user_id: User UUID
        email: User email
        
    Returns:
        JWT token
    """
    expiration = datetime.utcnow() + timedelta(hours=JWT_EXPIRATION_HOURS)
    payload = {
        "sub": user_id,
        "email": email,
        "exp": expiration,
        "iat": datetime.utcnow()
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def verify_token(token: str) -> dict:
    """
    Verify and decode JWT token.
    
    Args:
        token: JWT token
        
    Returns:
        Decoded token payload
        
    Raises:
        HTTPException: If token is invalid
    """
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.InvalidTokenError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    """
    Get current authenticated user from token.
    
    Args:
        credentials: HTTP Bearer credentials
        
    Returns:
        User data dictionary
        
    Raises:
        HTTPException: If user not found or token invalid
    """
    token = credentials.credentials
    payload = verify_token(token)
    user_id = payload.get("sub")
    
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload"
        )
    
    user = await get_user_by_id(user_id)
    
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )
    
    if not user.get("is_active"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="User account is inactive"
        )
    
    return user


# ============================================================================
# Authentication Endpoints
# ============================================================================

@router.post("/register", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def register(request: RegisterRequest):
    """
    Register a new user.
    
    Args:
        request: Registration request with user details
        
    Returns:
        Authentication response with token and user data
        
    Raises:
        HTTPException: If email already exists or registration fails
    """
    try:
        # Check if user already exists
        existing_user = await get_user_by_email(request.email)
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )
        
        # Hash password
        password_hash = hash_password(request.password)
        
        # Create user
        user = await create_user(
            email=request.email,
            password_hash=password_hash,
            full_name=request.full_name,
            preferences=request.preferences
        )
        
        # Create access token
        access_token = create_access_token(str(user["id"]), user["email"])
        
        # Update last login
        await update_last_login(str(user["id"]))
        
        logger.info(f"User registered: {user['email']}")
        
        return AuthResponse(
            access_token=access_token,
            user=UserResponse(
                id=str(user["id"]),
                email=user["email"],
                full_name=user.get("full_name"),
                avatar_url=user.get("avatar_url"),
                bio=user.get("bio"),
                preferences=user.get("preferences", {}),
                is_verified=user.get("is_verified", False),
                created_at=user["created_at"]
            )
        )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Registration error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Registration failed. Please try again."
        )


@router.post("/login", response_model=AuthResponse)
async def login(request: LoginRequest):
    """
    Login user and return access token.
    
    Args:
        request: Login request with email and password
        
    Returns:
        Authentication response with token and user data
        
    Raises:
        HTTPException: If credentials are invalid
    """
    try:
        # Get user by email
        user = await get_user_by_email(request.email)
        
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )
        
        # Verify password
        if not verify_password(request.password, user["password_hash"]):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )
        
        # Check if user is active
        if not user.get("is_active"):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="User account is inactive"
            )
        
        # Create access token
        access_token = create_access_token(str(user["id"]), user["email"])
        
        # Update last login
        await update_last_login(str(user["id"]))
        
        logger.info(f"User logged in: {user['email']}")
        
        return AuthResponse(
            access_token=access_token,
            user=UserResponse(
                id=str(user["id"]),
                email=user["email"],
                full_name=user.get("full_name"),
                avatar_url=user.get("avatar_url"),
                bio=user.get("bio"),
                preferences=user.get("preferences", {}),
                is_verified=user.get("is_verified", False),
                created_at=user["created_at"]
            )
        )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Login failed. Please try again."
        )


@router.get("/me", response_model=UserResponse)
async def get_current_user_profile(current_user: dict = Depends(get_current_user)):
    """
    Get current user profile.
    
    Args:
        current_user: Current authenticated user (from dependency)
        
    Returns:
        User profile data
    """
    return UserResponse(
        id=str(current_user["id"]),
        email=current_user["email"],
        full_name=current_user.get("full_name"),
        avatar_url=current_user.get("avatar_url"),
        bio=current_user.get("bio"),
        preferences=current_user.get("preferences", {}),
        is_verified=current_user.get("is_verified", False),
        created_at=current_user["created_at"]
    )


@router.put("/profile", response_model=UserResponse)
async def update_profile(
    request: ProfileUpdateRequest,
    current_user: dict = Depends(get_current_user)
):
    """
    Update user profile.
    
    Args:
        request: Profile update request
        current_user: Current authenticated user
        
    Returns:
        Updated user profile
    """
    try:
        update_data = {}
        
        if request.full_name is not None:
            update_data["full_name"] = request.full_name
        if request.avatar_url is not None:
            update_data["avatar_url"] = request.avatar_url
        if request.bio is not None:
            update_data["bio"] = request.bio
        if request.preferences is not None:
            # Merge preferences
            current_prefs = current_user.get("preferences", {})
            current_prefs.update(request.preferences)
            update_data["preferences"] = current_prefs
        
        if update_data:
            success = await update_user(str(current_user["id"]), **update_data)
            if not success:
                raise HTTPException(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    detail="Failed to update profile"
                )
        
        # Get updated user
        updated_user = await get_user_by_id(str(current_user["id"]))
        
        logger.info(f"Profile updated for user: {current_user['email']}")
        
        return UserResponse(
            id=str(updated_user["id"]),
            email=updated_user["email"],
            full_name=updated_user.get("full_name"),
            avatar_url=updated_user.get("avatar_url"),
            bio=updated_user.get("bio"),
            preferences=updated_user.get("preferences", {}),
            is_verified=updated_user.get("is_verified", False),
            created_at=updated_user["created_at"]
        )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Profile update error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update profile"
        )


@router.post("/logout")
async def logout(current_user: dict = Depends(get_current_user)):
    """
    Logout user (client should discard token).
    
    Args:
        current_user: Current authenticated user
        
    Returns:
        Success message
    """
    # In a stateless JWT system, logout is handled client-side
    # by discarding the token. This endpoint is for consistency.
    logger.info(f"User logged out: {current_user['email']}")
    return {"message": "Logged out successfully"}


@router.post("/verify-token")
async def verify_token_endpoint(current_user: dict = Depends(get_current_user)):
    """
    Verify if token is valid.
    
    Args:
        current_user: Current authenticated user
        
    Returns:
        Token validity status
    """
    return {
        "valid": True,
        "user_id": str(current_user["id"]),
        "email": current_user["email"]
    }

