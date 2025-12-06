"""
Pydantic models for request/response validation.

Defines data structures for API endpoints.
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ChatMessage(BaseModel):
    """Individual chat message model."""
    
    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content", min_length=1)
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    
    message: str = Field(
        ...,
        description="User's message/query",
        min_length=1,
        max_length=2000
    )
    conversation_id: Optional[str] = Field(
        None,
        description="Optional conversation ID for context"
    )
    selected_text: Optional[str] = Field(
        None,
        description="Optional selected text from the page for context"
    )
    use_rag: bool = Field(
        default=True,
        description="Whether to use RAG for enhanced responses"
    )
    
    class Config:
        """Pydantic configuration."""
        json_schema_extra = {
            "example": {
                "message": "What is ROS 2?",
                "conversation_id": "conv_123",
                "selected_text": "ROS 2 is a robotics framework...",
                "use_rag": True
            }
        }


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    
    response: str = Field(..., description="Assistant's response")
    conversation_id: str = Field(..., description="Conversation ID")
    sources: List[str] = Field(
        default_factory=list,
        description="Source documents used for RAG"
    )
    tokens_used: Optional[int] = Field(
        None,
        description="Number of tokens used in the response"
    )
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    
    class Config:
        """Pydantic configuration."""
        json_schema_extra = {
            "example": {
                "response": "ROS 2 is the Robot Operating System 2...",
                "conversation_id": "conv_123",
                "sources": ["/docs/module-01-ros2/introduction"],
                "tokens_used": 150,
                "timestamp": "2024-01-01T12:00:00Z"
            }
        }


class HealthResponse(BaseModel):
    """Health check response model."""
    
    status: str = Field(..., description="Service status")
    version: str = Field(..., description="Application version")
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    services: dict = Field(
        default_factory=dict,
        description="Status of external services"
    )


class IngestRequest(BaseModel):
    """Request model for content ingestion."""
    
    content: str = Field(
        ...,
        description="Content to ingest",
        min_length=1
    )
    metadata: dict = Field(
        default_factory=dict,
        description="Metadata for the content"
    )
    document_id: Optional[str] = Field(
        None,
        description="Optional document ID"
    )


class IngestResponse(BaseModel):
    """Response model for content ingestion."""
    
    success: bool = Field(..., description="Whether ingestion was successful")
    document_id: str = Field(..., description="Document ID")
    chunks_created: int = Field(..., description="Number of chunks created")
    message: str = Field(..., description="Status message")


class ErrorResponse(BaseModel):
    """Error response model."""
    
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Error details")
    timestamp: datetime = Field(default_factory=datetime.utcnow)

