

import os
from typing import Optional
from pydantic_settings import BaseSettings
from pydantic import Field


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""
    
    # API Keys
    openai_api_key: Optional[str] = Field(None, env="OPENAI_API_KEY", description="OpenAI API key")
    qdrant_api_key: Optional[str] = Field(None, env="QDRANT_API_KEY", description="Qdrant API key")
    
    # Database
    database_url: Optional[str] = Field(
        None,
        env="DATABASE_URL",
        description="PostgreSQL database URL (Neon Postgres)"
    )
    
    # Qdrant Configuration
    qdrant_url: str = Field(
        default="http://localhost:6333",
        env="QDRANT_URL",
        description="Qdrant vector database URL"
    )
    qdrant_collection_name: str = Field(
        default="textbook_embeddings",
        env="QDRANT_COLLECTION_NAME",
        description="Qdrant collection name for embeddings"
    )
    
    # OpenAI Configuration
    openai_model: str = Field(
        default="gpt-3.5-turbo",
        env="OPENAI_MODEL",
        description="OpenAI model for chat completions"
    )
    openai_embedding_model: str = Field(
        default="text-embedding-3-small",
        env="OPENAI_EMBEDDING_MODEL",
        description="OpenAI model for embeddings"
    )

    # Gemini Configuration
    gemini_api_key: Optional[str] = Field(None, env="GEMINI_API_KEY", description="Google Gemini API key")
    gemini_model: str = Field(
        default="gemini-2.5-flash",
        env="GEMINI_MODEL",
        description="Google Gemini model"
    )
    gemini_embedding_model: str = Field(
        default="models/embedding-001",
        env="GEMINI_EMBEDDING_MODEL",
        description="Google Gemini embedding model"
    )

    # HuggingFace Configuration (Inference API)
    hf_embedding_model: str = Field(
        default="sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2",
        env="HF_EMBEDDING_MODEL",
        description="HuggingFace sentence-transformer model ID"
    )
    hf_token: Optional[str] = Field(
        None,
        env="HF_TOKEN", 
        description="HuggingFace Access Token for Inference API"
    )
    
    # Application Configuration
    app_name: str = Field(
        default="Physical AI Textbook Chatbot",
        env="APP_NAME",
        description="Application name"
    )
    app_version: str = Field(
        default="1.0.0",
        env="APP_VERSION",
        description="Application version"
    )
    debug: bool = Field(
        default=False,
        env="DEBUG",
        description="Debug mode"
    )
    
    mock_mode: bool = Field(
        default=False,
        env="MOCK_MODE",
        description="Enable mock mode (no DB/OpenAI connection)"
    )
    
    # CORS Configuration
    cors_origins: list[str] = Field(
        default=["http://localhost:3000", "http://localhost:5173"],
        env="CORS_ORIGINS",
        description="Allowed CORS origins"
    )
    
    # RAG Configuration
    max_context_length: int = Field(
        default=4000,
        env="MAX_CONTEXT_LENGTH",
        description="Maximum context length for RAG"
    )
    top_k_results: int = Field(
        default=5,
        env="TOP_K_RESULTS",
        description="Number of top results to retrieve from vector DB"
    )
    
    # Logging
    log_level: str = Field(
        default="INFO",
        env="LOG_LEVEL",
        description="Logging level"
    )
    
    class Config:
        """Pydantic configuration."""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False
        extra = "ignore"


# Global settings instance
settings = Settings()

