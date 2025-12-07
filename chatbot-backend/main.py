"""
FastAPI backend for RAG-powered chatbot.

Provides endpoints for chat, content ingestion, and health checks.
"""

import logging
import uuid
from typing import List, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import openai
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import asyncpg
from dotenv import load_dotenv
import os

# Load environment variables FIRST
load_dotenv(override=True)

from config import settings
from models import (
    ChatRequest,
    ChatResponse,
    HealthResponse,
    IngestRequest,
    IngestResponse,
    ErrorResponse
)
from db import init_db_pool, close_db_pool
from auth import router as auth_router
from personalize import router as personalize_router
from translate import router as translate_router

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level.upper()),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

import requests
import time
from huggingface_hub import InferenceClient

# Initialize AI Clients
if settings.openai_api_key:
    openai.api_key = settings.openai_api_key

# Import Gemini unconditionally to avoid issues
import google.generativeai as genai

# Configure Gemini using direct env loading (like generate_all_versions.py)
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if GEMINI_API_KEY:
    genai.configure(api_key=GEMINI_API_KEY)
    logger.info(f"Gemini API configured at module level with model: {settings.gemini_model}")

# Global clients (initialized in lifespan)
qdrant_client: Optional[QdrantClient] = None # Qdrant Client
hf_client: Optional[InferenceClient] = None # HF Client (Inference API)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager."""
    global qdrant_client, hf_client
    
    # Startup
    logger.info("Starting up application...")
    
    # 1. Log Configuration Status
    logger.info(f"Qdrant URL: {settings.qdrant_url}")
    logger.info(f"Qdrant Key: {'***' if settings.qdrant_api_key else 'MISSING'}")
    logger.info(f"Gemini Key: {'***' if settings.gemini_api_key else 'MISSING'}")
    logger.info(f"HF Token: {'***' if settings.hf_token else 'MISSING'}")

    try:
        # 2. Initialize HF Client
        if not settings.hf_token:
            logger.warning("HF_TOKEN is missing. Embeddings will fail.")
        else:
            hf_client = InferenceClient(token=settings.hf_token, timeout=30.0)
            logger.info("HF Inference Client initialized")
        
        # 3. Initialize Qdrant client
        # We DO NOT fallback to None silently anymore.
        if settings.mock_mode:
            logger.warning("Running in MOCK MODE - Qdrant disabled")
            qdrant_client = None
        else:
            if settings.qdrant_api_key:
                qdrant_client = QdrantClient(
                    url=settings.qdrant_url,
                    api_key=settings.qdrant_api_key
                )
            else:
                qdrant_client = QdrantClient(url=settings.qdrant_url)
            
            # Connection Test
            try:
                collections = qdrant_client.get_collections()
                logger.info(f"Connected to Qdrant successfully. Found {len(collections.collections)} collections.")
            except Exception as e:
                logger.error(f"Failed Qdrant Connection Test: {e}")
                raise e  # Propagate error to crash startup if DB is down (better than 500s later)

            # Determine Vector Size
            vector_size = 384 
            logger.info(f"Using HuggingFace Embeddings ({vector_size} dimensions)")
            
            # Ensure proper collection exists
            try:
                qdrant_client.get_collection(settings.qdrant_collection_name)
                logger.info(f"Qdrant collection '{settings.qdrant_collection_name}' exists")
            except Exception:
                # Only create if strictly needed (ingest usually does this)
                logger.info(f"Collection '{settings.qdrant_collection_name}' not found during startup.")

    except Exception as e:
        logger.critical(f"Startup Failed: {e}", exc_info=True)
        # We don't raise here to allow app to start for Health Checks, 
        # but qdrant_client will be None, causing 500s on chat.
        # Ideally, we should raise.


        
        # Initialize database pool using db.py
        await init_db_pool()
        logger.info("Database connection pool created")
        
        # Create tables if they don't exist
        await create_tables()
        
        logger.info("Application startup complete")
    
    except Exception as e:
        logger.error(f"Error during startup: {e}", exc_info=True)
        raise
    
    yield
    
    # Shutdown
    logger.info("Shutting down application...")
    await close_db_pool()
    logger.info("Application shutdown complete")


# Create FastAPI app
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="RAG-powered chatbot backend for Physical AI & Humanoid Robotics textbook",
    lifespan=lifespan
)

# Include routers
app.include_router(auth_router)
app.include_router(personalize_router)
app.include_router(translate_router)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


async def create_tables():
    """Create database tables if they don't exist."""
    try:
        from db import get_db_connection
        
        async with get_db_connection() as conn:
            # Note: Full schema should be run from schema.sql
            # This is just for basic tables needed by the chat endpoint
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS conversations (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    conversation_id VARCHAR(255) UNIQUE NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
                
                CREATE TABLE IF NOT EXISTS messages (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    conversation_id VARCHAR(255) NOT NULL,
                    role VARCHAR(50) NOT NULL,
                    content TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (conversation_id) REFERENCES conversations(conversation_id)
                );
                
                CREATE INDEX IF NOT EXISTS idx_messages_conversation_id 
                ON messages(conversation_id);

                -- Users table for Auth
                CREATE TABLE IF NOT EXISTS users (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    email VARCHAR(255) UNIQUE NOT NULL,
                    password_hash VARCHAR(255) NOT NULL,
                    full_name VARCHAR(255),
                    avatar_url TEXT,
                    bio TEXT,
                    preferences JSONB DEFAULT '{}'::jsonb,
                    is_active BOOLEAN DEFAULT TRUE,
                    is_verified BOOLEAN DEFAULT FALSE,
                    last_login TIMESTAMP,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
                
                CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
            """)
        logger.info("Database tables created/verified")
        logger.info("Note: Run schema.sql for full database schema including users, chat_history, personalizations, and translations")
    except Exception as e:
        logger.error(f"Error creating tables: {e}", exc_info=True)
        raise


def get_embedding(text: str) -> List[float]:
    """
    Get embedding for text using HuggingFace Inference Client.
    """
    if hf_client is None:
         raise ValueError("HF Client not initialized (HF_TOKEN missing).")

    try:
        # Retry logic for API calls
        for attempt in range(3):
            try:
                # feature_extraction returns ndarray
                response = hf_client.feature_extraction(text, model=settings.hf_embedding_model)
                if response is not None:
                     return response.tolist()
            except Exception as e:
                # Check for loading error in string representation
                if "loading" in str(e).lower():
                    logger.info(f"Model loading... (Attempt {attempt+1})")
                    time.sleep(5)
                    continue
                else:
                    raise e
        
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to generate embedding (HF API Error)"
        )

    except Exception as e:
        logger.error(f"Error getting embedding: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate embedding: {str(e)}"
        )


async def retrieve_relevant_context(
    query: str, 
    top_k: int = None,
    difficulty: str = None,
    language: str = None
) -> List[dict]:
    """
    Retrieve relevant context from vector database using RAG.
    """
    if top_k is None:
        top_k = settings.top_k_results
    
    try:
        from qdrant_client.models import Filter, FieldCondition, MatchValue
        import asyncio
        
        # Get query embedding (Async call to sync function)
        query_embedding = await asyncio.to_thread(get_embedding, query)
        
        # Build filters
        must_filters = []
        if difficulty:
            must_filters.append(
                FieldCondition(key="metadata.difficulty", match=MatchValue(value=difficulty))
            )
        if language:
            must_filters.append(
                FieldCondition(key="metadata.language", match=MatchValue(value=language))
            )
            
        search_filter = Filter(must=must_filters) if must_filters else None
        
        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=top_k
        )
        
        # Format results
        contexts = []
        for result in search_results:
            contexts.append({
                "content": result.payload.get("content", ""),
                "metadata": result.payload.get("metadata", {}),
                "score": result.score
            })
        
        logger.info(f"Retrieved {len(contexts)} relevant contexts for query (Filters: diff={difficulty}, lang={language})")
        return contexts
    
    except Exception as e:
        logger.error(f"Error retrieving context: {e}", exc_info=True)
        return []


async def get_conversation_history(conversation_id: str, limit: int = 10) -> List[dict]:
    """
    Get conversation history from database.
    
    Args:
        conversation_id: Conversation ID
        limit: Maximum number of messages to retrieve
        
    Returns:
        List of messages
    """
    try:
        from db import get_db_connection
        
        async with get_db_connection() as conn:
            rows = await conn.fetch("""
                SELECT role, content, created_at
                FROM messages
                WHERE conversation_id = $1
                ORDER BY created_at DESC
                LIMIT $2
            """, conversation_id, limit)
            
            messages = [
                {
                    "role": row["role"],
                    "content": row["content"],
                    "timestamp": row["created_at"].isoformat()
                }
                for row in reversed(rows)  # Reverse to get chronological order
            ]
            
            return messages
    
    except Exception as e:
        logger.error(f"Error getting conversation history: {e}", exc_info=True)
        return []


async def save_message(conversation_id: str, role: str, content: str):
    """
    Save message to database.
    """
    try:
        from db import get_db_connection
        
        async with get_db_connection() as conn:
            # Ensure conversation exists
            await conn.execute("""
                INSERT INTO conversations (conversation_id)
                VALUES ($1)
                ON CONFLICT (conversation_id) DO NOTHING
            """, conversation_id)
            
            # Insert message
            await conn.execute("""
                INSERT INTO messages (conversation_id, role, content)
                VALUES ($1, $2, $3)
            """, conversation_id, role, content)
    
    except Exception as e:
        logger.error(f"Error saving message: {e}", exc_info=True)
        # Don't raise - message saving failure shouldn't break the request


@app.get("/", response_model=dict)
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI Textbook Chatbot API",
        "version": settings.app_version,
        "docs": "/docs"
    }


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.
    """
    services = {}
    
    # Check Qdrant
    try:
        qdrant_client.get_collection(settings.qdrant_collection_name)
        services["qdrant"] = "healthy"
    except Exception as e:
        services["qdrant"] = f"unhealthy: {str(e)}"
    
    # Check database
    try:
        from db import get_db_connection
        async with get_db_connection() as conn:
            await conn.fetchval("SELECT 1")
        services["database"] = "healthy"
    except Exception as e:
        services["database"] = f"unhealthy: {str(e)}"
    
    # Check AI Service
    if settings.gemini_api_key:
        services["ai_model"] = "gemini (configured)"
    elif settings.openai_api_key:
        services["ai_model"] = "openai (configured)"
    else:
        services["ai_model"] = "not configured"
    
    overall_status = "healthy" if all(
        "healthy" in status or "configured" in status
        for status in services.values()
    ) else "degraded"
    
    return HealthResponse(
        status=overall_status,
        version=settings.app_version,
        services=services
    )


@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint with RAG support.
    """
    try:
        # Generate or use conversation ID
        conversation_id = request.conversation_id or str(uuid.uuid4())
        
        # Get conversation history
        history = await get_conversation_history(conversation_id)
        
        # Build context
        context_parts = []
        sources = []
        
        if request.use_rag:
            # Retrieve relevant context with filters
            contexts = await retrieve_relevant_context(
                request.message, 
                difficulty=request.difficulty,
                language=request.language
            )
            
            for ctx in contexts:
                context_parts.append(ctx["content"])
                if "source" in ctx.get("metadata", {}):
                    sources.append(ctx["metadata"]["source"])
        
        # Add selected text if provided
        if request.selected_text:
            context_parts.append(f"Selected text from page: {request.selected_text}")
        
        # Build prompt for Gemini
        system_prompt = f"""You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
The user prefers '{request.language}' language and '{request.difficulty}' difficulty level.
Relevant context has been retrieved that matches these preferences.
Answer questions using ONLY the provided context.
If the context does not contain enough information, say so.
Be direct and helpful.

CONTEXT:
{' '.join(context_parts) if context_parts else 'No specific context provided.'}

CHAT HISTORY:
"""
        # Add history to prompt string (Gemini API handles history differently, but this is simple context injection)
        for msg in history[-5:]:
            system_prompt += f"{msg['role'].upper()}: {msg['content']}\n"
            
        system_prompt += f"USER: {request.message}\nASSISTANT:"
        
        # Save user message
        await save_message(conversation_id, "user", request.message)
        
        # Get response from AI
        assistant_response = ""
        tokens_used = 0
        
        try:
            if GEMINI_API_KEY:
                # Use Gemini - hardcode model like generate_all_versions.py
                import asyncio
                # Use exact model name from working script (NOT from env)
                model_name = "models/gemini-flash-latest"
                logger.info(f"Creating Gemini model with name: '{model_name}'")
                model = genai.GenerativeModel(model_name)
                response = await asyncio.to_thread(
                    model.generate_content,
                    system_prompt
                )
                assistant_response = response.text.strip()
            elif settings.openai_api_key:
                # Use OpenAI
                messages = [{"role": "system", "content": system_prompt}]
                messages.append({"role": "user", "content": request.message})
                
                response = openai.chat.completions.create(
                    model=settings.openai_model,
                    messages=messages,
                    max_tokens=500,
                    temperature=0.7
                )
                assistant_response = response.choices[0].message.content
                tokens_used = response.usage.total_tokens if response.usage else 0
            else:
                 raise ValueError("No AI service configured")
                 
        except Exception as e:
            logger.error(f"AI API error: {e}", exc_info=True)
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to generate response: {str(e)}"
            )
        
        # Save assistant message
        await save_message(conversation_id, "assistant", assistant_response)
        
        return ChatResponse(
            response=assistant_response,
            conversation_id=conversation_id,
            sources=list(set(sources)),  # Remove duplicates
            tokens_used=tokens_used
        )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}"
        )


@app.post("/ingest", response_model=IngestResponse)
async def ingest_content(request: IngestRequest):
    """
    Ingest content into vector database.
    
    Args:
        request: Content to ingest with metadata
        
    Returns:
        Ingestion result
    """
    try:
        from ingest import chunk_text, ingest_chunks
        
        # Chunk the content
        chunks = chunk_text(request.content)
        
        # Generate document ID
        document_id = request.document_id or str(uuid.uuid4())
        
        # Ingest chunks (now synchronous)
        chunks_created = ingest_chunks(
            chunks=chunks,
            metadata=request.metadata,
            document_id=document_id
        )
        
        return IngestResponse(
            success=True,
            document_id=document_id,
            chunks_created=chunks_created,
            message=f"Successfully ingested {chunks_created} chunks"
        )
    
    except Exception as e:
        logger.error(f"Error ingesting content: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to ingest content: {str(e)}"
        )


@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content=ErrorResponse(
            error="Internal server error",
            detail=str(exc) if settings.debug else None
        ).dict()
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug
    )
