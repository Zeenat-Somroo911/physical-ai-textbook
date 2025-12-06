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

# Initialize OpenAI
openai.api_key = settings.openai_api_key

# Global clients (initialized in lifespan)
qdrant_client: Optional[QdrantClient] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager."""
    global qdrant_client
    
    # Startup
    logger.info("Starting up application...")
    
    try:
        # Initialize Qdrant client
        if settings.mock_mode:
            logger.warning("Running in MOCK MODE - Qdrant disabled")
            qdrant_client = None
        else:
            try:
                if settings.qdrant_api_key:
                    qdrant_client = QdrantClient(
                        url=settings.qdrant_url,
                        api_key=settings.qdrant_api_key
                    )
                else:
                    qdrant_client = QdrantClient(url=settings.qdrant_url)
                
                # Ensure collection exists
                try:
                    qdrant_client.get_collection(settings.qdrant_collection_name)
                    logger.info(f"Qdrant collection '{settings.qdrant_collection_name}' exists")
                except Exception:
                    # Create collection if it doesn't exist
                    qdrant_client.create_collection(
                        collection_name=settings.qdrant_collection_name,
                        vectors_config=VectorParams(
                            size=1536,  # OpenAI text-embedding-3-small dimension
                            distance=Distance.COSINE
                        )
                    )
                    logger.info(f"Created Qdrant collection '{settings.qdrant_collection_name}'")
            except Exception as e:
                logger.error(f"Failed to connect to Qdrant: {e}")
                logger.warning("Disabling Qdrant functionality (Mock Mode)")
                qdrant_client = None

        
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
    Get embedding for text using OpenAI.
    
    Args:
        text: Text to embed
        
    Returns:
        List of floats representing the embedding
    """
    try:
        response = openai.embeddings.create(
            model=settings.openai_embedding_model,
            input=text
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate embedding: {str(e)}"
        )


async def retrieve_relevant_context(query: str, top_k: int = None) -> List[dict]:
    """
    Retrieve relevant context from vector database using RAG.
    
    Args:
        query: User query
        top_k: Number of results to retrieve
        
    Returns:
        List of relevant documents with metadata
    """
    if top_k is None:
        top_k = settings.top_k_results
    
    try:
        # Get query embedding
        query_embedding = get_embedding(query)
        
        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
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
        
        logger.info(f"Retrieved {len(contexts)} relevant contexts for query")
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
    
    Args:
        conversation_id: Conversation ID
        role: Message role ('user' or 'assistant')
        content: Message content
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
    
    Returns:
        Health status of the application and services
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
    
    # Check OpenAI (simple check)
    services["openai"] = "configured" if settings.openai_api_key else "not configured"
    
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
    
    Args:
        request: Chat request with message and optional context
        
    Returns:
        Chat response with assistant reply and sources
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
            # Retrieve relevant context
            contexts = await retrieve_relevant_context(request.message)
            
            for ctx in contexts:
                context_parts.append(ctx["content"])
                if "source" in ctx.get("metadata", {}):
                    sources.append(ctx["metadata"]["source"])
        
        # Add selected text if provided
        if request.selected_text:
            context_parts.append(f"Selected text from page: {request.selected_text}")
        
        # Build system prompt
        system_prompt = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context from the textbook.
If the context does not contain enough information, say so.
Be concise, accurate, and helpful."""
        
        # Build messages for OpenAI
        messages = [{"role": "system", "content": system_prompt}]
        
        # Add context if available
        if context_parts:
            context_text = "\n\n".join(context_parts)
            messages.append({
                "role": "system",
                "content": f"Relevant context from textbook:\n\n{context_text}"
            })
        
        # Add conversation history
        for msg in history[-5:]:  # Last 5 messages for context
            messages.append({
                "role": msg["role"],
                "content": msg["content"]
            })
        
        # Add current user message
        messages.append({"role": "user", "content": request.message})
        
        # Save user message
        await save_message(conversation_id, "user", request.message)
        
        # Get response from OpenAI
        try:
            response = openai.chat.completions.create(
                model=settings.openai_model,
                messages=messages,
                max_tokens=500,
                temperature=0.7
            )
            
            assistant_response = response.choices[0].message.content
            tokens_used = response.usage.total_tokens if response.usage else None
            
        except Exception as e:
            logger.error(f"OpenAI API error: {e}", exc_info=True)
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
