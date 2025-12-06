"""
Database connection and query utilities.

Provides database operations for users, chat history, personalizations, and translations.
"""

import logging
import json
from typing import List, Optional, Dict, Any
from datetime import datetime
import asyncpg
from contextlib import asynccontextmanager

from config import settings

logger = logging.getLogger(__name__)

# Global connection pool
_db_pool: Optional[asyncpg.Pool] = None


async def init_db_pool():
    """
    Initialize database connection pool.
    
    Should be called during application startup.
    """
    global _db_pool
    
    if settings.mock_mode:
        logger.warning("Mock Mode enabled - Skipping DB connection")
        return None
    
    if _db_pool is None:
        try:
            _db_pool = await asyncpg.create_pool(
                settings.database_url,
                min_size=5,
                max_size=20,
                command_timeout=60
            )
            logger.info("Database connection pool initialized")
        except Exception as e:
            logger.error(f"Failed to initialize database pool: {e}", exc_info=True)
            raise
    
    return _db_pool


async def close_db_pool():
    """Close database connection pool."""
    global _db_pool
    
    if _db_pool:
        await _db_pool.close()
        _db_pool = None
        logger.info("Database connection pool closed")


class MockConnection:
    """Mock database connection for testing/demo purposes."""
    
    def __init__(self):
        self.transaction = self.MockTransaction()

    class MockTransaction:
        async def __aenter__(self): return self
        async def __aexit__(self, *args): pass

    async def fetch(self, query, *args):
        logger.info(f"MOCK DB: fetch called with query: {query[:50]}...")
        return []

    async def fetchrow(self, query, *args):
        logger.info(f"MOCK DB: fetchrow called with query: {query[:50]}...")
        # Return dummy user for login
        if "SELECT id, email" in query and "users" in query:
             return {
                 "id": "00000000-0000-0000-0000-000000000000",
                 "email": args[0] if args else "mock@example.com",
                 "password_hash": "$2b$12$K.z...ignored...", # Dummy hash
                 "full_name": "Mock User",
                 "avatar_url": None,
                 "bio": "Running in Mock Mode",
                 "preferences": {},
                 "is_active": True,
                 "is_verified": True,
                 "last_login": datetime.utcnow(),
                 "created_at": datetime.utcnow()
             }
        return None

    async def fetchval(self, query, *args):
        logger.info(f"MOCK DB: fetchval called")
        return 1

    async def execute(self, query, *args):
        logger.info(f"MOCK DB: execute called")
        return "INSERT 1"
        
    def transaction(self):
        return self.transaction

    async def close(self):
        pass


@asynccontextmanager
async def get_db_connection():
    """
    Get database connection from pool.
    """
    if settings.mock_mode:
        yield MockConnection()
        return

    if _db_pool is None:
        await init_db_pool()
    
    async with _db_pool.acquire() as connection:
        yield connection


# ============================================================================
# USER OPERATIONS
# ============================================================================

async def create_user(
    email: str,
    password_hash: str,
    full_name: Optional[str] = None,
    **kwargs
) -> Dict[str, Any]:
    """
    Create a new user.
    
    Args:
        email: User email address
        password_hash: Hashed password
        full_name: User's full name
        **kwargs: Additional user fields
        
    Returns:
        Dictionary with user data
    """
    async with get_db_connection() as conn:
        # Serialize preferences to JSON string for asyncpg
        preferences_json = json.dumps(kwargs.get('preferences', {}))
        
        row = await conn.fetchrow("""
            INSERT INTO users (email, password_hash, full_name, preferences)
            VALUES ($1, $2, $3, $4::jsonb)
            RETURNING id, email, full_name, created_at
        """, email, password_hash, full_name, preferences_json)
        
        return dict(row)


async def get_user_by_email(email: str) -> Optional[Dict[str, Any]]:
    """
    Get user by email address.
    
    Args:
        email: User email address
        
    Returns:
        User data dictionary or None
    """
    async with get_db_connection() as conn:
        row = await conn.fetchrow("""
            SELECT id, email, password_hash, full_name, avatar_url, bio,
                   preferences, is_active, is_verified, last_login, created_at
            FROM users
            WHERE email = $1
        """, email)
        
        if row:
            data = dict(row)
            # Ensure preferences is a dict if returned as string (though asyncpg usually decodes jsonb)
            if isinstance(data.get('preferences'), str):
                try:
                    data['preferences'] = json.loads(data['preferences'])
                except:
                    data['preferences'] = {}
            return data
        return None


async def get_user_by_id(user_id: str) -> Optional[Dict[str, Any]]:
    """
    Get user by ID.
    
    Args:
        user_id: User UUID
        
    Returns:
        User data dictionary or None
    """
    async with get_db_connection() as conn:
        row = await conn.fetchrow("""
            SELECT id, email, full_name, avatar_url, bio, preferences,
                   is_active, is_verified, last_login, created_at
            FROM users
            WHERE id = $1
        """, user_id)
        
        if row:
            data = dict(row)
            # Ensure preferences is a dict
            if isinstance(data.get('preferences'), str):
                try:
                    data['preferences'] = json.loads(data['preferences'])
                except:
                    data['preferences'] = {}
            return data
        return None


async def update_user(user_id: str, **kwargs) -> bool:
    """
    Update user information.
    
    Args:
        user_id: User UUID
        **kwargs: Fields to update
        
    Returns:
        True if update successful
    """
    if not kwargs:
        return False
    
    # Build dynamic update query
    set_clauses = []
    values = []
    param_num = 1
    
    for key, value in kwargs.items():
        if key == 'preferences' and isinstance(value, dict):
            set_clauses.append(f"{key} = ${param_num}::jsonb")
            values.append(json.dumps(value))
        else:
            set_clauses.append(f"{key} = ${param_num}")
            values.append(value)
        param_num += 1
    
    values.append(user_id)
    
    query = f"""
        UPDATE users
        SET {', '.join(set_clauses)}
        WHERE id = ${param_num}
    """
    
    async with get_db_connection() as conn:
        result = await conn.execute(query, *values)
        return result == "UPDATE 1"


async def update_last_login(user_id: str):
    """Update user's last login timestamp."""
    async with get_db_connection() as conn:
        await conn.execute("""
            UPDATE users
            SET last_login = CURRENT_TIMESTAMP
            WHERE id = $1
        """, user_id)


# ============================================================================
# CHAT HISTORY OPERATIONS
# ============================================================================

async def save_chat_message(
    user_id: str,
    conversation_id: str,
    question: str,
    answer: str,
    sources: List[str] = None,
    tokens_used: Optional[int] = None,
    model_used: Optional[str] = None,
    selected_text: Optional[str] = None,
    use_rag: bool = True
) -> str:
    """
    Save a chat message to history.
    
    Args:
        user_id: User UUID
        conversation_id: Conversation ID
        question: User's question
        answer: Assistant's answer
        sources: List of source documents
        tokens_used: Number of tokens used
        model_used: Model used for response
        selected_text: Selected text from page
        use_rag: Whether RAG was used
        
    Returns:
        Message ID
    """
    async with get_db_connection() as conn:
        row = await conn.fetchrow("""
            INSERT INTO chat_history (
                user_id, conversation_id, question, answer, sources,
                tokens_used, model_used, selected_text, use_rag
            )
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
            RETURNING id
        """, user_id, conversation_id, question, answer,
            sources or [], tokens_used, model_used, selected_text, use_rag)
        
        return str(row['id'])


async def get_chat_history(
    user_id: str,
    conversation_id: Optional[str] = None,
    limit: int = 50,
    offset: int = 0
) -> List[Dict[str, Any]]:
    """
    Get chat history for a user.
    
    Args:
        user_id: User UUID
        conversation_id: Optional conversation ID filter
        limit: Maximum number of messages
        offset: Offset for pagination
        
    Returns:
        List of chat messages
    """
    async with get_db_connection() as conn:
        if conversation_id:
            rows = await conn.fetch("""
                SELECT id, conversation_id, question, answer, sources,
                       tokens_used, model_used, selected_text, use_rag, created_at
                FROM chat_history
                WHERE user_id = $1 AND conversation_id = $2
                ORDER BY created_at DESC
                LIMIT $3 OFFSET $4
            """, user_id, conversation_id, limit, offset)
        else:
            rows = await conn.fetch("""
                SELECT id, conversation_id, question, answer, sources,
                       tokens_used, model_used, selected_text, use_rag, created_at
                FROM chat_history
                WHERE user_id = $1
                ORDER BY created_at DESC
                LIMIT $2 OFFSET $3
            """, user_id, limit, offset)
        
        return [dict(row) for row in rows]


async def get_user_conversations(user_id: str) -> List[Dict[str, Any]]:
    """
    Get all conversation IDs for a user.
    
    Args:
        user_id: User UUID
        
    Returns:
        List of conversation metadata
    """
    async with get_db_connection() as conn:
        rows = await conn.fetch("""
            SELECT DISTINCT conversation_id,
                   MIN(created_at) as first_message,
                   MAX(created_at) as last_message,
                   COUNT(*) as message_count
            FROM chat_history
            WHERE user_id = $1
            GROUP BY conversation_id
            ORDER BY last_message DESC
        """, user_id)
        
        return [dict(row) for row in rows]


# ============================================================================
# PERSONALIZATIONS OPERATIONS
# ============================================================================

async def save_personalization(
    user_id: str,
    chapter_id: str,
    content: str,
    content_type: str = 'note',
    chapter_path: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None
) -> str:
    """
    Save or update a personalization.
    
    Args:
        user_id: User UUID
        chapter_id: Chapter identifier
        content: Personalization content
        content_type: Type (note, bookmark, highlight, summary)
        chapter_path: Full path to chapter
        metadata: Additional metadata
        
    Returns:
        Personalization ID
    """
    async with get_db_connection() as conn:
        row = await conn.fetchrow("""
            INSERT INTO personalizations (
                user_id, chapter_id, chapter_path, content, content_type, metadata
            )
            VALUES ($1, $2, $3, $4, $5, $6::jsonb)
            ON CONFLICT (user_id, chapter_id, content_type)
            DO UPDATE SET
                content = EXCLUDED.content,
                metadata = EXCLUDED.metadata,
                updated_at = CURRENT_TIMESTAMP
            RETURNING id
        """, user_id, chapter_id, chapter_path, content, content_type,
            metadata or {})
        
        return str(row['id'])


async def get_personalizations(
    user_id: str,
    chapter_id: Optional[str] = None,
    content_type: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    Get personalizations for a user.
    
    Args:
        user_id: User UUID
        chapter_id: Optional chapter filter
        content_type: Optional content type filter
        
    Returns:
        List of personalizations
    """
    async with get_db_connection() as conn:
        if chapter_id and content_type:
            rows = await conn.fetch("""
                SELECT id, chapter_id, chapter_path, content, content_type,
                       metadata, created_at, updated_at
                FROM personalizations
                WHERE user_id = $1 AND chapter_id = $2 AND content_type = $3
                ORDER BY updated_at DESC
            """, user_id, chapter_id, content_type)
        elif chapter_id:
            rows = await conn.fetch("""
                SELECT id, chapter_id, chapter_path, content, content_type,
                       metadata, created_at, updated_at
                FROM personalizations
                WHERE user_id = $1 AND chapter_id = $2
                ORDER BY updated_at DESC
            """, user_id, chapter_id)
        else:
            rows = await conn.fetch("""
                SELECT id, chapter_id, chapter_path, content, content_type,
                       metadata, created_at, updated_at
                FROM personalizations
                WHERE user_id = $1
                ORDER BY updated_at DESC
            """, user_id)
        
        return [dict(row) for row in rows]


async def delete_personalization(personalization_id: str) -> bool:
    """
    Delete a personalization.
    
    Args:
        personalization_id: Personalization UUID
        
    Returns:
        True if deletion successful
    """
    async with get_db_connection() as conn:
        result = await conn.execute("""
            DELETE FROM personalizations
            WHERE id = $1
        """, personalization_id)
        
        return result == "DELETE 1"


# ============================================================================
# TRANSLATIONS OPERATIONS
# ============================================================================

async def save_translation(
    chapter_id: str,
    translated_content: str,
    language_code: str = 'ur',
    chapter_path: Optional[str] = None,
    original_content_hash: Optional[str] = None,
    translation_model: Optional[str] = None,
    word_count: Optional[int] = None,
    expires_at: Optional[datetime] = None
) -> str:
    """
    Save or update a translation.
    
    Args:
        chapter_id: Chapter identifier
        translated_content: Translated content
        language_code: ISO 639-1 language code
        chapter_path: Full path to chapter
        original_content_hash: Hash of original content
        translation_model: Model used for translation
        word_count: Word count of translation
        expires_at: Expiration timestamp
        
    Returns:
        Translation ID
    """
    async with get_db_connection() as conn:
        row = await conn.fetchrow("""
            INSERT INTO translations (
                chapter_id, chapter_path, language_code, translated_content,
                original_content_hash, translation_model, word_count, expires_at
            )
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            ON CONFLICT (chapter_id, language_code)
            DO UPDATE SET
                translated_content = EXCLUDED.translated_content,
                original_content_hash = EXCLUDED.original_content_hash,
                cached_at = CURRENT_TIMESTAMP,
                expires_at = EXCLUDED.expires_at
            RETURNING id
        """, chapter_id, chapter_path, language_code, translated_content,
            original_content_hash, translation_model, word_count, expires_at)
        
        return str(row['id'])


async def get_translation(
    chapter_id: str,
    language_code: str = 'ur'
) -> Optional[Dict[str, Any]]:
    """
    Get translation for a chapter.
    
    Args:
        chapter_id: Chapter identifier
        language_code: ISO 639-1 language code
        
    Returns:
        Translation data or None
    """
    async with get_db_connection() as conn:
        row = await conn.fetchrow("""
            SELECT id, chapter_id, chapter_path, language_code, translated_content,
                   original_content_hash, translation_model, word_count,
                   cached_at, expires_at
            FROM translations
            WHERE chapter_id = $1 AND language_code = $2
            AND (expires_at IS NULL OR expires_at > CURRENT_TIMESTAMP)
        """, chapter_id, language_code)
        
        return dict(row) if row else None


async def delete_expired_translations() -> int:
    """
    Delete expired translations.
    
    Returns:
        Number of translations deleted
    """
    async with get_db_connection() as conn:
        result = await conn.execute("""
            DELETE FROM translations
            WHERE expires_at IS NOT NULL AND expires_at < CURRENT_TIMESTAMP
        """)
        
        # Extract number from result string like "DELETE 5"
        count = int(result.split()[-1]) if result.startswith("DELETE") else 0
        return count

