-- Database Schema for Physical AI Textbook Chatbot
-- Neon Postgres Database Schema
-- Run this script to create all necessary tables

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- ============================================================================
-- USERS TABLE
-- ============================================================================
-- Stores user account information and profiles
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
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

-- Index for email lookups (already unique, but explicit index for performance)
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Index for active users
CREATE INDEX IF NOT EXISTS idx_users_is_active ON users(is_active);

-- Index for created_at (for sorting/filtering)
CREATE INDEX IF NOT EXISTS idx_users_created_at ON users(created_at);

-- ============================================================================
-- CHAT_HISTORY TABLE
-- ============================================================================
-- Stores chat conversation history between users and the chatbot
CREATE TABLE IF NOT EXISTS chat_history (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL,
    conversation_id VARCHAR(255) NOT NULL,
    question TEXT NOT NULL,
    answer TEXT NOT NULL,
    sources TEXT[] DEFAULT ARRAY[]::TEXT[],
    tokens_used INTEGER,
    model_used VARCHAR(100),
    selected_text TEXT,
    use_rag BOOLEAN DEFAULT TRUE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    
    -- Foreign key constraint
    CONSTRAINT fk_chat_history_user 
        FOREIGN KEY (user_id) 
        REFERENCES users(id) 
        ON DELETE CASCADE
);

-- Index for user_id (most common query pattern)
CREATE INDEX IF NOT EXISTS idx_chat_history_user_id ON chat_history(user_id);

-- Index for conversation_id (for retrieving conversation threads)
CREATE INDEX IF NOT EXISTS idx_chat_history_conversation_id ON chat_history(conversation_id);

-- Index for created_at (for sorting by time)
CREATE INDEX IF NOT EXISTS idx_chat_history_created_at ON chat_history(created_at);

-- Composite index for user conversations
CREATE INDEX IF NOT EXISTS idx_chat_history_user_conversation 
    ON chat_history(user_id, conversation_id, created_at);

-- ============================================================================
-- PERSONALIZATIONS TABLE
-- ============================================================================
-- Stores user-specific personalizations and notes for chapters
CREATE TABLE IF NOT EXISTS personalizations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL,
    chapter_id VARCHAR(255) NOT NULL,
    chapter_path VARCHAR(500),
    content TEXT NOT NULL,
    content_type VARCHAR(50) DEFAULT 'note', -- 'note', 'bookmark', 'highlight', 'summary'
    metadata JSONB DEFAULT '{}'::jsonb,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    
    -- Foreign key constraint
    CONSTRAINT fk_personalizations_user 
        FOREIGN KEY (user_id) 
        REFERENCES users(id) 
        ON DELETE CASCADE,
    
    -- Ensure unique combination of user and chapter for same content type
    CONSTRAINT unique_user_chapter_type 
        UNIQUE (user_id, chapter_id, content_type)
);

-- Index for user_id
CREATE INDEX IF NOT EXISTS idx_personalizations_user_id ON personalizations(user_id);

-- Index for chapter_id (for chapter-specific queries)
CREATE INDEX IF NOT EXISTS idx_personalizations_chapter_id ON personalizations(chapter_id);

-- Index for content_type (for filtering by type)
CREATE INDEX IF NOT EXISTS idx_personalizations_content_type ON personalizations(content_type);

-- Composite index for user chapter lookups
CREATE INDEX IF NOT EXISTS idx_personalizations_user_chapter 
    ON personalizations(user_id, chapter_id);

-- Index for updated_at (for sorting)
CREATE INDEX IF NOT EXISTS idx_personalizations_updated_at ON personalizations(updated_at);

-- ============================================================================
-- TRANSLATIONS TABLE
-- ============================================================================
-- Caches translations of chapters (e.g., Urdu translations)
CREATE TABLE IF NOT EXISTS translations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    chapter_id VARCHAR(255) NOT NULL,
    chapter_path VARCHAR(500),
    language_code VARCHAR(10) NOT NULL DEFAULT 'ur', -- ISO 639-1 language code
    translated_content TEXT NOT NULL,
    original_content_hash VARCHAR(64), -- SHA-256 hash of original content
    translation_model VARCHAR(100),
    word_count INTEGER,
    cached_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP,
    
    -- Ensure unique combination of chapter and language
    CONSTRAINT unique_chapter_language 
        UNIQUE (chapter_id, language_code)
);

-- Index for chapter_id
CREATE INDEX IF NOT EXISTS idx_translations_chapter_id ON translations(chapter_id);

-- Index for language_code (for filtering by language)
CREATE INDEX IF NOT EXISTS idx_translations_language_code ON translations(language_code);

-- Index for cached_at (for cache management)
CREATE INDEX IF NOT EXISTS idx_translations_cached_at ON translations(cached_at);

-- Index for expires_at (for cache cleanup)
CREATE INDEX IF NOT EXISTS idx_translations_expires_at ON translations(expires_at);

-- Composite index for chapter language lookups
CREATE INDEX IF NOT EXISTS idx_translations_chapter_language 
    ON translations(chapter_id, language_code);

-- ============================================================================
-- CONVERSATIONS TABLE (Enhanced)
-- ============================================================================
-- Stores conversation metadata (already exists in main.py, but enhanced here)
CREATE TABLE IF NOT EXISTS conversations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    conversation_id VARCHAR(255) UNIQUE NOT NULL,
    user_id UUID,
    title VARCHAR(500),
    metadata JSONB DEFAULT '{}'::jsonb,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    
    -- Foreign key constraint (optional, for user-specific conversations)
    CONSTRAINT fk_conversations_user 
        FOREIGN KEY (user_id) 
        REFERENCES users(id) 
        ON DELETE SET NULL
);

-- Index for user_id
CREATE INDEX IF NOT EXISTS idx_conversations_user_id ON conversations(user_id);

-- Index for conversation_id (already unique, but explicit for performance)
CREATE INDEX IF NOT EXISTS idx_conversations_conversation_id ON conversations(conversation_id);

-- Index for updated_at (for sorting)
CREATE INDEX IF NOT EXISTS idx_conversations_updated_at ON conversations(updated_at);

-- ============================================================================
-- TRIGGERS
-- ============================================================================

-- Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Trigger for users table
DROP TRIGGER IF EXISTS update_users_updated_at ON users;
CREATE TRIGGER update_users_updated_at
    BEFORE UPDATE ON users
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for personalizations table
DROP TRIGGER IF EXISTS update_personalizations_updated_at ON personalizations;
CREATE TRIGGER update_personalizations_updated_at
    BEFORE UPDATE ON personalizations
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for conversations table
DROP TRIGGER IF EXISTS update_conversations_updated_at ON conversations;
CREATE TRIGGER update_conversations_updated_at
    BEFORE UPDATE ON conversations
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- ============================================================================
-- COMMENTS
-- ============================================================================

COMMENT ON TABLE users IS 'Stores user account information and profiles';
COMMENT ON TABLE chat_history IS 'Stores chat conversation history between users and the chatbot';
COMMENT ON TABLE personalizations IS 'Stores user-specific personalizations, notes, bookmarks, and highlights for chapters';
COMMENT ON TABLE translations IS 'Caches translations of chapters in different languages (e.g., Urdu)';
COMMENT ON TABLE conversations IS 'Stores conversation metadata and user-specific conversation information';

COMMENT ON COLUMN users.preferences IS 'JSON object storing user preferences (theme, language, etc.)';
COMMENT ON COLUMN chat_history.sources IS 'Array of source document paths used in RAG response';
COMMENT ON COLUMN chat_history.selected_text IS 'Selected text from page that was used as context';
COMMENT ON COLUMN personalizations.content_type IS 'Type of personalization: note, bookmark, highlight, or summary';
COMMENT ON COLUMN personalizations.metadata IS 'Additional metadata for the personalization (e.g., highlight color, position)';
COMMENT ON COLUMN translations.original_content_hash IS 'SHA-256 hash of original content for cache invalidation';
COMMENT ON COLUMN translations.expires_at IS 'Timestamp when cached translation expires and should be refreshed';

