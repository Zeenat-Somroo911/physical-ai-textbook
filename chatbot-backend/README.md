# Chatbot Backend

FastAPI backend for the RAG-powered chatbot assistant for the Physical AI & Humanoid Robotics textbook.

## Features

- **RAG (Retrieval-Augmented Generation)**: Enhanced responses using textbook content
- **Vector Search**: Qdrant integration for semantic search
- **Conversation History**: Persistent conversation storage in Neon Postgres
- **Selected Text Support**: Context-aware responses using selected page text
- **OpenAI Integration**: GPT-3.5-turbo for chat and text-embedding-3-small for embeddings
- **CORS Enabled**: Ready for frontend integration
- **Production Ready**: Error handling, logging, and validation

## Prerequisites

- Python 3.10+
- OpenAI API key
- Qdrant instance (local or cloud)
- Neon Postgres database (or any PostgreSQL database)

## Setup

### 1. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Configure Environment Variables

Copy `.env.example` to `.env` and fill in your values:

```bash
cp .env.example .env
```

Edit `.env` with your configuration:

```env
OPENAI_API_KEY=your-openai-api-key-here
QDRANT_URL=http://localhost:6333
DATABASE_URL=postgresql://user:password@host:5432/database
```

### 4. Set Up Qdrant

#### Option A: Local Qdrant (Docker)

```bash
docker run -p 6333:6333 qdrant/qdrant
```

#### Option B: Qdrant Cloud

1. Sign up at [qdrant.cloud](https://cloud.qdrant.io)
2. Create a cluster
3. Get your API key and URL
4. Update `.env` with your Qdrant Cloud credentials

### 5. Set Up Database

#### Option A: Neon Postgres (Recommended)

1. Sign up at [neon.tech](https://neon.tech)
2. Create a new project
3. Copy the connection string
4. Update `DATABASE_URL` in `.env`
5. Run the schema script:
   ```bash
   psql $DATABASE_URL -f schema.sql
   ```

#### Option B: Local PostgreSQL

```bash
# Install PostgreSQL (if not installed)
# Then create database
createdb chatbot_db
```

Update `DATABASE_URL` in `.env`:
```
DATABASE_URL=postgresql://user:password@localhost:5432/chatbot_db
```

Run the schema script:
```bash
psql -d chatbot_db -f schema.sql
```

### 6. Ingest Content

Ingest the textbook content into the vector database:

```bash
python ingest.py
```

This will process all markdown files in the `../docs` directory and create embeddings.

### 7. Run the Server

```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`

## API Endpoints

### `GET /`

Root endpoint with API information.

**Response:**
```json
{
  "message": "Physical AI Textbook Chatbot API",
  "version": "1.0.0",
  "docs": "/docs"
}
```

### `GET /health`

Health check endpoint.

**Response:**
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2024-01-01T12:00:00Z",
  "services": {
    "qdrant": "healthy",
    "database": "healthy",
    "openai": "configured"
  }
}
```

### `POST /chat`

Chat endpoint with RAG support.

**Request Body:**
```json
{
  "message": "What is ROS 2?",
  "conversation_id": "optional-conversation-id",
  "selected_text": "Optional selected text from page",
  "use_rag": true
}
```

**Response:**
```json
{
  "response": "ROS 2 is the Robot Operating System 2...",
  "conversation_id": "conv_123",
  "sources": ["/docs/module-01-ros2/introduction"],
  "tokens_used": 150,
  "timestamp": "2024-01-01T12:00:00Z"
}
```

### `POST /ingest`

Ingest content into the vector database.

**Request Body:**
```json
{
  "content": "Text content to ingest...",
  "metadata": {
    "source": "/docs/module-01-ros2/introduction",
    "title": "ROS 2 Introduction"
  },
  "document_id": "optional-document-id"
}
```

**Response:**
```json
{
  "success": true,
  "document_id": "doc_123",
  "chunks_created": 5,
  "message": "Successfully ingested 5 chunks"
}
```

## API Documentation

Interactive API documentation is available at:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Development

### Running in Development Mode

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Running Tests

```bash
pytest
```

### Code Structure

```
chatbot-backend/
├── main.py           # FastAPI application and endpoints
├── config.py         # Configuration management
├── models.py         # Pydantic models
├── ingest.py         # Content ingestion script
├── requirements.txt  # Python dependencies
├── .env.example     # Environment variables template
├── .gitignore       # Git ignore rules
└── README.md        # This file
```

## Environment Variables

| Variable | Description | Required | Default |
|----------|-------------|----------|---------|
| `OPENAI_API_KEY` | OpenAI API key | Yes | - |
| `OPENAI_MODEL` | OpenAI chat model | No | `gpt-3.5-turbo` |
| `OPENAI_EMBEDDING_MODEL` | OpenAI embedding model | No | `text-embedding-3-small` |
| `QDRANT_URL` | Qdrant server URL | No | `http://localhost:6333` |
| `QDRANT_API_KEY` | Qdrant API key | No | - |
| `QDRANT_COLLECTION_NAME` | Qdrant collection name | No | `textbook_embeddings` |
| `DATABASE_URL` | PostgreSQL connection string | Yes | - |
| `DEBUG` | Debug mode | No | `False` |
| `LOG_LEVEL` | Logging level | No | `INFO` |
| `CORS_ORIGINS` | Allowed CORS origins | No | `http://localhost:3000,http://localhost:5173` |
| `MAX_CONTEXT_LENGTH` | Max context length for RAG | No | `4000` |
| `TOP_K_RESULTS` | Number of results to retrieve | No | `5` |

## Security Best Practices

1. **Never commit `.env` file**: It contains sensitive API keys
2. **Use environment variables**: Store secrets in environment variables
3. **Enable CORS properly**: Only allow trusted origins in production
4. **Rate limiting**: Consider adding rate limiting for production
5. **Input validation**: All inputs are validated using Pydantic models
6. **Error handling**: Errors don't expose sensitive information

## Troubleshooting

### Qdrant Connection Error

**Problem**: Cannot connect to Qdrant

**Solution**:
- Check if Qdrant is running: `curl http://localhost:6333/health`
- Verify `QDRANT_URL` in `.env`
- For Qdrant Cloud, check API key

### Database Connection Error

**Problem**: Cannot connect to database

**Solution**:
- Verify `DATABASE_URL` format: `postgresql://user:password@host:port/database`
- Check database is running and accessible
- Verify credentials

### OpenAI API Error

**Problem**: OpenAI API calls failing

**Solution**:
- Verify `OPENAI_API_KEY` is set correctly
- Check API key has sufficient credits
- Verify network connectivity

### Embedding Generation Slow

**Problem**: Embeddings taking too long

**Solution**:
- Use smaller embedding model
- Reduce chunk size
- Process in batches

## Production Deployment

### Using Docker

Create `Dockerfile`:

```dockerfile
FROM python:3.10-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Build and run:

```bash
docker build -t chatbot-backend .
docker run -p 8000:8000 --env-file .env chatbot-backend
```

### Using Gunicorn

```bash
pip install gunicorn
gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

## License

This project is part of the Physical AI & Humanoid Robotics textbook.

## Support

For issues and questions:
- Check the [FAQ](../../docs/faq.md)
- Review the [documentation](../../docs/intro.md)
- [Open an issue on GitHub](https://github.com/Zeenat-Somroo911/physical-ai-textbook/issues)
