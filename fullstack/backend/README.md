# AI-Spec Driven Book - Backend

FastAPI-based RAG (Retrieval-Augmented Generation) chatbot service providing intelligent, context-aware assistance for the AI-Spec Driven Book learning platform.

## Overview

The backend is a stateless REST API that powers the embedded chatbot on the frontend. It uses semantic search over course content to provide accurate, citation-backed answers to student questions about Physical AI and Humanoid Robotics.

### Architecture

```
Backend Service
├── FastAPI REST API (async)
├── Groq API (LLM inference)
├── Cohere (text embeddings)
├── Qdrant Cloud (vector database)
└── Conversation context management
```

## Features

- **Semantic Search**: Qdrant vector database with Cohere embeddings (1024 dimensions)
- **Multiple LLM Support**: Groq API with Llama, Mixtral, and Gemma models
- **Conversation Management**: Client-side history with server-side context optimization
- **Token Budget Management**: Conservative 7900-token limit with 292-token safety margin
- **Citation System**: URL transformation from absolute to relative Docusaurus routes
- **Retry Logic**: Exponential backoff with tenacity for API resilience
- **CORS Configuration**: Environment-based allowed origins for security
- **Health Checks**: `/health` endpoint for monitoring

## Tech Stack

- **Framework**: FastAPI (async Python web framework)
- **Server**: Uvicorn (ASGI server)
- **LLM**: Groq API (Llama 3.3 70B, Mixtral 8x7B, Gemma 2 9B)
- **Embeddings**: Cohere embed-english-v3.0 (1024 dimensions, cosine similarity)
- **Vector Database**: Qdrant Cloud
- **Tokenization**: tiktoken (cl100k_base encoding)
- **Configuration**: python-dotenv
- **Retry Logic**: tenacity
- **Data Validation**: pydantic

## Getting Started

### Prerequisites

- Python 3.9 or higher
- pip package manager
- API keys for Groq, Cohere, and Qdrant Cloud

### Installation

1. Navigate to the backend directory:
   ```bash
   cd fullstack/backend
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Create `.env` file with required credentials:
   ```bash
   cp .env.example .env
   ```

4. Configure environment variables (see Configuration section)

### Development

Start the development server:
```bash
uvicorn app:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

### Production

Run with production settings:
```bash
uvicorn app:app --host 0.0.0.0 --port 8000 --workers 4
```

## Configuration

### Environment Variables

Create a `.env` file with the following variables:

```bash
# API Keys
GROQ_API_KEY=your_groq_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_COLLECTION_NAME=ai_book_embeddings

# CORS Configuration
ALLOWED_ORIGINS=http://localhost:3000,https://ai-spec-driven-book-six.vercel.app

# Model Configuration (optional)
DEFAULT_MODEL=llama-3.3-70b-versatile
EMBEDDING_MODEL=embed-english-v3.0

# Token Budget (optional)
MAX_CONTEXT_TOKENS=7900
SAFETY_MARGIN_TOKENS=292
```

### Validation

The backend validates configuration on startup using `validate_config.py`. Missing or invalid credentials will prevent the service from starting.

## API Endpoints

### POST /chat

Main chat endpoint for conversational interactions.

**Request Body:**
```json
{
  "message": "What is ROS 2?",
  "conversation_history": [
    {
      "role": "user",
      "content": "Previous question"
    },
    {
      "role": "assistant",
      "content": "Previous answer"
    }
  ]
}
```

**Response:**
```json
{
  "response": "ROS 2 (Robot Operating System 2) is...",
  "citations": [
    {
      "url": "/docs/module-1/robotic-middleware-fundamentals",
      "title": "Robotic Middleware Fundamentals",
      "snippet": "ROS 2 is a flexible framework..."
    }
  ],
  "model_used": "llama-3.3-70b-versatile"
}
```

**Status Codes:**
- `200 OK`: Successful response
- `400 Bad Request`: Invalid request format
- `500 Internal Server Error`: Server-side error

### GET /health

Health check endpoint for monitoring.

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2026-02-13T10:30:00Z"
}
```

## Project Structure

```
backend/
├── app.py                    # FastAPI application and routes
├── agent.py                  # RAG agent with conversation management
├── retrieval.py              # Semantic search using Qdrant
├── models.py                 # Pydantic data models
├── api_models.py             # API contract definitions
├── ingest_local.py           # Content ingestion pipeline
├── validate_config.py        # Configuration validation
├── requirements.txt          # Python dependencies
├── .env.example              # Environment variable template
└── README.md                 # This file
```

## Key Components

### RAG Agent (agent.py)

Manages conversation flow with:
- **Context Window Optimization**: Prioritizes recent messages and relevant context
- **Token Budget Management**: Conservative limits to prevent truncation
- **Citation Extraction**: Transforms URLs for frontend routing
- **Error Handling**: Graceful degradation on API failures

### Retrieval System (retrieval.py)

Semantic search implementation:
- **Vector Search**: Qdrant with cosine similarity
- **Top-K Retrieval**: Configurable number of relevant chunks
- **Metadata Filtering**: Optional filtering by module or topic
- **Score Thresholding**: Minimum relevance scores for quality

### Content Ingestion (ingest_local.py)

Pipeline for embedding generation:
- **Document Parsing**: Markdown files from `docs/` directory
- **Chunking Strategy**: Semantic chunking with overlap
- **Batch Processing**: Efficient embedding generation
- **Metadata Extraction**: Module, title, and URL information

## API Integration

### Frontend Integration

The backend integrates with the Docusaurus frontend via:
1. **CORS Configuration**: Allows requests from frontend domains
2. **URL Transformation**: Citations use relative Docusaurus routes
3. **Stateless Design**: No server-side session storage
4. **Client-Side History**: Conversation context managed by frontend

### Example Integration

```javascript
// Frontend API call (useChatApi.js)
const response = await fetch('https://backend-url/chat', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    message: userMessage,
    conversation_history: conversationHistory,
  }),
});

const data = await response.json();
// data.response, data.citations, data.model_used
```

## Token Budget Strategy

The backend implements conservative token management:

- **Total Context Window**: 8192 tokens (model limit)
- **Allocated Budget**: 7900 tokens (96.4% utilization)
- **Safety Margin**: 292 tokens (3.6% buffer)

**Allocation Priority:**
1. System prompt (instructions)
2. Retrieved context (semantic search results)
3. Recent conversation history (last N messages)
4. Current user message

## Performance Considerations

- **Async Operations**: FastAPI with async/await for concurrent requests
- **Connection Pooling**: Reused connections to external APIs
- **Caching**: 15-minute cache for repeated queries (if implemented)
- **Retry Logic**: Exponential backoff for transient failures
- **Timeout Management**: Configurable timeouts for API calls

## Security

- **Environment Variables**: All secrets stored in `.env` (never committed)
- **CORS Restrictions**: Whitelist of allowed origins
- **Input Validation**: Pydantic models for request validation
- **Rate Limiting**: (Recommended for production)
- **API Key Rotation**: Regular rotation of service credentials

## Monitoring & Observability

### Health Checks

```bash
curl http://localhost:8000/health
```

### Logs

FastAPI provides automatic logging:
- Request/response logging
- Error tracking
- Performance metrics

### Metrics (Recommended)

Consider adding:
- Request latency (p50, p95, p99)
- Error rates by endpoint
- Token usage per request
- Cache hit rates

## Troubleshooting

### Common Issues

**Issue**: `GROQ_API_KEY not found`
- **Solution**: Ensure `.env` file exists with valid API key

**Issue**: `Qdrant connection failed`
- **Solution**: Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct

**Issue**: `CORS error from frontend`
- **Solution**: Add frontend URL to `ALLOWED_ORIGINS` in `.env`

**Issue**: `Token limit exceeded`
- **Solution**: Reduce `MAX_CONTEXT_TOKENS` or conversation history length

## Development Workflow

1. **Local Development**: Use `.env` with development credentials
2. **Testing**: Run unit tests with `pytest` (if implemented)
3. **Staging**: Deploy to staging environment with staging credentials
4. **Production**: Deploy with production credentials and monitoring

## Deployment

### Docker Deployment

```dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Environment-Specific Configuration

- **Development**: Local Qdrant instance, verbose logging
- **Staging**: Qdrant Cloud, moderate logging
- **Production**: Qdrant Cloud, error-only logging, monitoring enabled

## Contributing

To contribute to the backend:

1. Fork the repository
2. Create a feature branch
3. Make your changes with tests
4. Ensure all tests pass
5. Submit a pull request

## License

This project is licensed under the MIT License.

## Support

For issues or questions:
- Check the troubleshooting section
- Review API documentation
- Open an issue on GitHub
- Contact the development team

---

**Built with FastAPI, Groq, Cohere, and Qdrant**

*Part of the AI-Spec Driven Book educational platform*
