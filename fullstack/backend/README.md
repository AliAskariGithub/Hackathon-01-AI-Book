# RAG Pipeline Backend

RAG (Retrieval-Augmented Generation) pipeline for book content embeddings, semantic retrieval, and conversational AI agent.

## Setup

### 1. Install Dependencies

```bash
cd fullstack/backend
uv sync
```

### 2. Configure Environment

Copy the example environment file and add your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your API keys:

| Variable | Description | Source |
|----------|-------------|--------|
| `COHERE_API_KEY` | Cohere API key | [dashboard.cohere.com](https://dashboard.cohere.com/api-keys) |
| `QDRANT_URL` | Qdrant Cloud cluster URL | [cloud.qdrant.io](https://cloud.qdrant.io/) |
| `QDRANT_API_KEY` | Qdrant API key | Cluster settings |
| `BOOK_BASE_URL` | Target book URL | Your deployed Docusaurus site |
| `OPENROUTER_API_KEY` | OpenRouter API key | [openrouter.ai](https://openrouter.ai/keys) |
| `BATCH_SIZE` | Embedding batch size (default: 96) | Optional |
| `COLLECTION_NAME` | Qdrant collection name | Optional |
| `LOG_LEVEL` | Logging level (INFO/DEBUG/ERROR) | Optional |

### 3. Verify Setup

```bash
# Test Cohere connection
python -c "
import os; from dotenv import load_dotenv; import cohere
load_dotenv()
co = cohere.Client(os.getenv('COHERE_API_KEY'))
print('Cohere OK:', len(co.embed(texts=['test'], model='embed-english-v3.0', input_type='search_query').embeddings[0]), 'dims')
"

# Test Qdrant connection
python -c "
import os; from dotenv import load_dotenv; from qdrant_client import QdrantClient
load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
print('Qdrant OK:', len(client.get_collections().collections), 'collections')
"
```

## Usage

### Content Ingestion

Scrape book content, generate embeddings, and store in Qdrant:

```bash
# Full ingestion
python main.py --url https://your-username.github.io/your-book

# Dry run (scrape and chunk without storing)
python main.py --url https://... --dry-run

# Custom collection name
python main.py --url https://... --collection my_collection
```

**Output:**
```
==================================================
Ingestion Complete
==================================================
Pages discovered: 45
Pages scraped: 42
Chunks created: 312
Embeddings generated: 312
Vectors stored: 312
Duplicates skipped: 0
Duration: 127.3s
```

### Semantic Search

Query the indexed content:

```bash
# Search for content
python retrieval.py --query "What is URDF?"

# Limit results
python retrieval.py --query "forward kinematics" --top-k 3

# Filter by similarity threshold
python retrieval.py --query "Gazebo setup" --threshold 0.7
```

**Output:**
```
Query: What is URDF?

1. [0.847] URDF Mapping - Real to Simulated
   URDF (Unified Robot Description Format) is an XML format...
   URL: https://...docs/module-2/urdf-mapping-real-simulated

2. [0.812] Links, Joints, and Coordinate Frames
   The URDF file defines the robot's kinematic structure...
   URL: https://...docs/module-2/links-joints-coordinate-frames
```

### Validation

Validate retrieval quality with test queries:

```bash
# Run validation (default 0.7 threshold)
python retrieval.py --validate

# Custom threshold
python retrieval.py --validate --threshold 0.8
```

**Output:**
```
==================================================
Validation Report
==================================================
Threshold: 0.70
Total queries: 6
Passed: 6
Failed: 0
Pass rate: 100.0%
Average similarity: 0.823

All validation checks passed!
```

## RAG Agent

Interactive conversational assistant that answers questions using retrieved book content.

### Quick Start

```bash
# Start interactive chat
python agent.py
```

**Example Session:**
```
Book Assistant (type 'quit' to exit, 'clear' to reset, 'help' for commands)

> What is URDF?
Thinking...

URDF (Unified Robot Description Format) is an XML format for describing
robots in ROS. It defines the robot's links, joints, and kinematic
structure [Source: URDF Basics](https://example.com/docs/urdf).

> Tell me more about that
Thinking...

URDF files contain several key elements... [continues with context retention]

> quit
Goodbye!
```

### Commands

| Command | Description |
|---------|-------------|
| `quit`, `exit`, `bye`, `q` | Exit the assistant |
| `clear`, `reset` | Clear conversation history |
| `help`, `?` | Show available commands |

### Configuration

The agent uses sensible defaults but can be customized via `AgentConfig`:

| Setting | Default | Description |
|---------|---------|-------------|
| `model` | `meta-llama/llama-3.2-3b-instruct:free` | LLM model via OpenRouter |
| `temperature` | `0.7` | Response creativity |
| `max_tokens` | `2048` | Max response length |
| `retrieval_top_k` | `5` | Max context chunks |
| `retrieval_threshold` | `0.3` | Min similarity score |

### Programmatic Usage

```python
import asyncio
from agent import create_agent, process_query

async def main():
    agent = create_agent()
    response, citations = await process_query(agent, "What is URDF?")
    print(response)
    print(f"Sources: {len(citations)}")

asyncio.run(main())
```

### Token Budget

The agent manages context within 8,192 tokens:

| Component | Budget |
|-----------|--------|
| System prompt | 400 |
| Retrieved context | 4,000 |
| Conversation history | 2,500 |
| Response buffer | 1,000 |
| **Total** | **7,900** |

## Testing

```bash
# Run all tests
pytest test/ -v

# Run specific test file
pytest test/test_main.py -v
pytest test/test_retrieval.py -v
pytest test/test_agent.py -v

# Run with coverage
pytest test/test_agent.py --cov=agent --cov-report=term-missing -v
```

## Project Structure

```
fullstack/backend/
├── main.py           # Content ingestion pipeline
├── retrieval.py      # Semantic search and validation
├── agent.py          # Conversational RAG agent
├── models.py         # Data models (dataclasses)
├── test/
│   ├── test_main.py
│   ├── test_retrieval.py
│   └── test_agent.py
├── .env.example      # Environment template
├── .gitignore
├── pyproject.toml    # Project configuration
└── README.md         # This file
```

## API Reference

### main.py Functions

| Function | Description |
|----------|-------------|
| `scrape_sitemap(base_url)` | Fetch and parse sitemap.xml |
| `scrape_page(url)` | Extract clean text from page |
| `chunk_content(page, co)` | Split content into chunks |
| `generate_embeddings(chunks, co)` | Generate Cohere embeddings |
| `init_collection(client)` | Create Qdrant collection |
| `store_vectors(embeddings, chunks, client)` | Store vectors with dedup |
| `ingest_book(base_url)` | Full ingestion pipeline |

### retrieval.py Functions

| Function | Description |
|----------|-------------|
| `search(query, top_k, threshold)` | Semantic search |
| `validate(test_queries, threshold)` | Run validation suite |
| `format_results(results)` | Format for display |

### agent.py Functions

| Function | Description |
|----------|-------------|
| `create_agent(config)` | Initialize agent with config |
| `process_query(state, query)` | Process query through RAG pipeline |
| `run_cli()` | Start interactive CLI session |
| `count_tokens(text)` | Count tokens using tiktoken |
| `retrieve_context(query, top_k, threshold)` | Get relevant chunks |
| `generate_response(messages, config)` | Call LLM with retry |
| `extract_citations(response, context)` | Parse citations from response |
| `handle_error(error)` | Convert exception to user message |

## Troubleshooting

### "COHERE_API_KEY not found"
- Ensure `.env` file exists
- Check that key is set without quotes

### "Connection refused" to Qdrant
- Verify QDRANT_URL includes `https://`
- Check API key is correct
- Ensure cluster is running

### Low similarity scores
- Verify book content is accessible
- Check HTML extraction captured main content
- Review chunk sizes

### "OPENROUTER_API_KEY not found"
- Ensure `.env` file has `OPENROUTER_API_KEY=sk-or-v1-...`
- Get free key at [openrouter.ai/keys](https://openrouter.ai/keys)

### Agent shows "Configuration error"
- Check all required API keys are set
- Verify Qdrant collection exists (run ingestion first)

## FastAPI REST API

The backend also provides a REST API for the Docusaurus chat widget.

### API Endpoints

#### `GET /health`

Health check endpoint.

```bash
curl http://localhost:8000/health
```

#### `POST /api/chat`

Process a chat request through the RAG pipeline.

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"What is URDF?"}'
```

**Response:**
```json
{
  "answer": "URDF (Unified Robot Description Format) is...",
  "citations": [{"title": "URDF Basics", "url": "/docs/urdf-basics", "score": 0.85}],
  "conversation_id": "uuid",
  "error": null
}
```

### Running the API Server

```bash
# Development
uvicorn app:app --reload --port 8000

# Production
uvicorn app:app --host 0.0.0.0 --port 8000
```

### Hugging Face Spaces Deployment

1. Create a new Space: `{username}/isaac-sim-chatbot-api`
2. Select **Docker** as the SDK
3. Upload files: `app.py`, `api_models.py`, `validate_config.py`, `agent.py`, `retrieval.py`, `models.py`, `requirements.txt`
4. Configure secrets in HF Space Settings:
   - `COHERE_API_KEY`
   - `QDRANT_URL`, `QDRANT_API_KEY`
   - `OPENROUTER_API_KEY`
   - `BOOK_BASE_URL`
   - `ALLOWED_ORIGINS`

See `specs/003-fastapi-chatbot/quickstart.md` for detailed deployment instructions.

## License

See repository root for license information.
