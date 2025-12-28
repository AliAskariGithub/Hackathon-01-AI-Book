# Quickstart: RAG Pipeline Setup

**Feature**: 001-rag-book-embeddings
**Date**: 2025-12-28

## Prerequisites

- Python 3.11+ installed
- [uv](https://docs.astral.sh/uv/) package manager installed
- Cohere account (free tier: https://dashboard.cohere.com/)
- Qdrant Cloud account (free tier: https://cloud.qdrant.io/)
- Deployed Docusaurus book on GitHub Pages

## Step 1: Initialize Backend Project

```bash
# Navigate to fullstack directory
cd fullstack

# Create backend directory with uv
mkdir backend
cd backend

# Initialize uv project
uv init

# Install dependencies
uv add cohere qdrant-client beautifulsoup4 requests python-dotenv
uv add --dev pytest
```

## Step 2: Configure Environment

Create `.env` file from template:

```bash
# Copy example
cp .env.example .env

# Edit with your credentials
```

**.env.example** contents:
```env
# Cohere API (https://dashboard.cohere.com/api-keys)
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud (https://cloud.qdrant.io/)
QDRANT_URL=https://your-cluster-id.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Optional: Default book URL
BOOK_BASE_URL=https://username.github.io/repo-name

# Optional: Logging level (DEBUG, INFO, WARNING, ERROR)
LOG_LEVEL=INFO
```

## Step 3: Get API Credentials

### Cohere API Key
1. Go to https://dashboard.cohere.com/
2. Sign up or log in
3. Navigate to API Keys
4. Copy your API key to `.env`

### Qdrant Cloud
1. Go to https://cloud.qdrant.io/
2. Create a free cluster
3. Copy the cluster URL (e.g., `https://abc123.us-east4-0.gcp.cloud.qdrant.io`)
4. Generate an API key from cluster settings
5. Add both to `.env`

## Step 4: Verify Setup

```bash
# Activate environment
source .venv/bin/activate  # Unix
# or
.venv\Scripts\activate     # Windows

# Test Cohere connection
python -c "
import os
from dotenv import load_dotenv
import cohere

load_dotenv()
co = cohere.Client(os.getenv('COHERE_API_KEY'))
response = co.embed(texts=['test'], model='embed-english-v3.0', input_type='search_query')
print(f'Cohere OK: {len(response.embeddings[0])} dimensions')
"

# Test Qdrant connection
python -c "
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
collections = client.get_collections()
print(f'Qdrant OK: {len(collections.collections)} collections')
"
```

## Step 5: Run Ingestion

```bash
# Run the ingestion pipeline
python main.py --url https://your-username.github.io/your-book

# Expected output:
# INFO: Fetching sitemap from https://...
# INFO: Discovered 45 pages
# INFO: Scraping pages... [45/45]
# INFO: Created 312 chunks
# INFO: Generating embeddings... [312/312]
# INFO: Storing vectors...
# INFO: Ingestion complete!
# - Pages scraped: 45
# - Chunks created: 312
# - Vectors stored: 312
# - Duration: 127.3s
```

## Step 6: Test Retrieval

```bash
# Interactive query
python retrieval.py --query "What is URDF?"

# Expected output:
# Query: What is URDF?
#
# Results:
# 1. [0.847] URDF Mapping - Real to Simulated
#    "URDF (Unified Robot Description Format) is an XML format..."
#    URL: https://...docs/module-2/urdf-mapping-real-simulated
#
# 2. [0.812] Links, Joints, and Coordinate Frames
#    "The URDF file defines the robot's kinematic structure..."
#    URL: https://...docs/module-2/links-joints-coordinate-frames
```

## Step 7: Run Validation

```bash
# Run validation suite
python retrieval.py --validate

# Expected output:
# Validation Report
# =================
# Threshold: 0.70
# Total queries: 5
# Passed: 5
# Failed: 0
# Pass rate: 100%
# Average similarity: 0.823
#
# All validation checks passed!
```

## Troubleshooting

### "COHERE_API_KEY not found"
- Ensure `.env` file exists in `fullstack/backend/`
- Check that `python-dotenv` is installed
- Verify the key is set without quotes around the value

### "Connection refused" to Qdrant
- Check QDRANT_URL format (must include https://)
- Verify API key is correct
- Ensure cluster is running (check Qdrant Cloud dashboard)

### "Rate limit exceeded"
- Wait 60 seconds and retry
- Reduce batch size in `main.py` if persistent
- Check Cohere dashboard for usage limits

### Low similarity scores
- Verify book content is accessible (try URLs manually)
- Check that HTML extraction captured main content
- Review chunk sizes (may need adjustment)

## Project Structure After Setup

```
fullstack/backend/
├── main.py           # Ingestion script
├── retrieval.py      # Query/validation script
├── test/
│   ├── test_main.py
│   └── test_retrieval.py
├── .env              # Your credentials (gitignored)
├── .env.example      # Template
├── pyproject.toml    # uv configuration
└── README.md
```

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement `main.py` following the API contract
3. Implement `retrieval.py` for query testing
4. Run validation to verify >0.7 similarity threshold
