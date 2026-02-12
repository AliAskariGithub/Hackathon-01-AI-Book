# Quickstart: OpenAI Agent with RAG Retrieval Integration

**Feature Branch**: `002-rag-agent`
**Date**: 2025-12-28
**Estimated Setup Time**: 10 minutes

## Prerequisites

- Python 3.11+
- Spec-1 completed (Qdrant collection populated with book embeddings)
- API keys: OpenRouter, Cohere, Qdrant

## Step 1: Install Dependencies

```bash
cd fullstack/backend
pip install openai-agents openai tiktoken tenacity
```

Or add to `pyproject.toml`:
```toml
[project.dependencies]
openai-agents = ">=0.0.1"
openai = ">=1.0.0"
tiktoken = ">=0.5.0"
tenacity = ">=8.0.0"
# Existing from Spec-1
cohere = ">=5.0.0"
qdrant-client = ">=1.7.0"
python-dotenv = ">=1.0.0"
```

## Step 2: Configure Environment

Add to `.env` file:
```bash
# OpenRouter (new for Spec-2)
GROQ_API_KEY=sk-or-v1-your-key-here

# Existing from Spec-1
COHERE_API_KEY=your-cohere-key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
```

Get your OpenRouter API key at: https://openrouter.ai/keys

## Step 3: Verify Spec-1 Setup

Ensure retrieval is working:
```bash
python retrieval.py --query "What is URDF?" --top-k 3
```

Expected output:
```
Query: What is URDF?

1. [0.650] URDF Mapping - Part 1
   URDF (Unified Robot Description Format) is...
   URL: https://...

2. [0.620] Robot Description
   ...
```

## Step 4: Run the Agent

```bash
python agent.py
```

Expected output:
```
Book Assistant (type 'quit' to exit, 'clear' to reset conversation)
> What is URDF?

URDF (Unified Robot Description Format) is an XML format used to describe
robot models in ROS and Isaac Sim. It defines the robot's physical structure
including links, joints, and their properties [Source: URDF Mapping - Part 1](https://...).

> How does it relate to Isaac Sim?

Isaac Sim can import URDF files to create robot simulations. The URDF defines
the kinematic chain while Isaac Sim adds physics and rendering
[Source: Isaac Sim Overview](https://...).

> quit
Goodbye!
```

## Step 5: Test Commands

| Command | Action |
|---------|--------|
| `quit`, `exit`, `bye` | Exit the agent |
| `clear`, `reset` | Clear conversation history |
| `help` | Show available commands |
| Ctrl+C | Force exit |

## Troubleshooting

### "GROQ_API_KEY not set"
```bash
# Check .env file exists and contains the key
cat .env | grep OPENROUTER
```

### "No results found" from retrieval
```bash
# Verify Qdrant collection has data
python retrieval.py --validate
```

### Rate limit errors
The agent will automatically retry with backoff. If persistent:
- Wait 60 seconds before retrying
- Check OpenRouter dashboard for usage limits

### Slow responses
Free-tier models may have variable latency (5-30 seconds). First request may be slower due to cold start.

## Project Structure

```
fullstack/backend/
├── agent.py          # Main agent implementation (this spec)
├── retrieval.py      # Vector search (Spec-1)
├── models.py         # Data models (Spec-1 + extensions)
├── main.py           # Ingestion pipeline (Spec-1)
├── .env              # Environment variables
└── test/
    └── test_agent.py # Agent tests
```

## Next Steps

1. Run test suite: `python -m pytest test/test_agent.py -v`
2. Try multi-turn conversations
3. Test out-of-scope query handling
4. Proceed to Spec-3 for REST API integration
