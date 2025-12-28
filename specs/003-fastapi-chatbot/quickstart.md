# Quickstart: FastAPI Backend with Docusaurus Chatbot

**Feature**: 3-fastapi-chatbot | **Date**: 2025-12-28

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed
- Spec-1 complete: Qdrant populated with book embeddings
- Spec-2 complete: RAG agent functional
- API keys configured:
  - `COHERE_API_KEY` (embeddings)
  - `QDRANT_URL` and `QDRANT_API_KEY` (vector DB)
  - `OPENROUTER_API_KEY` (LLM)

## Quick Start (5 minutes)

### 1. Start Backend Locally

```bash
cd fullstack/backend

# Install dependencies (if not already)
pip install fastapi uvicorn

# Start API server
uvicorn app:app --reload --port 8000
```

**Verify**: `curl http://localhost:8000/health`

### 2. Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is URDF?"}'
```

**Expected Response**:
```json
{
  "answer": "URDF (Unified Robot Description Format) is an XML format...",
  "citations": [{"title": "URDF Basics", "url": "/docs/module-2/urdf-basics"}],
  "conversation_id": "...",
  "error": null
}
```

### 3. Start Docusaurus Frontend

```bash
cd fullstack/frontend-book

# Install if needed
npm install

# Configure backend URL
export BACKEND_URL=http://localhost:8000

# Start dev server
npm start
```

**Verify**: Open http://localhost:3000 and see chat button in bottom-right

### 4. Test End-to-End

1. Click floating chat button
2. Type "What is URDF?"
3. See response with clickable citations
4. Click citation → navigates to book page
5. Navigate to another page → chat persists

## File Structure

```
fullstack/
├── backend/
│   ├── app.py              # FastAPI entry point (NEW)
│   ├── api_models.py       # Request/Response schemas (NEW)
│   ├── agent.py            # RAG agent (Spec-2)
│   ├── retrieval.py        # Vector search (Spec-1)
│   ├── models.py           # Data models
│   └── requirements.txt    # HF Spaces deps (NEW)
│
└── frontend-book/
    ├── src/
    │   ├── components/
    │   │   └── Chatbot/    # Chat widget (NEW)
    │   │       ├── index.jsx
    │   │       ├── ChatPanel.jsx
    │   │       ├── ChatMessage.jsx
    │   │       ├── ChatContext.jsx
    │   │       └── Chatbot.module.css
    │   └── theme/
    │       └── Root.js     # Swizzled for global widget (NEW)
    └── docusaurus.config.js  # Add customFields.backendUrl
```

## Configuration

### Backend Environment (.env)

```bash
# Required from Spec-1/2
COHERE_API_KEY=your-cohere-key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
OPENROUTER_API_KEY=sk-or-v1-your-key

# New for Spec-3
BOOK_BASE_URL=https://aliaskarigithub.github.io
CORS_ORIGINS=http://localhost:3000,http://localhost:8000,https://aliaskarigithub.github.io
```

### Frontend Configuration (docusaurus.config.js)

```javascript
const config = {
  // ... existing config
  customFields: {
    backendUrl: process.env.BACKEND_URL || 'https://huggingface.co/spaces/user/chatbot',
  },
};
```

## Hugging Face Deployment

### 1. Create Space

1. Go to https://huggingface.co/new-space
2. Select "Docker" as SDK
3. Name your space (e.g., "isaac-sim-chatbot")

### 2. Upload Files

```bash
# From fullstack/backend/
git clone https://huggingface.co/spaces/YOUR_USER/YOUR_SPACE
cp app.py api_models.py agent.py retrieval.py models.py requirements.txt YOUR_SPACE/
cd YOUR_SPACE
git add . && git commit -m "Initial deployment" && git push
```

### 3. Configure Secrets

In Space Settings → Repository secrets:
- `COHERE_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `OPENROUTER_API_KEY`
- `BOOK_BASE_URL`

### 4. Verify Deployment

```bash
curl https://YOUR_USER-YOUR_SPACE.hf.space/health
curl -X POST https://YOUR_USER-YOUR_SPACE.hf.space/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is URDF?"}'
```

## Development Workflow

### Backend Changes

```bash
cd fullstack/backend
uvicorn app:app --reload --port 8000
# Changes auto-reload
```

### Frontend Changes

```bash
cd fullstack/frontend-book
npm start
# Changes auto-reload via Docusaurus
```

### Testing Chat Flow

1. Open browser DevTools → Network tab
2. Submit question in chat
3. Verify POST to /api/chat
4. Check response format matches contract

## Troubleshooting

### "Unable to connect" Error
- Check backend is running: `curl http://localhost:8000/health`
- Check CORS: Backend must allow frontend origin
- Check network: No firewall blocking

### "Rate limit" Error
- Wait 60 seconds (Hugging Face free tier limit)
- Check HF Space logs for details

### Citations Not Linking
- Verify URLs are relative (`/docs/...` not `https://...`)
- Check `BOOK_BASE_URL` is correct
- Verify page exists in Docusaurus

### Chat Disappears on Navigation
- Verify Root.js swizzle is correct
- Check ChatContext wraps entire app
- Clear localStorage and retry

## API Testing with curl

```bash
# Health check
curl http://localhost:8000/health

# Simple question
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is URDF?"}'

# With conversation history
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How is it used in Isaac Sim?",
    "conversation_history": [
      {"role": "user", "content": "What is URDF?"},
      {"role": "assistant", "content": "URDF is..."}
    ],
    "conversation_id": "test-123"
  }'
```

## Next Steps

After basic setup works:

1. **Style chat widget** to match book theme
2. **Add mobile responsiveness** for <768px screens
3. **Implement keyboard navigation** for accessibility
4. **Deploy to Hugging Face** for production
5. **Update docusaurus.config.js** with production backend URL
