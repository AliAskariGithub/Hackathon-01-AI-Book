# Research: FastAPI Backend with Docusaurus Chatbot

**Feature**: 3-fastapi-chatbot | **Date**: 2025-12-28

## R1: FastAPI + Hugging Face Spaces Integration

**Question**: How to deploy FastAPI on Hugging Face Spaces with proper entry point?

**Decision**: Use `app.py` as entry point with Gradio-style FastAPI mount

**Rationale**:
- Hugging Face Spaces expects `app.py` as main entry point
- FastAPI apps need to be mounted as ASGI application
- Space type should be "Docker" or "Gradio" (Gradio supports FastAPI mounting)

**Implementation Pattern**:
```python
# app.py (Hugging Face Spaces entry point)
import gradio as gr
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()
# Mount CORS, routes, etc.

# For Gradio Spaces: mount FastAPI to Gradio
demo = gr.Blocks()
app = gr.mount_gradio_app(app, demo, path="/gradio")

# Or use Docker Space type with uvicorn directly
```

**Alternatives Considered**:
- Pure Docker deployment: More control but requires Dockerfile management
- Gradio-only: Simpler but doesn't give REST API flexibility

**Selected Approach**: Docker Space with `uvicorn app:app` for cleaner REST API

---

## R2: Docusaurus @theme/Root Swizzling

**Question**: How to render global widget across all Docusaurus pages?

**Decision**: Swizzle `@theme/Root` component to wrap entire app with Chatbot

**Rationale**:
- Root component wraps the entire Docusaurus application
- Persists across page navigation (SPA behavior)
- Standard Docusaurus pattern for global components

**Implementation Pattern**:
```bash
# Swizzle Root component
npm run swizzle @docusaurus/theme-classic Root -- --wrap
```

Creates `src/theme/Root.js`:
```jsx
import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
```

**Alternatives Considered**:
- Custom plugin: More complex, overkill for single component
- Layout swizzle: Doesn't persist across all page types
- ClientModule: Doesn't support React components well

---

## R3: React Context for SPA State Persistence

**Question**: How to maintain chat state during Docusaurus navigation?

**Decision**: React Context at Root level + localStorage for cross-session

**Rationale**:
- Context at Root level survives client-side navigation
- localStorage provides persistence across browser sessions
- No server-side state needed (stateless backend)

**Implementation Pattern**:
```jsx
// ChatContext.js
const ChatContext = createContext();

export function ChatProvider({children}) {
  const [messages, setMessages] = useState(() => {
    const saved = localStorage.getItem('chat_messages');
    return saved ? JSON.parse(saved) : [];
  });

  useEffect(() => {
    localStorage.setItem('chat_messages', JSON.stringify(messages));
  }, [messages]);

  return (
    <ChatContext.Provider value={{messages, setMessages}}>
      {children}
    </ChatContext.Provider>
  );
}
```

**Alternatives Considered**:
- Redux/Zustand: Overkill for simple chat state
- sessionStorage: Doesn't persist across tabs/sessions
- URL state: Not appropriate for conversation data

---

## R4: Citation URL Transformation

**Question**: How to transform absolute Qdrant URLs to relative Docusaurus routes?

**Decision**: Backend transforms URLs using regex pattern matching

**Rationale**:
- Qdrant stores full URLs (e.g., `https://site.github.io/repo/docs/page`)
- Frontend needs relative paths for SPA navigation (e.g., `/docs/page`)
- Transformation should happen server-side for consistency

**Implementation Pattern**:
```python
def transform_citation_url(url: str, base_url: str) -> str:
    """Transform absolute URL to relative Docusaurus route."""
    # Remove base URL prefix
    # https://user.github.io/repo/docs/page -> /docs/page
    if url.startswith(base_url):
        return url[len(base_url):]
    # Handle variations (http vs https, trailing slashes)
    parsed = urlparse(url)
    return parsed.path
```

**Configuration**:
- `BOOK_BASE_URL` environment variable for transformation
- Fallback to path extraction if base doesn't match

---

## R5: CORS Configuration for Multiple Origins

**Question**: How to configure CORS for dev, production, and HF preview?

**Decision**: Environment-based origin list with pattern matching for GitHub Pages

**Rationale**:
- Need localhost for development
- Need GitHub Pages domain for production
- Need HuggingFace domain for Space preview

**Implementation Pattern**:
```python
from fastapi.middleware.cors import CORSMiddleware

# Configure based on environment
CORS_ORIGINS = [
    "http://localhost:3000",
    "http://localhost:8000",
    "https://aliaskarigithub.github.io",  # GitHub Pages
    "https://huggingface.co",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)
```

**Note**: GitHub Pages username from docusaurus.config.js: `AliAskariGithub`

---

## R6: Docusaurus Theme Variables for Dark/Light Mode

**Question**: How to style chat widget to match Docusaurus theme?

**Decision**: Use CSS custom properties from `--ifm-*` namespace

**Rationale**:
- Docusaurus uses CSS custom properties for theming
- Automatically updates on theme toggle
- No JavaScript needed for theme detection

**Key Variables**:
```css
/* Light mode (default) */
--ifm-background-color: #ffffff;
--ifm-color-primary: #2e8555;
--ifm-font-color-base: #1c1e21;

/* Dark mode (auto-applied via [data-theme='dark']) */
[data-theme='dark'] {
  --ifm-background-color: #1b1b1d;
  --ifm-font-color-base: #e3e3e3;
}
```

**Chatbot CSS Pattern**:
```css
.chatbot-panel {
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  border: 1px solid var(--ifm-color-emphasis-300);
}

.chatbot-button {
  background: var(--ifm-color-primary);
  color: white;
}
```

---

## R7: Keyboard Accessibility and Focus Trap

**Question**: How to implement focus trap for accessible modal?

**Decision**: Use `focus-trap-react` library or manual implementation

**Rationale**:
- Focus trap prevents keyboard users from navigating outside modal
- Required for WCAG 2.1 compliance
- Well-supported library available

**Implementation Pattern**:
```jsx
import FocusTrap from 'focus-trap-react';

function ChatPanel({isOpen, onClose}) {
  return isOpen ? (
    <FocusTrap>
      <div
        className="chat-panel"
        role="dialog"
        aria-label="Chat with book assistant"
      >
        <button onClick={onClose} aria-label="Close chat">×</button>
        <div className="messages" role="log" aria-live="polite">
          {/* messages */}
        </div>
        <input
          type="text"
          aria-label="Type your question"
          onKeyDown={(e) => e.key === 'Escape' && onClose()}
        />
      </div>
    </FocusTrap>
  ) : null;
}
```

**Alternative**: Manual `tabindex` management (more complex, error-prone)

---

## R8: Error Handling and Retry Logic

**Question**: How to handle various error states gracefully?

**Decision**: Typed error responses with frontend-mapped messages

**Backend Error Types**:
```python
class ErrorType(str, Enum):
    RETRIEVAL = "retrieval"
    GENERATION = "generation"
    RATE_LIMIT = "rate_limit"
    VALIDATION = "validation"
    INTERNAL = "internal"

# Response includes error type for frontend handling
{
    "answer": null,
    "citations": [],
    "conversation_id": "...",
    "error": "rate_limit"  # or detailed message
}
```

**Frontend Handling**:
```javascript
const ERROR_MESSAGES = {
  'rate_limit': 'Too many requests. Please wait 60 seconds.',
  'retrieval': 'Could not search book content. Please try again.',
  'generation': 'Could not generate response. Please try again.',
  'network': 'Unable to connect. Check your internet connection.',
  'timeout': 'Request timed out. Please try again.',
};
```

---

## R9: Spec-2 Agent Integration

**Question**: How to integrate existing RAG agent with FastAPI?

**Decision**: Import `process_query` from agent.py and wrap in async endpoint

**Rationale**:
- Spec-2 already implements full RAG pipeline
- `process_query(state, query)` returns `(response, citations)`
- Need to manage AgentState per request (stateless backend)

**Implementation Pattern**:
```python
from agent import create_agent, process_query
from models import AgentState, Message

@app.post("/api/chat")
async def chat(request: ChatRequest):
    # Create fresh agent state per request
    agent = create_agent()

    # Reconstruct conversation from request history
    for msg in request.conversation_history:
        agent.conversation.add_message(
            Message(role=msg.role, content=msg.content)
        )

    # Process query
    response, citations = await process_query(agent, request.query)

    # Transform citation URLs
    transformed_citations = [
        {"title": c.title, "url": transform_url(c.url)}
        for c in citations
    ]

    return ChatResponse(
        answer=response,
        citations=transformed_citations,
        conversation_id=request.conversation_id or str(uuid4())
    )
```

---

## R10: Hugging Face Space Configuration

**Question**: What files are needed for HF Spaces deployment?

**Decision**: Docker Space type with requirements.txt and app.py

**Required Files**:
```
fullstack/backend/
├── app.py              # FastAPI entry point (HF Spaces)
├── requirements.txt    # Dependencies for HF Spaces
├── Dockerfile          # Optional, for Docker Space type
├── agent.py            # Existing RAG agent (Spec-2)
├── retrieval.py        # Existing retrieval (Spec-1)
├── models.py           # Existing models
└── .env.example        # Environment template
```

**requirements.txt**:
```
fastapi>=0.104.0
uvicorn>=0.24.0
python-dotenv>=1.0
cohere>=5.0
qdrant-client>=1.7
openai>=1.0.0
tiktoken>=0.5.0
tenacity>=8.0.0
```

**HF Space README.md** (metadata):
```yaml
---
title: Isaac Sim Book Chatbot
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
app_port: 7860
---
```

---

## Summary

| Topic | Decision | Rationale |
|-------|----------|-----------|
| HF Deployment | Docker Space + uvicorn | Clean REST API, standard deployment |
| Global Widget | @theme/Root swizzle | Standard Docusaurus pattern |
| State Persistence | React Context + localStorage | SPA-friendly, cross-session |
| URL Transform | Server-side regex | Consistent, configurable |
| CORS | Environment-based origins | Dev/prod flexibility |
| Theme Integration | CSS custom properties | Auto-updates, no JS |
| Accessibility | focus-trap-react + ARIA | WCAG compliance |
| Error Handling | Typed errors + mapped messages | User-friendly |
| Agent Integration | Import existing process_query | Reuse Spec-2 |
| HF Files | requirements.txt + app.py | Standard HF pattern |
