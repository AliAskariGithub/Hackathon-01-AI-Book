# Feature Specification: OpenAI Agent with RAG Retrieval Integration

**Feature Branch**: `002-rag-agent`
**Created**: 2025-12-28
**Status**: Draft
**Input**: User description: "OpenAI Agent with RAG retrieval integration for book content Q&A"

## Technical Approach

### OpenAI Agents SDK + OpenRouter Integration

This implementation uses the **OpenAI Agents SDK** with a `base_url` override to route requests through **OpenRouter**. This approach provides:
- Native OpenAI SDK compatibility with agent patterns
- Access to free-tier models via OpenRouter
- Standard conversation management and tool calling patterns

**Configuration**:
```
base_url: https://openrouter.ai/api/v1
api_key: OPENROUTER_API_KEY (environment variable)
```

### Model Specification

- **Primary Model**: `meta-llama/llama-3.2-3b-instruct:free`
- **Fallback Model**: `mistralai/devstral-2512:free` or `xiaomi/mimo-v2-flash:free`
- **Context Window**: 8,192 tokens (Llama 3.2 3B)
- **Max Output Tokens**: 2,048 tokens per response
- **Rate Limits**: Subject to OpenRouter free tier limits

### Documentation Reference

For OpenAI Agents SDK usage, refer to context7 MCP tool for up-to-date documentation on:
- Agent creation and configuration
- Tool/function calling patterns
- Conversation state management
- OpenRouter base_url override patterns

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Single Question Answering (Priority: P1)

As a user, I want to ask a natural language question about the book content and receive an accurate, grounded answer with citations so that I can quickly find information without reading the entire book.

**Why this priority**: This is the core value proposition - enabling users to get answers from book content. Without this, no other feature provides value.

**Independent Test**: Can be tested by running the agent with a single question and verifying:
- The agent retrieves relevant content from the vector database
- The response directly answers the question
- Citations with source URLs are included in the response

**Acceptance Scenarios**:

1. **Given** the agent is running and vectors are indexed, **When** a user asks "What is URDF?", **Then** the agent retrieves relevant chunks and responds with an accurate answer citing the source page.
2. **Given** the agent receives a question, **When** the question relates to book content, **Then** the response includes at least one citation with source URL.
3. **Given** the agent receives a question, **When** the retrieved content has low relevance, **Then** the agent acknowledges uncertainty rather than fabricating an answer.

---

### User Story 2 - Multi-turn Conversation (Priority: P1)

As a user, I want to have a back-and-forth conversation where the agent remembers previous context so that I can ask follow-up questions without repeating myself.

**Why this priority**: Conversational context is essential for natural interaction. Users expect to ask follow-up questions like "Can you explain that further?" or "What about the second point?"

**Independent Test**: Can be tested by asking an initial question, then asking a follow-up that references the previous answer, and verifying the agent maintains context.

**Acceptance Scenarios**:

1. **Given** the user has asked about forward kinematics, **When** the user asks "How does this relate to inverse kinematics?", **Then** the agent understands "this" refers to forward kinematics and provides a relevant answer.
2. **Given** an ongoing conversation, **When** the user asks "Tell me more about that", **Then** the agent correctly identifies what "that" refers to from context.
3. **Given** a multi-turn conversation, **When** the conversation exceeds a reasonable length, **Then** the agent maintains coherence by summarizing or retaining key context points.

---

### User Story 3 - Out-of-Scope Query Handling (Priority: P2)

As a user, I want the agent to gracefully handle questions outside the book's domain so that I understand the agent's limitations and don't receive misleading information.

**Why this priority**: Proper boundary handling builds user trust and prevents hallucination. Important for reliability but secondary to core Q&A functionality.

**Independent Test**: Can be tested by asking questions unrelated to the book content and verifying the agent politely declines or redirects.

**Acceptance Scenarios**:

1. **Given** the user asks about an unrelated topic (e.g., "What's the weather today?"), **When** the agent searches and finds no relevant content, **Then** the agent explains it can only answer questions about the book content.
2. **Given** the user asks a partially related question, **When** retrieval returns low-confidence results, **Then** the agent attempts to answer the related parts while acknowledging limitations.

---

### Edge Cases

- **What happens when the vector database is empty or unavailable?** The agent should inform the user that content is not yet indexed or temporarily unavailable, and suggest trying again later.
- **How does the agent handle ambiguous questions?** The agent should ask clarifying questions or provide multiple interpretations with answers for each.
- **What happens when retrieved content contradicts itself?** The agent should present both perspectives and note the apparent contradiction.
- **How does the agent handle very long user inputs?** Input should be truncated or summarized with a note to the user about the limitation.
- **What happens if the language model API is unavailable?** The agent should display a user-friendly error message and suggest retrying.

## Requirements *(mandatory)*

### Functional Requirements

#### Core Input/Output
- **FR-001**: System MUST accept natural language questions from users via a command-line interface with a `>` prompt indicator.
- **FR-010**: System MUST allow users to exit the conversation gracefully via keywords: "quit", "exit", "bye", or Ctrl+C.

#### Retrieval Integration
- **FR-002**: System MUST retrieve relevant content chunks from the vector database before generating responses.
- **FR-009**: System MUST integrate with `retrieval.py` from Spec-1 by importing and calling `search(query, top_k=5, threshold=0.3)` function.
- **FR-011**: System MUST retrieve top 5 chunks per query with minimum similarity threshold of 0.3.
- **FR-012**: System MUST format retrieved chunks as context for the LLM prompt with source attribution.

#### Response Generation
- **FR-003**: System MUST generate responses that are grounded in the retrieved content, not general knowledge.
- **FR-004**: System MUST include citations in format: `[Source: {title}]({url})` for content referenced in responses.
- **FR-013**: System MUST instruct the model via system prompt to only answer from provided context and cite sources.

#### Context Window Management
- **FR-014**: System MUST manage context window budget of 8,192 tokens total:
  - System prompt: ~500 tokens reserved
  - Retrieved context: ~4,000 tokens max (truncate oldest chunks if exceeded)
  - Conversation history: ~2,500 tokens max (sliding window)
  - User query + response: ~1,200 tokens reserved
- **FR-015**: System MUST implement sliding window for conversation history, keeping most recent 5-10 exchanges.
- **FR-016**: System MUST truncate retrieved chunks if total context exceeds budget, prioritizing highest-scoring chunks.

#### Conversation Management
- **FR-005**: System MUST maintain conversation history within a session for context-aware responses.
- **FR-017**: System MUST store conversation history as list of `{role, content}` dictionaries in memory.
- **FR-018**: System MUST clear conversation history when user types "clear" or "reset".

#### Error Handling
- **FR-006**: System MUST gracefully handle queries outside the book's domain by acknowledging limitations.
- **FR-007**: System MUST use environment variables for all API keys and credentials (no hardcoding).
- **FR-008**: System MUST provide clear error messages when dependencies (vector DB, language model) are unavailable.
- **FR-019**: System MUST implement retry logic (3 attempts with exponential backoff) for transient API failures.
- **FR-020**: System MUST handle rate limiting gracefully with user-friendly wait messages.

#### Agent System Prompt
- **FR-021**: System MUST use a system prompt that includes:
  - Role definition: "You are a helpful assistant for the Isaac Sim robotics book"
  - Content scope: "Only answer questions based on the provided context from the book"
  - Citation requirement: "Always cite sources using [Source: title](url) format"
  - Limitation acknowledgment: "If the context doesn't contain relevant information, say so"
  - Domain boundaries: "Do not answer questions outside the scope of robotics and Isaac Sim"

### Key Entities

- **Conversation**: Represents an ongoing dialogue session with message history, context state, and session metadata.
- **Message**: Represents a single turn in the conversation with role (user/assistant), content, timestamp, and optional citations.
- **Citation**: Represents a source reference with page title, URL, relevance score, and relevant text excerpt.
- **RetrievalContext**: Represents the chunks retrieved for a query with relevance scores, source metadata, and token count.
- **AgentConfig**: Configuration for the agent including model name, base_url, temperature, max_tokens, and system prompt.
- **AgentState**: Current state of the agent including conversation history, total tokens used, and session start time.
- **ErrorResponse**: Standardized error structure with error_type, message, retry_after (for rate limits), and user_friendly_message.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive relevant, grounded responses to book-related questions in under 15 seconds (note: free-tier models may have variable latency; first request may take longer due to cold start).
- **SC-002**: 90% of responses include at least one citation when answering factual questions from book content.
- **SC-003**: Multi-turn conversations maintain coherent context for at least 5 consecutive exchanges within the sliding window.
- **SC-004**: Out-of-scope queries are handled gracefully 100% of the time (no fabricated answers from general knowledge).
- **SC-005**: Error states (API unavailable, empty database, rate limits) display user-friendly messages rather than technical stack traces.
- **SC-006**: Agent correctly uses retrieved context in 95% of responses (verified by citation presence and content grounding).

## Assumptions

- The vector database (Qdrant) is populated with book content embeddings from Spec-1.
- The retrieval module from Spec-1 (retrieval.py) is functional and returns relevant results.
- Users interact via command-line interface (CLI); web interface is out of scope (Spec-3).
- A free-tier language model API is available and sufficient for response generation.
- Conversation history is maintained in memory only; persistence across sessions is not required.
- The book content is primarily in English.

## Out of Scope

- FastAPI endpoints or REST API (deferred to Spec-3).
- Frontend UI components or web interface (deferred to Spec-3).
- User authentication or session management across restarts.
- Real-time streaming of responses (responses are returned complete).
- Multi-agent orchestration or specialized sub-agents.
- Conversation persistence beyond the current session.
- Rate limiting or usage tracking.

## Dependencies

### Spec-1 Integration (Required)
- **Qdrant Collection**: `book_embeddings` collection must be populated with book content vectors
- **retrieval.py**: Must import and use `search()` function from `fullstack/backend/retrieval.py`
- **models.py**: Reuse `RetrievalResult` dataclass from Spec-1

### API Keys (Environment Variables)
- **OPENROUTER_API_KEY**: Required for LLM access via OpenRouter
- **COHERE_API_KEY**: Required for query embedding generation (used by retrieval.py)
- **QDRANT_URL**: Qdrant Cloud cluster URL
- **QDRANT_API_KEY**: Qdrant Cloud API key

### Python Dependencies
- `openai>=1.0.0`: OpenAI SDK with base_url support for OpenRouter
- `python-dotenv`: Environment variable loading
- Spec-1 dependencies (cohere, qdrant-client) via retrieval.py import

## Testing Queries

The following queries should be used to validate agent functionality:

### Book Content Queries (Should Answer with Citations)
1. "What is URDF and how is it used in robotics?"
2. "How do forward kinematics work?"
3. "What sensors are commonly used in robotics simulation?"
4. "How do I set up a Gazebo simulation?"
5. "What is the Isaac Sim architecture?"
6. "How do AI agents plan actions in robotics?"

### Follow-up Queries (Should Use Context)
1. After asking about URDF: "Can you explain that in simpler terms?"
2. After asking about kinematics: "How does this relate to inverse kinematics?"
3. After any answer: "Tell me more about that"

### Out-of-Scope Queries (Should Decline Gracefully)
1. "What's the weather today?"
2. "Write me a poem about robots"
3. "What is the capital of France?"
4. "How do I cook pasta?"

### Edge Case Queries
1. Empty input (just pressing Enter)
2. Very long input (>500 words)
3. Non-English input
4. Special characters and code snippets
