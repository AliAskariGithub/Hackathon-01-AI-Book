---
id: "002"
title: "Create RAG agent implementation plan"
stage: "plan"
date: "2025-12-28"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "rag-agent"
branch: "2-rag-agent"
user: "Claude Code User"
command: "/sp.plan OpenAI Agent with RAG retrieval integration"
labels: ["rag", "agent", "openai", "planning", "openrouter"]
links:
  spec: "specs/2-rag-agent/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/2-rag-agent/plan.md"
  - "specs/2-rag-agent/research.md"
  - "specs/2-rag-agent/data-model.md"
  - "specs/2-rag-agent/quickstart.md"
  - "specs/2-rag-agent/contracts/agent-api.md"
tests: []
---

## Prompt

OpenAI Agent with RAG retrieval integration

Phase 1: Environment Setup
- Install dependencies: openai (or openrouter SDK), python-dotenv
- Add OPENROUTER_API_KEY, OPENROUTER_MODEL to .env
- Verify OpenRouter API connectivity with test call
- Import retrieval.search from Spec-1

Phase 2: Agent Configuration (agent.py)
- Define AgentConfig dataclass (model, temperature, max_tokens, system_prompt)
- Create system prompt defining agent scope and behavior
- Initialize OpenAI client with OpenRouter base_url override
- Implement token counting for context window management

Phase 3: Retrieval Integration (agent.py)
- Implement retrieve_context(query) calling retrieval.search with top_k=5, threshold=0.7
- Format retrieved chunks into agent context with citations
- Handle empty retrieval results gracefully

Phase 4: Conversation Management (agent.py)
- Implement Conversation class with message history and token tracking
- Add message formatting with role (user/assistant) and content
- Implement context truncation when approaching model limits
- Add conversation reset/clear command

Phase 5: Response Generation (agent.py)
- Implement generate_response(user_query, conversation) orchestrating retrieval + LLM call
- Inject retrieved context into messages before API call
- Parse LLM response and extract citations
- Handle API errors with retry logic (3 attempts, exponential backoff)

Phase 6: CLI Interface (agent.py)
- Implement interactive loop with user input prompts
- Handle exit commands (quit, exit, bye, q)
- Display responses with formatted citations
- Show token usage warnings when approaching limits

Phase 7: Testing & Validation
- Test 5+ sample queries covering book topics
- Verify citation inclusion in responses
- Test multi-turn context retention (5+ exchanges)
- Validate out-of-scope query handling

Validation checkpoints:
- Verify OpenRouter API integration works
- Confirm retrieval returns relevant chunks
- Test context window overflow handling
- Validate citation format consistency

## Response snapshot

Created comprehensive implementation plan for RAG Agent with:

1. **Research Phase** (`research.md`):
   - 8 research questions resolved
   - OpenAI Agents SDK + OpenRouter integration approach documented
   - Model selection: `meta-llama/llama-3.2-3b-instruct:free`
   - Token counting via tiktoken with cl100k_base encoding
   - Context window budget: 8,192 tokens allocated
   - Retry strategy: 3 attempts with exponential backoff

2. **Data Model** (`data-model.md`):
   - 8 entities defined: AgentConfig, AgentState, Conversation, Message, Citation, RetrievalContext, RetrievalResult, ErrorResponse
   - Token budget allocation documented
   - Python dataclass definitions provided

3. **API Contracts** (`contracts/agent-api.md`):
   - Public functions: create_agent(), process_query(), clear_conversation(), run_cli()
   - Internal functions: retrieve_context(), format_context_for_prompt(), generate_response(), etc.
   - Error handling contract with custom exceptions
   - CLI commands contract

4. **Quickstart** (`quickstart.md`):
   - Step-by-step setup guide
   - Dependency installation
   - Environment configuration
   - Verification steps

5. **Implementation Plan** (`plan.md`):
   - 13 implementation steps
   - Acceptance criteria for each step
   - Validation checkpoints
   - Risk assessment

## Outcome

- ✅ Impact: Complete planning artifacts generated for RAG agent implementation
- 🧪 Tests: Test strategy defined in Step 12, test queries from spec included
- 📁 Files: 5 artifacts created in specs/2-rag-agent/
- 🔁 Next prompts: `/sp.tasks` to generate task breakdown, then `/sp.implement`
- 🧠 Reflection: Plan aligns with user's 7-phase structure, added detail for implementation clarity

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): N/A - planning phase
- Prompt variant (if applicable): User-provided phase structure incorporated
- Next experiment (smallest change to try): Proceed to tasks generation
