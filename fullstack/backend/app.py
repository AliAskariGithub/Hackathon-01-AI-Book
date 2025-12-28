"""FastAPI backend for Isaac Sim Book Chatbot.

This module provides the REST API for the Docusaurus-embedded chat widget.
It integrates with the existing RAG agent (Spec-2) to process questions.

Entry point for Hugging Face Spaces deployment.
"""

import logging
import os
from contextlib import asynccontextmanager
from typing import Optional
from urllib.parse import urlparse
from uuid import uuid4

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from api_models import (
    ChatRequest, ChatResponse, Citation, ErrorResponse, ErrorType,
    HealthResponse
)
from validate_config import validate_config, ConfigurationError

# Load environment variables
load_dotenv()

# Configure logging
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL),
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# API Version
API_VERSION = "1.0.0"


# =============================================================================
# Lifespan Management
# =============================================================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager.

    Validates configuration on startup.
    """
    logger.info("Starting FastAPI application...")

    # Validate configuration (fail fast)
    try:
        validate_config()
        logger.info("Configuration validated successfully")
    except ConfigurationError as e:
        logger.error(f"Configuration error: {e}")
        raise

    yield

    logger.info("Shutting down FastAPI application...")


# =============================================================================
# FastAPI App Initialization
# =============================================================================
app = FastAPI(
    title="Isaac Sim Book Chatbot API",
    description="REST API for conversational Q&A over the Isaac Sim robotics book",
    version=API_VERSION,
    lifespan=lifespan
)


# =============================================================================
# CORS Configuration (FR-003)
# =============================================================================
def get_allowed_origins() -> list:
    """Get allowed CORS origins from environment or defaults.

    Per spec FR-003c:
    - Reads from ALLOWED_ORIGINS env var (comma-separated)
    - Defaults to localhost:3000 only if not set
    """
    origins_str = os.getenv("ALLOWED_ORIGINS", "")
    if origins_str:
        origins = [o.strip() for o in origins_str.split(",") if o.strip()]
        if origins:
            return origins

    # Default to localhost only (dev safety per spec)
    return ["http://localhost:3000"]


ALLOWED_ORIGINS = get_allowed_origins()
logger.info(f"CORS allowed origins: {ALLOWED_ORIGINS}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["*"],
)


# =============================================================================
# URL Transformation (FR-009c)
# =============================================================================
def get_book_base_url() -> str:
    """Get book base URL from environment.

    Returns:
        Base URL for citation transformation
    """
    return os.getenv("BOOK_BASE_URL", "https://aliaskarigithub.github.io")


def transform_citation_url(url: str, base_url: Optional[str] = None) -> str:
    """Transform absolute URL to relative Docusaurus route.

    Per spec FR-009c and research R4:
    - Uses URL parsing (not regex)
    - Transforms: https://user.github.io/repo/docs/page → /docs/page

    Args:
        url: Original URL from Qdrant
        base_url: Base URL to strip (uses BOOK_BASE_URL if not provided)

    Returns:
        Relative Docusaurus route
    """
    if base_url is None:
        base_url = get_book_base_url()

    # If URL starts with base URL, strip it
    if url.startswith(base_url):
        relative = url[len(base_url):]
        # Ensure it starts with /
        if not relative.startswith("/"):
            relative = "/" + relative
        return relative

    # Fallback: extract path from URL
    parsed = urlparse(url)
    return parsed.path or "/"


# =============================================================================
# Error Handling
# =============================================================================
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for unhandled errors."""
    logger.exception(f"Unhandled exception: {exc}")
    return JSONResponse(
        status_code=500,
        content=ErrorResponse(
            error_type=ErrorType.BACKEND,
            message="Something went wrong. Please try again.",
            details=str(exc) if LOG_LEVEL == "DEBUG" else None
        ).model_dump()
    )


# =============================================================================
# Health Endpoint (FR-024)
# =============================================================================
@app.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check():
    """Health check endpoint.

    Returns service health status and dependency availability.
    """
    dependencies = {
        "cohere": bool(os.getenv("COHERE_API_KEY")),
        "qdrant": bool(os.getenv("QDRANT_URL")),
        "openrouter": bool(os.getenv("OPENROUTER_API_KEY")),
    }

    all_healthy = all(dependencies.values())

    return HealthResponse(
        status="healthy" if all_healthy else "degraded",
        version=API_VERSION,
        dependencies=dependencies
    )


# =============================================================================
# Chat Endpoint (FR-001, FR-002)
# =============================================================================
@app.post("/api/chat", response_model=ChatResponse, tags=["Chat"])
async def chat(request: ChatRequest):
    """Process a chat request through the RAG pipeline.

    Integrates with Spec-2 agent to:
    1. Retrieve relevant context from Qdrant
    2. Generate LLM response with citations
    3. Transform citation URLs for Docusaurus

    Args:
        request: ChatRequest with query and optional history

    Returns:
        ChatResponse with answer and citations
    """
    logger.info(f"Processing chat request: {request.query[:50]}...")

    # Generate conversation ID if not provided
    conversation_id = request.conversation_id or str(uuid4())

    try:
        # Import agent here to avoid startup issues if not configured
        from agent import create_agent, process_query, AgentError
        from models import Message, Conversation

        # Create fresh agent state (stateless backend per spec)
        agent = create_agent()

        # Reconstruct conversation history from request
        for msg in request.conversation_history:
            agent.conversation.add_message(
                Message(role=msg.role, content=msg.content)
            )

        # Process query through RAG pipeline
        response_text, raw_citations = await process_query(agent, request.query)

        # Transform citation URLs
        citations = [
            Citation(
                title=c.title,
                url=transform_citation_url(c.url),
                score=c.score
            )
            for c in raw_citations
        ]

        logger.info(f"Generated response with {len(citations)} citations")

        return ChatResponse(
            answer=response_text,
            citations=citations,
            conversation_id=conversation_id,
            error=None
        )

    except ImportError as e:
        logger.error(f"Failed to import agent modules: {e}")
        return ChatResponse(
            answer=None,
            citations=[],
            conversation_id=conversation_id,
            error="backend"
        )

    except Exception as e:
        logger.exception(f"Error processing chat request: {e}")

        # Determine error type
        error_type = "backend"
        error_str = str(e).lower()
        if "rate" in error_str or "429" in error_str:
            error_type = "rate_limit"
        elif "timeout" in error_str:
            error_type = "timeout"
        elif "connection" in error_str or "network" in error_str:
            error_type = "network"

        return ChatResponse(
            answer=None,
            citations=[],
            conversation_id=conversation_id,
            error=error_type
        )


# =============================================================================
# OpenAPI Schema Export (T010b)
# =============================================================================
def export_openapi_schema(output_path: str = None):
    """Export OpenAPI schema to YAML file.

    Args:
        output_path: Path to save schema (defaults to contracts/chat-api.yaml)
    """
    import json
    import yaml

    if output_path is None:
        output_path = "../../specs/003-fastapi-chatbot/contracts/chat-api.yaml"

    schema = app.openapi()

    with open(output_path, "w") as f:
        yaml.dump(schema, f, default_flow_style=False, sort_keys=False)

    logger.info(f"OpenAPI schema exported to {output_path}")


# =============================================================================
# Main Entry Point
# =============================================================================
if __name__ == "__main__":
    import uvicorn

    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")

    logger.info(f"Starting server on {host}:{port}")
    uvicorn.run(app, host=host, port=port)
