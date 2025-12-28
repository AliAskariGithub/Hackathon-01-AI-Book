"""RAG pipeline for semantic retrieval and validation."""

import argparse
import logging
import os
from datetime import datetime
from typing import List

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient

from models import QueryResult, RetrievalResult, ValidationReport

# Load environment variables
load_dotenv()

# Configure logging
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL),
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Configuration
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "book_embeddings")
COHERE_MODEL = "embed-english-v3.0"

# Validation test queries (from plan.md)
TEST_QUERIES = [
    "What is URDF and how is it used?",
    "How do forward kinematics work?",
    "What sensors are used in robotics?",
    "How to set up Gazebo simulation?",
    "What is Isaac Sim architecture?",
    "How do AI agents plan actions?",
]


def search(
    query: str,
    top_k: int = 5,
    threshold: float = 0.0,
    collection_name: str = COLLECTION_NAME
) -> List[RetrievalResult]:
    """Perform semantic search for a query (T022)."""
    logger.debug(f"Searching for: {query}")

    # Initialize clients
    co = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Generate query embedding
    response = co.embed(
        texts=[query],
        model=COHERE_MODEL,
        input_type="search_query"
    )
    query_vector = response.embeddings[0]

    # Search Qdrant
    results = qdrant.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=top_k,
        score_threshold=threshold if threshold > 0 else None
    )

    # Convert to RetrievalResult
    retrieval_results = []
    for hit in results.points:
        payload = hit.payload or {}
        retrieval_results.append(RetrievalResult(
            chunk_id=str(hit.id),
            text=payload.get("text", ""),
            score=hit.score,
            url=payload.get("url", ""),
            title=payload.get("title", ""),
            chunk_index=payload.get("chunk_index", 0)
        ))

    logger.debug(f"Found {len(retrieval_results)} results")
    return retrieval_results


def format_results(results: List[RetrievalResult], max_text_length: int = 200) -> str:
    """Format results for display (T023)."""
    if not results:
        return "No results found."

    output = []
    for i, r in enumerate(results, 1):
        text_preview = r.text[:max_text_length] + "..." if len(r.text) > max_text_length else r.text
        output.append(
            f"{i}. [{r.score:.3f}] {r.title}\n"
            f"   {text_preview}\n"
            f"   URL: {r.url}"
        )

    return "\n\n".join(output)


def validate(
    test_queries: List[str] = None,
    threshold: float = 0.7,
    top_k: int = 3,
    collection_name: str = COLLECTION_NAME
) -> ValidationReport:
    """Run validation against test queries (T025)."""
    if test_queries is None:
        test_queries = TEST_QUERIES

    logger.info(f"Running validation with {len(test_queries)} queries, threshold={threshold}")

    query_results = []
    total_similarity = 0.0

    for query in test_queries:
        results = search(query, top_k=top_k, collection_name=collection_name)

        top_score = results[0].score if results else 0.0
        passed = top_score >= threshold
        total_similarity += top_score

        query_results.append(QueryResult(
            query=query,
            top_score=top_score,
            passed=passed,
            top_results=results
        ))

        status = "PASS" if passed else "FAIL"
        logger.info(f"  [{status}] {query[:50]}... (score: {top_score:.3f})")

    passed_queries = sum(1 for r in query_results if r.passed)
    failed_queries = len(query_results) - passed_queries
    pass_rate = passed_queries / len(query_results) if query_results else 0.0
    avg_similarity = total_similarity / len(query_results) if query_results else 0.0

    report = ValidationReport(
        timestamp=datetime.now(),
        total_queries=len(query_results),
        passed_queries=passed_queries,
        failed_queries=failed_queries,
        pass_rate=pass_rate,
        threshold=threshold,
        avg_similarity=avg_similarity,
        results=query_results
    )

    return report


def format_validation_report(report: ValidationReport) -> str:
    """Format validation report for display."""
    lines = [
        "=" * 50,
        "Validation Report",
        "=" * 50,
        f"Timestamp: {report.timestamp.isoformat()}",
        f"Threshold: {report.threshold:.2f}",
        f"Total queries: {report.total_queries}",
        f"Passed: {report.passed_queries}",
        f"Failed: {report.failed_queries}",
        f"Pass rate: {report.pass_rate * 100:.1f}%",
        f"Average similarity: {report.avg_similarity:.3f}",
        "",
        "Results:",
        "-" * 50,
    ]

    for r in report.results:
        status = "PASS" if r.passed else "FAIL"
        lines.append(f"[{status}] {r.query}")
        lines.append(f"       Score: {r.top_score:.3f}")
        if r.top_results:
            lines.append(f"       Top result: {r.top_results[0].title}")

    lines.append("-" * 50)

    if report.pass_rate == 1.0:
        lines.append("All validation checks passed!")
    else:
        lines.append(f"WARNING: {report.failed_queries} queries failed validation.")

    return "\n".join(lines)


def main():
    """CLI interface (T024, T026)."""
    parser = argparse.ArgumentParser(description="RAG semantic retrieval")
    parser.add_argument("--query", help="Search query")
    parser.add_argument("--top-k", type=int, default=5, help="Number of results")
    parser.add_argument("--threshold", type=float, default=0.0, help="Minimum similarity score")
    parser.add_argument("--collection", default=COLLECTION_NAME, help="Qdrant collection name")
    parser.add_argument("--validate", action="store_true", help="Run validation suite")

    args = parser.parse_args()

    if args.validate:
        # Run validation (T026)
        report = validate(
            threshold=args.threshold if args.threshold > 0 else 0.7,
            collection_name=args.collection
        )
        print(format_validation_report(report))
        return 0 if report.pass_rate == 1.0 else 1

    elif args.query:
        # Run search (T024)
        results = search(
            query=args.query,
            top_k=args.top_k,
            threshold=args.threshold,
            collection_name=args.collection
        )

        print(f"\nQuery: {args.query}\n")
        print(format_results(results))
        return 0

    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    exit(main())
