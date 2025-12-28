"""Startup configuration validator for FastAPI backend.

Validates all required environment variables are present.
Fails fast with clear error messages per spec FR-026.
"""

import os
import sys
from typing import List, Optional


# Required environment variables (must be set)
REQUIRED_VARS = [
    "COHERE_API_KEY",
    "QDRANT_URL",
    "QDRANT_API_KEY",
    "OPENROUTER_API_KEY",
    "BOOK_BASE_URL",
]

# Optional environment variables (have defaults)
OPTIONAL_VARS = {
    "ALLOWED_ORIGINS": "http://localhost:3000",  # default to localhost only
    "LOG_LEVEL": "INFO",
    "COLLECTION_NAME": "book_embeddings",
}


class ConfigurationError(Exception):
    """Raised when required configuration is missing."""
    pass


def validate_required_vars() -> List[str]:
    """Check that all required environment variables are set.

    Returns:
        List of missing variable names (empty if all present)
    """
    missing = []
    for var in REQUIRED_VARS:
        if not os.getenv(var):
            missing.append(var)
    return missing


def get_config_value(name: str, default: Optional[str] = None) -> Optional[str]:
    """Get configuration value from environment.

    Args:
        name: Environment variable name
        default: Default value if not set

    Returns:
        Configuration value or default
    """
    return os.getenv(name, default)


def validate_config() -> None:
    """Validate all required configuration on startup.

    Raises:
        ConfigurationError: If any required variables are missing
    """
    missing = validate_required_vars()

    if missing:
        error_msg = (
            f"Missing required environment variables: {', '.join(missing)}\n"
            f"Please set these variables in your .env file or environment.\n"
            f"See .env.example for reference."
        )
        raise ConfigurationError(error_msg)


def print_config_status() -> None:
    """Print configuration status for debugging."""
    print("Configuration Status:")
    print("-" * 40)

    # Required vars
    print("\nRequired Variables:")
    for var in REQUIRED_VARS:
        value = os.getenv(var)
        status = "✓ SET" if value else "✗ MISSING"
        # Mask sensitive values
        display = f"{value[:4]}..." if value and len(value) > 4 else value
        print(f"  {var}: {status} ({display})")

    # Optional vars
    print("\nOptional Variables (with defaults):")
    for var, default in OPTIONAL_VARS.items():
        value = os.getenv(var, default)
        print(f"  {var}: {value}")


if __name__ == "__main__":
    """CLI for checking configuration."""
    from dotenv import load_dotenv
    load_dotenv()

    print_config_status()
    print()

    try:
        validate_config()
        print("✓ All required configuration is valid!")
        sys.exit(0)
    except ConfigurationError as e:
        print(f"✗ Configuration Error:\n{e}")
        sys.exit(1)
