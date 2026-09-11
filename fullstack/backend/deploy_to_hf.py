"""Helper script to deploy the Backend to Hugging Face Spaces.

Supports two methods:
1. Automated upload using huggingface_hub (if HF_TOKEN is provided)
2. Interactive step-by-step Git instructions
"""

import os
import sys
from pathlib import Path
from dotenv import load_dotenv

load_dotenv()

FILES_TO_DEPLOY = [
    "Dockerfile",
    ".dockerignore",
    "README.md",
    "requirements.txt",
    "app.py",
    "api_models.py",
    "agent.py",
    "models.py",
    "retrieval.py",
    "validate_config.py",
]

REQUIRED_SECRETS = [
    ("COHERE_API_KEY", "Cohere API key for semantic embeddings"),
    ("QDRANT_URL", "Qdrant cluster URL (including port :6333)"),
    ("QDRANT_API_KEY", "Qdrant API key"),
    ("GROQ_API_KEY", "Groq API key for LLM inference"),
    ("GROQ_MODEL", "LLM model on Groq (default: openai/gpt-oss-120b)"),
    ("BOOK_BASE_URL", "Book URL for citation transformation (e.g. https://ai-powered-book.vercel.app)"),
    ("ALLOWED_ORIGINS", "Comma-separated CORS origins (e.g. http://localhost:3000,https://ai-powered-book.vercel.app)"),
]


def print_header(title: str):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60 + "\n")


def check_files():
    backend_dir = Path(__file__).parent
    missing = []
    for f in FILES_TO_DEPLOY:
        p = backend_dir / f
        if not p.exists():
            missing.append(f)
    return missing


def print_secrets_summary():
    print("Required Repository Secrets to configure in Hugging Face Space Settings:")
    print("-" * 60)
    for key, desc in REQUIRED_SECRETS:
        val = os.getenv(key)
        status = "[SET in local .env]" if val else "[NOT SET locally]"
        print(f"  * {key:18} : {desc}")
        print(f"    Status: {status}")
    print("-" * 60)


def deploy_via_hub(space_id: str, hf_token: str):
    try:
        from huggingface_hub import HfApi
        api = HfApi(token=hf_token)
        backend_dir = Path(__file__).parent

        print(f"Uploading files to Space: {space_id}...")
        for filename in FILES_TO_DEPLOY:
            file_path = backend_dir / filename
            if file_path.exists():
                print(f"  -> Uploading {filename}...")
                api.upload_file(
                    path_or_fileobj=str(file_path),
                    path_in_repo=filename,
                    repo_id=space_id,
                    repo_type="space",
                )
        print("\n[SUCCESS] All files uploaded successfully to Hugging Face Space!")
        print(f"Space URL: https://huggingface.co/spaces/{space_id}")
        return True
    except Exception as e:
        print(f"[ERROR] Upload via huggingface_hub failed: {e}")
        return False


def main():
    print_header("Hugging Face Spaces Deployment Assistant")

    missing = check_files()
    if missing:
        print(f"[ERROR] Missing deployment files: {', '.join(missing)}")
        sys.exit(1)

    print("[OK] All required deployment files are present:")
    for f in FILES_TO_DEPLOY:
        print(f"  - {f}")
    print()

    print_secrets_summary()

    hf_token = os.getenv("HF_TOKEN")
    space_id = os.getenv("HF_SPACE_ID")

    if hf_token and space_id:
        print(f"\nFound HF_TOKEN and HF_SPACE_ID ({space_id}) in environment.")
        confirm = input(f"Do you want to deploy directly to {space_id}? (y/n): ").strip().lower()
        if confirm == 'y':
            deploy_via_hub(space_id, hf_token)
            return

    print_header("Manual Deployment Instructions (via Git)")
    print("1. Create a new Space on Hugging Face:")
    print("   - Go to: https://huggingface.co/new-space")
    print("   - Space Name: backend-chatbot-book (or your chosen name)")
    print("   - Space SDK: Docker (Blank)")
    print("   - License: MIT / Apache-2.0")
    print("   - Visibility: Public")
    print()
    print("2. Clone your Space repository locally:")
    print("   git clone https://huggingface.co/spaces/<YOUR_USERNAME>/<SPACE_NAME>")
    print()
    print("3. Copy the backend files into the cloned folder:")
    backend_dir = Path(__file__).parent.resolve()
    print(f"   Copy from: {backend_dir}")
    print("   Files to copy:")
    for f in FILES_TO_DEPLOY:
        print(f"     - {f}")
    print()
    print("4. Push to Hugging Face:")
    print("   cd <SPACE_NAME>")
    print("   git add .")
    print('   git commit -m "Deploy FastAPI RAG backend with Groq"')
    print("   git push")
    print()
    print("5. Configure Secrets in Space Settings:")
    print("   Go to: https://huggingface.co/spaces/<YOUR_USERNAME>/<SPACE_NAME>/settings")
    print("   Under 'Variables and secrets' -> 'New secret', add:")
    print("     - COHERE_API_KEY")
    print("     - QDRANT_URL")
    print("     - QDRANT_API_KEY")
    print("     - GROQ_API_KEY")
    print("     - GROQ_MODEL (Value: openai/gpt-oss-120b)")
    print("     - BOOK_BASE_URL (Value: https://ai-powered-book.vercel.app)")
    print("     - ALLOWED_ORIGINS (Value: http://localhost:3000,http://localhost:8000,https://ai-powered-book.vercel.app)")
    print()
    print("6. Test your live endpoint:")
    print("   curl https://<YOUR_USERNAME>-<SPACE_NAME>.hf.space/health")


if __name__ == "__main__":
    main()
