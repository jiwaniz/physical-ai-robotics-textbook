#!/usr/bin/env python3
"""
Document ingestion script for the RAG chatbot.

This script:
1. Reads all .md files from the frontend/docs directory
2. Chunks them into smaller pieces with overlap
3. Generates embeddings using Google's embedding model
4. Uploads to Qdrant Cloud vector database

Usage:
    python scripts/ingest_docs.py [--force]

Options:
    --force     Delete existing collection and re-ingest all documents
"""

import argparse
import hashlib
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()

from src.chatbot.services import VectorStoreService

# Configuration
DOCS_DIR = Path(__file__).parent.parent.parent / "frontend" / "docs"
CHUNK_SIZE = 1000  # Characters per chunk
CHUNK_OVERLAP = 200  # Overlap between chunks


def extract_frontmatter(content: str) -> Tuple[Dict, str]:
    """Extract YAML frontmatter from markdown content."""
    frontmatter = {}
    body = content

    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            fm_content = parts[1].strip()
            body = parts[2].strip()

            # Simple YAML parsing for common fields
            for line in fm_content.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    frontmatter[key.strip()] = value.strip().strip('"').strip("'")

    return frontmatter, body


def clean_markdown(content: str) -> str:
    """Clean markdown content for better chunking."""
    # Remove code blocks (keep their content but mark them)
    content = re.sub(
        r"```(\w+)?\n(.*?)```",
        lambda m: f"\n[Code: {m.group(1) or 'text'}]\n{m.group(2)}\n[/Code]\n",
        content,
        flags=re.DOTALL,
    )

    # Remove HTML comments
    content = re.sub(r"<!--.*?-->", "", content, flags=re.DOTALL)

    # Remove import statements (MDX)
    content = re.sub(r"^import\s+.*$", "", content, flags=re.MULTILINE)

    # Clean up excessive whitespace
    content = re.sub(r"\n{3,}", "\n\n", content)

    return content.strip()


def chunk_content(
    content: str,
    chunk_size: int = CHUNK_SIZE,
    overlap: int = CHUNK_OVERLAP,
) -> List[str]:
    """
    Split content into overlapping chunks.

    Strategy:
    1. Try to split at paragraph boundaries
    2. Fall back to sentence boundaries
    3. Fall back to character boundaries
    """
    chunks = []

    # Split by paragraphs first
    paragraphs = content.split("\n\n")
    current_chunk = ""

    for para in paragraphs:
        para = para.strip()
        if not para:
            continue

        # If adding this paragraph exceeds chunk size
        if len(current_chunk) + len(para) + 2 > chunk_size:
            if current_chunk:
                chunks.append(current_chunk.strip())

                # Start new chunk with overlap
                if overlap > 0:
                    # Get last part of current chunk for overlap
                    overlap_text = current_chunk[-overlap:] if len(current_chunk) > overlap else current_chunk
                    current_chunk = overlap_text + "\n\n" + para
                else:
                    current_chunk = para
            else:
                # Single paragraph is too long, split it
                sentences = re.split(r"(?<=[.!?])\s+", para)
                for sentence in sentences:
                    if len(current_chunk) + len(sentence) + 1 > chunk_size:
                        if current_chunk:
                            chunks.append(current_chunk.strip())
                            current_chunk = sentence
                        else:
                            # Single sentence too long, split by characters
                            for i in range(0, len(sentence), chunk_size - overlap):
                                chunks.append(sentence[i : i + chunk_size])
                            current_chunk = ""
                    else:
                        current_chunk += " " + sentence if current_chunk else sentence
        else:
            current_chunk += "\n\n" + para if current_chunk else para

    # Don't forget the last chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def generate_chunk_id(page_path: str, chunk_index: int) -> str:
    """Generate a deterministic ID for a chunk."""
    unique_str = f"{page_path}:{chunk_index}"
    return hashlib.md5(unique_str.encode()).hexdigest()


def process_markdown_file(file_path: Path, base_dir: Path) -> List[Dict]:
    """Process a single markdown file and return chunks with metadata."""
    relative_path = file_path.relative_to(base_dir)
    page_path = "/" + str(relative_path).replace("\\", "/").replace(".md", "")

    # Handle index files
    if page_path.endswith("/index"):
        page_path = page_path.rsplit("/index", 1)[0] or "/"

    print(f"  Processing: {relative_path}")

    content = file_path.read_text(encoding="utf-8")
    frontmatter, body = extract_frontmatter(content)
    cleaned = clean_markdown(body)
    chunks = chunk_content(cleaned)

    title = frontmatter.get("title", frontmatter.get("sidebar_label", file_path.stem))

    documents = []
    for i, chunk in enumerate(chunks):
        if len(chunk.strip()) < 50:  # Skip very small chunks
            continue

        # Add title context to first chunk
        if i == 0 and title:
            chunk = f"# {title}\n\n{chunk}"

        doc = {
            "id": generate_chunk_id(page_path, i),
            "content": chunk,
            "page_path": page_path,
            "title": title,
            "chunk_index": i,
        }
        documents.append(doc)

    return documents


def ingest_documents(force: bool = False):
    """Main ingestion function."""
    print("=" * 60)
    print("Physical AI Textbook - Document Ingestion")
    print("=" * 60)

    # Verify environment variables
    required_vars = ["GOOGLE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing = [var for var in required_vars if not os.getenv(var)]
    if missing:
        print(f"ERROR: Missing environment variables: {', '.join(missing)}")
        print("Please set them in your .env file")
        sys.exit(1)

    # Initialize vector store
    vector_store = VectorStoreService()

    if force:
        print("\n[Force mode] Deleting existing collection...")
        vector_store.delete_collection()

    # Find all markdown files
    if not DOCS_DIR.exists():
        print(f"ERROR: Docs directory not found: {DOCS_DIR}")
        sys.exit(1)

    md_files = list(DOCS_DIR.rglob("*.md"))
    print(f"\nFound {len(md_files)} markdown files in {DOCS_DIR}")

    # Process all files
    all_documents = []
    for file_path in md_files:
        try:
            docs = process_markdown_file(file_path, DOCS_DIR)
            all_documents.extend(docs)
        except Exception as e:
            print(f"  WARNING: Failed to process {file_path}: {e}")

    print(f"\nTotal chunks: {len(all_documents)}")

    if not all_documents:
        print("No documents to ingest!")
        sys.exit(1)

    # Upload to Qdrant
    print("\nUploading to Qdrant...")
    vector_store.upsert_documents(all_documents, batch_size=50)

    print("\n" + "=" * 60)
    print("Ingestion complete!")
    print(f"  - Files processed: {len(md_files)}")
    print(f"  - Chunks created: {len(all_documents)}")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ingest documentation into Qdrant")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Delete existing collection and re-ingest all documents",
    )
    args = parser.parse_args()

    ingest_documents(force=args.force)
