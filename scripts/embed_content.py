#!/usr/bin/env python3
"""
Content embedding script for Physical AI & Humanoid Robotics Textbook.

Reads markdown files from frontend/docs, generates embeddings using Google AI (Gemini),
and stores them in Qdrant vector database.
"""

import hashlib
import os
import re
import sys
from pathlib import Path
from typing import Generator

from dotenv import load_dotenv
from google import genai
from google.genai import types
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, PointStruct, VectorParams

# Load environment variables
load_dotenv()

# Configuration
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION", "book_content")
FORCE_REGENERATE = os.getenv("FORCE_REGENERATE", "false").lower() == "true"

# Google AI Embedding model
EMBEDDING_MODEL = "text-embedding-004"
VECTOR_SIZE = 768  # Google text-embedding-004 dimension

# Chunk settings
CHUNK_SIZE = 1000  # characters
CHUNK_OVERLAP = 200  # characters


def validate_config() -> bool:
    """Validate required environment variables are set."""
    missing = []
    if not GOOGLE_API_KEY:
        missing.append("GOOGLE_API_KEY")
    if not QDRANT_URL:
        missing.append("QDRANT_URL")
    if not QDRANT_API_KEY:
        missing.append("QDRANT_API_KEY")

    if missing:
        print(f"Error: Missing required environment variables: {', '.join(missing)}")
        return False
    return True


def get_markdown_files(docs_path: Path) -> list[Path]:
    """Find all markdown files in the docs directory."""
    return list(docs_path.glob("**/*.md"))


def extract_frontmatter(content: str) -> tuple[dict, str]:
    """Extract YAML frontmatter and return metadata dict and remaining content."""
    frontmatter = {}
    body = content

    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            yaml_content = parts[1].strip()
            body = parts[2].strip()

            # Parse simple YAML key: value pairs
            for line in yaml_content.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    frontmatter[key.strip()] = value.strip().strip("\"'")

    return frontmatter, body


def clean_markdown(text: str) -> str:
    """Clean markdown content for embedding."""
    # Remove code blocks
    text = re.sub(r"```[\s\S]*?```", "", text)
    # Remove inline code
    text = re.sub(r"`[^`]+`", "", text)
    # Remove images
    text = re.sub(r"!\[.*?\]\(.*?\)", "", text)
    # Remove links but keep text
    text = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", text)
    # Remove HTML tags
    text = re.sub(r"<[^>]+>", "", text)
    # Normalize whitespace
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def chunk_text(
    text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP
) -> Generator[str, None, None]:
    """Split text into overlapping chunks."""
    if len(text) <= chunk_size:
        yield text
        return

    start = 0
    while start < len(text):
        end = start + chunk_size

        # Try to break at sentence boundary
        if end < len(text):
            # Look for sentence end in the last 20% of chunk
            search_start = end - int(chunk_size * 0.2)
            sentence_end = text.rfind(". ", search_start, end)
            if sentence_end > search_start:
                end = sentence_end + 1

        chunk = text[start:end].strip()
        if chunk:
            yield chunk

        start = end - overlap


def generate_point_id(file_path: str, chunk_index: int) -> str:
    """Generate a deterministic point ID from file path and chunk index."""
    content = f"{file_path}:{chunk_index}"
    return hashlib.md5(content.encode()).hexdigest()


def process_file(file_path: Path, docs_root: Path) -> list[dict]:
    """Process a single markdown file and return chunk data."""
    relative_path = file_path.relative_to(docs_root)

    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    frontmatter, body = extract_frontmatter(content)
    clean_text = clean_markdown(body)

    if not clean_text or len(clean_text) < 50:
        return []

    chunks = []
    for i, chunk in enumerate(chunk_text(clean_text)):
        # Extract section/module from path
        parts = list(relative_path.parts)
        section = parts[0] if parts else "general"

        chunks.append(
            {
                "id": generate_point_id(str(relative_path), i),
                "text": chunk,
                "metadata": {
                    "file_path": str(relative_path),
                    "section": section,
                    "title": frontmatter.get("title", file_path.stem),
                    "chunk_index": i,
                    "sidebar_position": frontmatter.get("sidebar_position", "0"),
                },
            }
        )

    return chunks


def generate_embeddings(client: genai.Client, texts: list[str]) -> list[list[float]]:
    """Generate embeddings using Google AI."""
    embeddings = []
    for text in texts:
        result = client.models.embed_content(
            model=EMBEDDING_MODEL,
            contents=text,
            config=types.EmbedContentConfig(task_type="RETRIEVAL_DOCUMENT"),
        )
        embeddings.append(result.embeddings[0].values)
    return embeddings


def main():
    """Main embedding pipeline."""
    print("Starting content embedding generation with Google AI...")

    # Validate configuration
    if not validate_config():
        sys.exit(1)

    # Initialize Google AI client
    google_client = genai.Client(api_key=GOOGLE_API_KEY)

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=60.0,
    )

    # Determine docs path (works from project root or scripts folder)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    docs_path = project_root / "frontend" / "docs"

    if not docs_path.exists():
        print(f"Error: Docs directory not found at {docs_path}")
        sys.exit(1)

    # Ensure collection exists
    collections = qdrant_client.get_collections()
    collection_exists = any(
        c.name == QDRANT_COLLECTION for c in collections.collections
    )

    if FORCE_REGENERATE and collection_exists:
        print(f"Force regenerate: Deleting existing collection '{QDRANT_COLLECTION}'")
        qdrant_client.delete_collection(collection_name=QDRANT_COLLECTION)
        collection_exists = False

    if not collection_exists:
        print(f"Creating collection '{QDRANT_COLLECTION}'")
        qdrant_client.create_collection(
            collection_name=QDRANT_COLLECTION,
            vectors_config=VectorParams(
                size=VECTOR_SIZE,
                distance=Distance.COSINE,
            ),
        )

    # Process markdown files
    md_files = get_markdown_files(docs_path)
    print(f"Found {len(md_files)} markdown files")

    all_chunks = []
    for file_path in md_files:
        chunks = process_file(file_path, docs_path)
        all_chunks.extend(chunks)
        if chunks:
            print(
                f"  Processed: {file_path.relative_to(docs_path)} ({len(chunks)} chunks)"
            )

    print(f"\nTotal chunks to embed: {len(all_chunks)}")

    if not all_chunks:
        print("No content to embed")
        sys.exit(0)

    # Generate embeddings in batches
    batch_size = 20  # Smaller batches for rate limits
    points = []

    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i : i + batch_size]
        texts = [chunk["text"] for chunk in batch]

        print(
            f"Generating embeddings for batch {i // batch_size + 1}/{(len(all_chunks) + batch_size - 1) // batch_size}..."
        )

        embeddings = generate_embeddings(google_client, texts)

        for j, embedding in enumerate(embeddings):
            chunk = batch[j]
            points.append(
                PointStruct(
                    id=chunk["id"],
                    vector=embedding,
                    payload=chunk["metadata"],
                )
            )

    # Upsert to Qdrant
    print(f"\nUpserting {len(points)} points to Qdrant...")

    # Upsert in batches
    upsert_batch_size = 100
    for i in range(0, len(points), upsert_batch_size):
        batch = points[i : i + upsert_batch_size]
        qdrant_client.upsert(
            collection_name=QDRANT_COLLECTION,
            points=batch,
        )

    # Get collection info
    print(f"\nEmbedding generation complete!")
    print(f"Collection: {QDRANT_COLLECTION}")
    print(f"Total points uploaded: {len(points)}")

    try:
        info = qdrant_client.get_collection(collection_name=QDRANT_COLLECTION)
        print(f"Verified vectors in collection: {info.vectors_count}")
    except Exception as e:
        print(
            f"Note: Could not verify collection info (client/server version mismatch): {type(e).__name__}"
        )


if __name__ == "__main__":
    main()
