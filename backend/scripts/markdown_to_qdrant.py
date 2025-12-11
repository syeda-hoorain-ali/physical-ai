#!/usr/bin/env python3
"""
Script to convert markdown files to vectors and upload them to Qdrant.
Uses qdrant-client[fastembed] for efficient embedding generation.
"""

import os
import logging
from pathlib import Path
import hashlib
from typing import List, Optional
import argparse
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from fastembed import TextEmbedding
import markdown
from bs4 import BeautifulSoup
from dotenv import find_dotenv, load_dotenv

# Configure environment variables
load_dotenv(find_dotenv())

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

MODEL_NAME = "BAAI/bge-small-en-v1.5"  # Using a lightweight model


def extract_text_from_markdown(file_path: str) -> str:
    """
    Extract text content from a markdown file, removing headers and formatting.
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        md_content = file.read()

    # Convert markdown to HTML
    html_content = markdown.markdown(md_content, output_format="html")

    # Parse HTML and extract plain text
    soup = BeautifulSoup(html_content, 'html.parser')
    text = soup.get_text(separator=' ', strip=True)

    return text


def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """
    Split text into overlapping chunks to preserve context.
    """
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # If this is not the last chunk, try to break at sentence boundary
        if end < len(text):
            # Look for sentence endings near the end
            temp_end = end
            while temp_end < min(len(text), end + 200) and text[temp_end] not in '.!?':
                temp_end += 1

            if temp_end != end:
                end = temp_end + 1  # Include the punctuation mark

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        # Move start position forward, accounting for overlap
        start = end - overlap

        # If we've reached the end or overlap causes no progress, break to avoid infinite loop
        if start >= len(text):
            break

    # Clean up any chunks that are too short
    chunks = [chunk for chunk in chunks if len(chunk) > 50]

    return chunks


def initialize_qdrant_client(host: str = "localhost", port: int = 6333, api_key: Optional[str] = None):
    """
    Initialize and return Qdrant client.
    """
    if api_key:
        client = QdrantClient(host=host, port=port, api_key=api_key)
    else:
        client = QdrantClient(host=host, port=port, api_key=os.getenv("QDRANT_API_KEY"))

    logger.info(f"Connected to Qdrant at {host}:{port}")
    return client


def create_collection(client: QdrantClient, collection_name: str):
    """
    Create a Qdrant collection if it doesn't exist.
    """
    try:
        # Check if collection exists
        client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' already exists")
    except Exception:
        # Create new collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=client.get_embedding_size(MODEL_NAME),
                distance=Distance.COSINE
            )
        )
        logger.info(f"Created new collection '{collection_name}'")


def delete_collection(client: QdrantClient, collection_name: str):
    """
    Delete Qdrant collection if it exists.
    """
    try:
        client.delete_collection(collection_name)
        logger.info(f"Deleted collection '{collection_name}'")
    except Exception:
        logger.info(f"Collection '{collection_name}' does not exists")


def process_markdown_files(
    source_dir: str,
    collection_name: str,
    client: QdrantClient,
    embedding_model: TextEmbedding,
    chunk_size: int = 1000,
    overlap: int = 100,
    dry_run: bool = False
):
    """
    Process all markdown files in the source directory and upload to Qdrant.
    """
    source_path = Path(source_dir)
    markdown_files = list(source_path.rglob("*.md"))

    logger.info(f"Found {len(markdown_files)} markdown files to process")

    points = []
    point_id = 0

    for file_path in markdown_files:
        logger.info(f"Processing file: {file_path}")

        try:
            # Extract text from markdown
            text = extract_text_from_markdown(str(file_path))

            # Skip empty files
            if not text.strip():
                logger.warning(f"Skipping empty file: {file_path}")
                continue

            # Chunk the text
            chunks = chunk_text(text, chunk_size, overlap)

            # Generate embedding for the chunks
            if chunks:
                embeddings = list(embedding_model.embed(chunks))
            else:
                embeddings = []

            # Process each chunk
            for i, chunk in enumerate(embeddings):
                embedding = embeddings[i]
                # Create a unique ID for this chunk
                chunk_id = hashlib.md5(f"{file_path}_{i}".encode()).hexdigest()

                # Create payload with metadata
                payload = {
                    "source_file": str(file_path),
                    "chunk_index": i,
                    "content": chunk,
                    "original_file_path": str(file_path.relative_to(source_path)),
                    "file_name": file_path.name
                }


                # Add point to collection
                point_struct = models.PointStruct(
                    id=chunk_id,
                    vector=list(embedding),
                    payload=payload
                )

                if not dry_run:
                    points.append(point_struct)
                point_id += 1

                # Batch upload every 100 points to avoid memory issues (only if not dry run)
                if not dry_run and len(points) >= 100:
                    client.upsert(
                        collection_name=collection_name,
                        points=points
                    )
                    logger.info(f"Uploaded batch of {len(points)} points to Qdrant")
                    points = []  # Reset the batch

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {str(e)}")
            continue

    # Upload any remaining points (only if not dry run)
    if not dry_run and points:
        client.upsert(
            collection_name=collection_name,
            points=points
        )
        logger.info(f"Uploaded final batch of {len(points)} points to Qdrant")
    elif dry_run:
        logger.info(f"Dry run: Would have uploaded {point_id} chunks from {len(markdown_files)} files to Qdrant collection '{collection_name}'")
    else:
        logger.info(f"Uploaded {point_id} chunks from {len(markdown_files)} files to Qdrant collection '{collection_name}'")


def main():
    parser = argparse.ArgumentParser(description="Convert markdown files to vectors and upload to Qdrant")
    parser.add_argument("--source-dir", required=True, help="Directory containing markdown files")
    parser.add_argument("--collection-name", required=True, help="Qdrant collection name")
    parser.add_argument("--qdrant-host", default="localhost", help="Qdrant host (default: localhost)")
    parser.add_argument("--qdrant-port", type=int, default=6333, help="Qdrant port (default: 6333)")
    parser.add_argument("--qdrant-api-key", help="Qdrant API key (optional)")
    parser.add_argument("--chunk-size", type=int, default=1000, help="Text chunk size (default: 1000)")
    parser.add_argument("--overlap", type=int, default=100, help="Chunk overlap size (default: 100)")
    parser.add_argument("--recreate-collection", action="store_true", help="Process files without uploading to Qdrant (for testing)")
    parser.add_argument("--dry-run", action="store_true", help="Process files without uploading to Qdrant (for testing)")

    args = parser.parse_args()

    logger.info("Starting markdown to Qdrant conversion process...")

    # Initialize components
    embedding_model = TextEmbedding(model_name=MODEL_NAME)  # Using a lightweight model

    if not args.dry_run:
        client = initialize_qdrant_client(args.qdrant_host, args.qdrant_port, args.qdrant_api_key)

        if args.recreate_collection:
            # Delete and recreate collection
            delete_collection(client, args.collection_name)
        create_collection(client, args.collection_name)
    else:
        logger.info("Dry run mode: skipping Qdrant initialization and upload")
        client = None

    # Process markdown files
    process_markdown_files(
        args.source_dir,
        args.collection_name,
        client,
        embedding_model,
        args.chunk_size,
        args.overlap,
        dry_run=args.dry_run
    )

    logger.info("Markdown to Qdrant conversion completed successfully!")


if __name__ == "__main__":
    main()

