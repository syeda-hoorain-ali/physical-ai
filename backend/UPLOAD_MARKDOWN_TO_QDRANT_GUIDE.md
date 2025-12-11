# Markdown to Qdrant Converter Guide

## Overview
This script converts markdown files into vector embeddings and uploads them to a Qdrant vector database for semantic search capabilities. Use this tool to index your documentation for AI-powered retrieval and search functionality.

**Location**: `scripts/markdown_to_qdrant.py`

**Prerequisites**:
```bash
uv sync
```

**Basic Usage**:

**Dry Run Mode** (Recommended for testing without uploading to Qdrant):
```bash
uv run backend/scripts/markdown_to_qdrant.py --source-dir <source_directory> --collection-name <collection_name> --chunk-size 512 --overlap 50 --dry-run
```

**Production Mode** (With actual Qdrant upload, requires Qdrant server running):
```bash
uv run backend/scripts/markdown_to_qdrant.py --source-dir <source_directory> --collection-name <collection_name> --chunk-size 512 --overlap 50
```

**Parameters**:
-  `--source-dir`: Directory containing markdown files
-  `--collection-name`: Name of the Qdrant collection to store vectors
-  `--qdrant-host`: Qdrant host (default: localhost)
-  `--qdrant-port`: Qdrant port (default: 6333)
-  `--qdrant-api-key`: Qdrant API key (optional)
-  `--chunk-size`:  Size of text chunks (default: 1000)
-  `--overlap` Overlap between chunks in characters (default: 100)
-  `--dry-run`: Run without uploading to Qdrant (optional flag for testing)

**Example**:
```bash
# From the backend directory
uv run scripts/markdown_to_qdrant.py \
  --source-dir ./test_markdown \
  --collection-name example_docs \
  --chunk-size 512 \
  --overlap 50 \
  --dry-run
```

**Dependencies**:
- `qdrant_client`: For interacting with Qdrant vector database
- `fastembed`: For generating vector embeddings
- `markdown`: For parsing markdown files
- `beautifulsoup4`: For HTML processing during conversion

**Troubleshooting**:
- If dependencies are missing, install them using the uv command mentioned above
- Ensure the source directory exists and contains valid markdown files
- For Qdrant connection issues, verify that the Qdrant server is running
- Use the `--dry-run` flag to test the conversion process without affecting Qdrant
