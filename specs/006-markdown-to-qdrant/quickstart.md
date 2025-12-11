# Quickstart: Markdown to Qdrant Vector Conversion

## Prerequisites
- Python 3.9+
- Qdrant instance running locally or accessible via network

## Setup
1. Install required dependencies:
   ```bash
   uv add qdrant-client[fastembed] markdown beautifulsoup4
   ```

2. Ensure Qdrant is running (default: localhost:6333)

## Usage
Run the conversion script:
```bash
python backend/scripts/markdown_to_qdrant.py --source-dir /path/to/markdown/files --collection-name my_docs
```

## Configuration Options
- `--source-dir`: Directory containing markdown files (required)
- `--collection-name`: Qdrant collection name (default: "markdown_docs")
- `--qdrant-host`: Qdrant host (default: "localhost")
- `--qdrant-port`: Qdrant port (default: 6333)
- `--qdrant-api-key`: Qdrant API key (optional)
- `--chunk-size`: Text chunk size (default: 1000)
- `--overlap`: Chunk overlap size (default: 100)

## Example
```bash
python backend/scripts/markdown_to_qdrant.py --source-dir ./book-source/docs --collection-name physical_ai_docs --chunk-size 1500
```

## Verification
After running the script, you can verify the vectors were stored by checking the Qdrant web UI or using the Qdrant client to query the collection.
