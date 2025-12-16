# Data Model: Markdown to Qdrant Vector Conversion

## Document Chunk Entity
- **id**: Unique identifier for the chunk (string, required)
- **vector**: The embedding vector (list of floats, required)
- **payload**: Metadata object containing:
  - **source_file**: Original file path (string, required)
  - **chunk_index**: Position of chunk in original document (integer, required)
  - **content**: The text content of the chunk (string, required)
  - **original_file_path**: Relative path from source directory (string, required)
  - **file_name**: Name of the original file (string, required)

## Qdrant Collection Schema
- **collection_name**: Name of the Qdrant collection (string, required)
- **vector_size**: Dimension of the embedding vectors (integer, required)
- **distance_metric**: Distance metric for similarity search (string, default: "Cosine")

## Configuration Parameters
- **source_dir**: Directory containing markdown files to process (string, required)
- **chunk_size**: Size of text chunks (integer, default: 1000)
- **overlap**: Overlap between chunks (integer, default: 100)
- **qdrant_host**: Host address for Qdrant instance (string, default: "localhost")
- **qdrant_port**: Port for Qdrant instance (integer, default: 6333)
- **qdrant_api_key**: Authentication key for Qdrant (string, optional)
- **collection_name**: Name of the Qdrant collection (string, default: "markdown_docs")
