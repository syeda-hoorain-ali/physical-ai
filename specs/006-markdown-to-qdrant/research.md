# Research: Markdown to Qdrant Vector Conversion

## Decision: Use qdrant-client[fastembed] package
**Rationale**: The fastembed library provides efficient, lightweight embedding generation that works well with Qdrant. It's designed specifically for this use case and offers good performance without requiring complex setup like other embedding models.

## Decision: Python script structure for markdown processing
**Rationale**: A dedicated Python script will handle the markdown parsing, chunking, embedding generation, and storage in Qdrant. This approach keeps the functionality self-contained and reusable.

## Decision: Text chunking strategy
**Rationale**: Large markdown files will be split into smaller chunks to maintain context while fitting within embedding model limits. Default chunk size of 1000 characters with 100-character overlap will preserve context across splits.

## Decision: Metadata storage approach
**Rationale**: Each vector in Qdrant will include metadata with source file path, chunk index, and original content to enable proper retrieval and attribution when performing semantic searches.

## Alternatives considered:
- Using sentence-transformers library: More complex setup and dependencies required
- Using OpenAI embeddings: Requires API key and costs money for usage
- Using Hugging Face models directly: Requires more complex model management
- Using Gemini API: Requires Google Cloud setup and authentication
