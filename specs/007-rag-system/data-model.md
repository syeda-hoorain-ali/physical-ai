# Data Model: RAG System for Document Retrieval

## Core Entities

### Document Chunk
- **id**: string (Qdrant point ID)
- **content**: string (text content of the document chunk)
- **source_file**: string (path to original document)
- **file_name**: string (name of the original document)
- **chunk_index**: integer (position of chunk in original document)
- **similarity_score**: float (relevance score to user query)
- **vector**: float array (embedding vector for semantic search)

### Query Embedding
- **query_text**: string (original user query)
- **vector**: float array (embedding vector for semantic search)
- **timestamp**: datetime (when the query was processed)

### Context Window
- **query**: string (user's original query)
- **relevant_chunks**: array of Document Chunk (top 3-5 most relevant chunks)
- **combined_context**: string (formatted context for AI agent)
- **retrieval_metadata**: object (information about the retrieval process)

## Service Data Flows

### Document Retrieval Process
1. User query → Query Embedding (generate vector from text)
2. Query Embedding → Qdrant search (find similar document chunks)
3. Qdrant results → Context Window (format for AI agent)
4. Context Window → AI Agent (generate response with document context)

### Caching Structure
- **cache_key**: string (hashed query text)
- **cached_result**: Context Window (previously retrieved context)
- **timestamp**: datetime (when cached)
- **ttl**: integer (time-to-live in seconds, 1800 for 30 minutes)

## Validation Rules

### Document Chunk Validation
- Content must not be empty
- Similarity score must be between 0 and 1
- Chunk index must be non-negative

### Query Processing Validation
- Query text must not exceed 500 characters
- Minimum similarity threshold of 0.3 for inclusion
- Maximum of 5 chunks returned per query

### Context Window Validation
- Combined context must not exceed AI model's token limit
- At least one relevant chunk required when documents exist
- Fallback to empty context when no relevant documents found