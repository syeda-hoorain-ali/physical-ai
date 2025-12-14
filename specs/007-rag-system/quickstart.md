# Quickstart: RAG System for Document Retrieval

## Overview
This guide will help you set up and run the Retrieval-Augmented Generation (RAG) system that enables the AI agent to retrieve relevant information from documents stored in Qdrant vector database. The system performs semantic search on user queries, retrieves top 3-5 relevant document chunks, and incorporates this information into the AI agent's response while maintaining the existing conversational flow and fallback mechanisms.

## Architecture
The RAG system consists of the following key components:

### Qdrant Service (`src/services/qdrant_service.py`)
- Handles vector search operations in Qdrant database
- Uses `BAAI/bge-small-en-v1.5` embedding model for semantic similarity
- Implements caching with 30-minute TTL for improved performance
- Includes minimum similarity threshold (0.3) filtering
- Provides health checks and graceful fallback mechanisms

### Server Integration (`src/services/server.py`)
- Integrates QdrantService into CustomChatKitServer
- Extracts user queries and performs document retrieval
- Formats retrieved document chunks as context for AI agent
- Handles fallback scenarios when no documents match
- Maintains conversation history and thread management

### Configuration (`src/config.py`)
- Manages Qdrant connection settings (host, port, API key, collection)
- Supports environment variable configuration via .env files

## Prerequisites
- Python 3.13+
- Qdrant vector database running (with document vectors already uploaded)
- Backend service dependencies installed
- Environment variables configured
- Documents must be pre-indexed in Qdrant using the same embedding model

## Environment Setup

### 1. Install Dependencies
```bash
cd backend
uv sync
```

### 2. Set Environment Variables
Create a `.env` file in the backend directory:
```env
# Qdrant Configuration
QDRANT_HOST=localhost
QDRANT_PORT=6333
QDRANT_API_KEY=your_api_key_here  # Optional, if using authentication
QDRANT_COLLECTION_NAME=docs_collection

# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key

# Neon Database Configuration
NEON_DATABASE_URL=postgresql://username:password@region.aws.neon.tech/dbname?sslmode=require

# Application Configuration
HOST=0.0.0.0
PORT=8000
LOG_LEVEL=INFO
FRONTEND_BASE_URL=http://localhost:3000
```

## Running the Service

### 1. Start the Backend Service
```bash
cd backend
uv run python -m src.main
```

Or using uvicorn:
```bash
cd backend
uv run uvicorn src.main:app --port 8000 --reload
```

### 2. Verify the Service
The service will be available at `http://localhost:8000`

## Using the RAG System

### 1. Send a Document-Related Query
Use the `/chatkit` endpoint to send a query related to your documents:
```bash
curl -X POST http://localhost:8000/chatkit \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What does the document say about physical AI?",
    "thread_id": "test-thread-123"
  }'
```

### 2. Expected Response with Document Context
When relevant documents are found, the response will incorporate information from document chunks with proper attribution. The system will include:
- Retrieved document excerpts with source attribution
- Relevance scores for each document chunk
- Proper fallback when no documents match the query

### 3. Fallback Behavior
When no relevant documents are found or Qdrant is unavailable:
- System gracefully falls back to general AI knowledge
- User is notified that no relevant documents were found
- Response continues to be generated using general AI capabilities

## Performance and Monitoring

### Response Time Goals
- Document retrieval should complete in under 3 seconds
- System includes performance monitoring and logging
- Cache improves repeated query performance

### Caching Strategy
- Results cached with 30-minute TTL
- Cache key includes query text and result limit
- Automatic cache invalidation based on TTL

## Testing the System

### Unit Tests
Run unit tests for the Qdrant service:
```bash
cd backend
uv run pytest tests/unit/test_qdrant_service.py
```

### Integration Tests
Run integration tests for RAG functionality:
```bash
cd backend
uv run pytest tests/integration/test_rag_integration.py
```

## Key Components

### Qdrant Service
Located at `src/services/qdrant_service.py`, this service handles:
- Semantic search in the Qdrant vector database using BAAI/bge-small-en-v1.5 model
- Retrieval of relevant document chunks (top 3-5 results)
- Caching of search results with 30-minute TTL
- Minimum similarity threshold (0.3) filtering
- Health checks and error handling
- Performance monitoring with timing logs

### Server Integration
The `CustomChatKitServer` in `src/services/server.py` has been modified to:
- Perform document retrieval before AI processing
- Integrate retrieved context with user queries as system messages
- Handle fallback scenarios when no documents match
- Extract query text from conversation context
- Track document sources and retrieval metadata

## Troubleshooting

### Qdrant Connection Issues
- Verify Qdrant is running at the configured host/port
- Check that the collection name matches your uploaded documents
- Ensure the Qdrant API key is correct if authentication is enabled
- Confirm the embedding model matches between document upload and query processing

### No Document Results
- Verify that documents have been properly uploaded to Qdrant
- Check that the embedding model matches between document upload and query processing
- Ensure the similarity threshold is appropriate for your use case
- Confirm document content is relevant to your queries

### Performance Issues
- Monitor the cache hit rate in application logs
- Adjust the number of retrieved chunks if response times are too slow
- Consider indexing optimizations in Qdrant for large collections
- Check that the embedding generation is working efficiently

### Configuration Issues
- Ensure all required environment variables are set
- Verify Qdrant collection contains document vectors
- Check that the embedding model is correctly configured