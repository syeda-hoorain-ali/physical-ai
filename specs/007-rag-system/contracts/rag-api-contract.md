# API Contract: RAG System for Document Retrieval

## Overview
This document defines the API contract for the Retrieval-Augmented Generation (RAG) system that integrates document retrieval with the existing chat functionality.

## Base Path
All endpoints are relative to the backend service root (e.g., `http://localhost:8000`)

## Endpoints

### POST /chatkit
Main chat endpoint that now includes RAG functionality

#### Request
```json
{
  "message": "string (user query)",
  "thread_id": "string (optional, existing thread ID)"
}
```

#### Response
```json
{
  "response": "string (AI response incorporating document context when relevant)",
  "thread_id": "string (thread ID)",
  "timestamp": "string (ISO 8601 format)",
  "document_sources": [
    {
      "source_file": "string (document source)",
      "file_name": "string (document name)",
      "similarity_score": "number (0-1 similarity score)",
      "content_preview": "string (brief preview of relevant content)"
    }
  ],
  "retrieval_info": {
    "chunks_retrieved": "number (count of document chunks used)",
    "query_processed": "boolean (whether document retrieval was performed)",
    "fallback_mode": "boolean (whether system fell back to general knowledge)"
  }
}
```

#### Error Responses
- `400 Bad Request`: Invalid request format
- `422 Unprocessable Entity`: Invalid thread ID or other validation errors
- `500 Internal Server Error`: Backend service error, including Qdrant connectivity issues

## Internal Service Endpoints

### Document Retrieval Service
The following describes the internal service contract for document retrieval:

#### QdrantService.search_relevant_chunks(query: str, limit: int = 5) -> List[dict]
- **Input**: User query string and maximum number of chunks to retrieve
- **Output**: List of document chunks with metadata
- **Error handling**: Returns empty list if Qdrant is unavailable

#### Expected Chunk Format
```json
{
  "content": "string (document content)",
  "source_file": "string (path to original document)",
  "file_name": "string (name of the original document)",
  "chunk_index": "number (position in original document)",
  "similarity_score": "number (relevance score)"
}
```

## Authentication
The endpoint uses the same authentication as the base chatkit implementation (if any).

## Rate Limiting
Standard rate limits apply as defined in the base service (not modified by RAG functionality).

## Headers
- `Content-Type: application/json` required for requests
- Responses return `application/json`