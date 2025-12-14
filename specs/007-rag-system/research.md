# Research: RAG System for Document Retrieval

## Decision: Qdrant Integration Approach
**Rationale**: The system will use the existing Qdrant setup with the `BAAI/bge-small-en-v1.5` embedding model for semantic search. This aligns with the existing workflow in `.github/workflows/sync_docs_to_qdrant.yml` which already uploads documents to Qdrant.

**Alternatives considered**:
- Using a different embedding model (e.g., OpenAI embeddings) - rejected due to consistency with existing document upload process
- Using a different vector database - rejected due to existing Qdrant infrastructure

## Decision: Service Architecture
**Rationale**: A dedicated Qdrant service class will be created to handle vector search operations. This service will integrate with the existing `CustomChatKitServer` in the backend to perform document retrieval before AI agent processing.

**Alternatives considered**:
- Direct integration without a service layer - rejected for maintainability and testability
- Separate microservice - rejected due to project simplicity requirements

## Decision: Embedding Generation
**Rationale**: The system will use the same `BAAI/bge-small-en-v1.5` model that was used for document vectorization to generate embeddings for user queries. This ensures compatibility with existing document vectors in Qdrant.

**Alternatives considered**:
- Using different embedding models for queries vs documents - rejected due to complexity and potential incompatibility
- Using the AI model's own embedding capabilities - rejected due to consistency with existing document vectors

## Decision: Context Integration Method
**Rationale**: Retrieved document chunks will be formatted as context and appended to the user's query before sending to the AI agent. This approach maintains compatibility with the existing agent interface while providing relevant document information.

**Alternatives considered**:
- Modifying the agent's system prompt dynamically - rejected due to complexity
- Creating a separate agent for document queries - rejected due to user experience fragmentation

## Decision: Caching Strategy
**Rationale**: A simple in-memory cache with 30-minute TTL will be implemented to store recent query results, improving performance for repeated queries while maintaining reasonable freshness.

**Alternatives considered**:
- No caching - rejected due to performance concerns
- Distributed caching (Redis) - rejected due to project complexity requirements
- File-based caching - rejected due to performance concerns

## Decision: Fallback Handling
**Rationale**: When Qdrant is unavailable or no relevant documents are found, the system will gracefully fall back to normal AI responses, ensuring continuous service availability.

**Alternatives considered**:
- Returning errors when Qdrant is unavailable - rejected due to user experience concerns
- Blocking responses until Qdrant is available - rejected due to availability concerns