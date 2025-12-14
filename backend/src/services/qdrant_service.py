"""
Qdrant Service for RAG System

This service handles vector search operations in the Qdrant vector database,
retrieving relevant document chunks based on semantic similarity to user queries.
"""
import asyncio
import logging
import time
from typing import List, Dict, Optional, Any
from datetime import datetime, timedelta

from qdrant_client import AsyncQdrantClient
from fastembed import TextEmbedding


logger = logging.getLogger(__name__)


class QdrantService:
    """
    Service class for handling Qdrant vector search operations for RAG system.
    """

    def __init__(self, host: str = "localhost", port: int = 6333,
                 collection_name: str = "docs_collection", api_key: Optional[str] = None):
        """
        Initialize the Qdrant service with connection parameters.

        Args:
            host: Qdrant server host
            port: Qdrant server port
            collection_name: Name of the collection containing document vectors
            api_key: Optional API key for Qdrant authentication
        """
        self.host = host
        self.port = port
        self.collection_name = collection_name
        self.api_key = api_key

        # Initialize Async Qdrant client - handle both local and remote instances
        if self.host.startswith(('http://', 'https://')):
            # For remote/cloud instances, use the full URL
            self.client = AsyncQdrantClient(
                url=self.host,
                api_key=self.api_key,
            )
        else:
            # For local instances, use host/port
            self.client = AsyncQdrantClient(
                host=self.host,
                port=self.port,
                api_key=self.api_key,
            )

        # Initialize text embedding model
        self.embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")

        # Simple in-memory cache with TTL
        self.cache = {}
        self.cache_ttl = 1800  # 30 minutes in seconds

        logger.info(f"QdrantService initialized with collection: {self.collection_name}")

    def _generate_cache_key(self, query: str, limit: int = 5) -> str:
        """
        Generate a cache key for the given query and limit.

        Args:
            query: The search query
            limit: Maximum number of results to return

        Returns:
            Cache key string
        """
        return f"{query}:{limit}"

    def _is_cache_valid(self, timestamp: datetime) -> bool:
        """
        Check if cached entry is still valid based on TTL.

        Args:
            timestamp: When the cache entry was created

        Returns:
            True if cache is still valid, False otherwise
        """
        return datetime.now() - timestamp < timedelta(seconds=self.cache_ttl)

    def _get_from_cache(self, query: str, limit: int = 5) -> Optional[List[Dict[str, Any]]]:
        """
        Get cached results for a query if available and still valid.

        Args:
            query: The search query
            limit: Maximum number of results to return

        Returns:
            Cached results if available and valid, None otherwise
        """
        cache_key = self._generate_cache_key(query, limit)

        if cache_key in self.cache:
            cached_result, timestamp = self.cache[cache_key]
            if self._is_cache_valid(timestamp):
                logger.debug(f"Cache hit for query: {query[:50]}...")
                return cached_result
            else:
                # Remove expired cache entry
                del self.cache[cache_key]

        logger.debug(f"Cache miss for query: {query[:50]}...")
        return None

    def _set_cache(self, query: str, limit: int, results: List[Dict[str, Any]]) -> None:
        """
        Store results in cache.

        Args:
            query: The search query
            limit: Maximum number of results to return
            results: Search results to cache
        """
        cache_key = self._generate_cache_key(query, limit)
        self.cache[cache_key] = (results, datetime.now())
        logger.debug(f"Cached results for query: {query[:50]}...")

    async def search_relevant_chunks(self, query: str, limit: int = 5,
                                   min_similarity: float = 0.3) -> List[Dict[str, Any]]:
        """
        Search for relevant document chunks based on semantic similarity to the query.

        Args:
            query: User query to search for relevant documents
            limit: Maximum number of chunks to retrieve (default: 5)
            min_similarity: Minimum similarity threshold (default: 0.3)

        Returns:
            List of document chunks with metadata, sorted by relevance
        """
        start_time = time.time()

        try:
            # Check cache first
            cached_results = self._get_from_cache(query, limit)
            if cached_results is not None:
                cache_time = time.time() - start_time
                logger.info(f"Cache hit for query: '{query[:50]}...', retrieved {len(cached_results)} chunks in {cache_time:.3f}s")
                return cached_results

            logger.info(f"Searching for relevant chunks for query: {query}")

            # Generate embedding for the query
            embed_start_time = time.time()
            query_embeddings = await asyncio.to_thread(list, self.embedding_model.embed([query]))
            embed_time = time.time() - embed_start_time

            if not query_embeddings:
                logger.warning("Failed to generate embedding for query")
                total_time = time.time() - start_time
                logger.info(f"Query processing failed after {total_time:.3f}s (embedding: {embed_time:.3f}s)")
                return []

            query_vector = query_embeddings[0].tolist()
            logger.debug(f"Generated embedding in {embed_time:.3f}s")

            # Perform semantic search in Qdrant using async client
            search_start_time = time.time()
            search_results = await self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit * 2,  # Get more results than needed for filtering
                with_payload=True,
                score_threshold=min_similarity
            )
            search_time = time.time() - search_start_time
            logger.debug(f"Qdrant search completed in {search_time:.3f}s")

            # Format results to match expected structure
            format_start_time = time.time()
            formatted_results = []
            for result in search_results.points:
                # For async client, results should have score and payload attributes
                similarity_score = getattr(result, 'score', getattr(result, 'similarity', 0))
                if similarity_score >= min_similarity:
                    # Extract payload from the result
                    payload = getattr(result, 'payload', {})

                    formatted_result = {
                        "content": payload.get("content", ""),
                        "source_file": payload.get("source_file", ""),
                        "file_name": payload.get("file_name", ""),
                        "chunk_index": payload.get("chunk_index", 0),
                        "similarity_score": similarity_score
                    }
                    formatted_results.append(formatted_result)

            # Sort by similarity score (descending) and limit to requested number
            formatted_results.sort(key=lambda x: x["similarity_score"], reverse=True)
            formatted_results = formatted_results[:limit]
            format_time = time.time() - format_start_time
            logger.debug(f"Results formatting completed in {format_time:.3f}s")

            # Cache the results
            cache_set_start_time = time.time()
            self._set_cache(query, limit, formatted_results)
            cache_set_time = time.time() - cache_set_start_time
            logger.debug(f"Results cached in {cache_set_time:.3f}s")

            logger.info(f"Found {len(formatted_results)} relevant chunks for query in {time.time() - start_time:.3f}s "
                       f"(embed: {embed_time:.3f}s, search: {search_time:.3f}s, format: {format_time:.3f}s, cache: {cache_set_time:.3f}s)")
            return formatted_results

        except Exception as e:
            total_time = time.time() - start_time
            logger.error(f"Error searching for relevant chunks after {total_time:.3f}s: {str(e)}", exc_info=True)
            # Return empty list on error to allow fallback behavior
            return []

    async def health_check(self) -> bool:
        """
        Check if Qdrant service is accessible and operational.

        Returns:
            True if service is healthy, False otherwise
        """
        try:
            # Try to get collection info as a simple health check
            # For async client, we need to await the operation
            await self.client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {str(e)}")
            return False
