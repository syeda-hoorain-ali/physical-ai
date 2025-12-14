"""
Unit tests for QdrantService class.

These tests verify the functionality of the QdrantService without requiring
a real Qdrant instance, using mocking to isolate the service logic.
"""
import pytest
from unittest.mock import Mock, patch
from src.services.qdrant_service import QdrantService


@pytest.fixture
def qdrant_service():
    """Create a QdrantService instance for testing."""
    with patch('src.services.qdrant_service.QdrantClient'), \
         patch('src.services.qdrant_service.TextEmbedding'):
        service = QdrantService(
            host="test_host",
            port=1234,
            collection_name="test_collection"
        )
        # Mock the embedding model to return predictable values
        service.embedding_model = Mock()
        service.embedding_model.embed.return_value = [Mock(tolist=lambda: [0.1, 0.2, 0.3])]
        yield service


@pytest.mark.asyncio
async def test_qdrant_service_initialization(qdrant_service):
    """Test that QdrantService initializes correctly with provided parameters."""
    assert qdrant_service.host == "test_host"
    assert qdrant_service.port == 1234
    assert qdrant_service.collection_name == "test_collection"
    assert qdrant_service.cache == {}
    assert qdrant_service.cache_ttl == 1800


@pytest.mark.asyncio
async def test_generate_cache_key():
    """Test cache key generation."""
    with patch('src.services.qdrant_service.QdrantClient'), \
         patch('src.services.qdrant_service.TextEmbedding'):
        service = QdrantService()

    cache_key = service._generate_cache_key("test query", 5)
    assert cache_key == "test query:5"

    # Test with different limit
    cache_key2 = service._generate_cache_key("test query", 10)
    assert cache_key2 == "test query:10"


@pytest.mark.asyncio
async def test_cache_operations(qdrant_service):
    """Test cache get/set operations."""
    test_query = "test query"
    test_results = [{"content": "test content", "score": 0.8}]

    # Initially should not be in cache
    result = qdrant_service._get_from_cache(test_query)
    assert result is None

    # Add to cache
    qdrant_service._set_cache(test_query, 5, test_results)

    # Should now be in cache
    result = qdrant_service._get_from_cache(test_query)
    assert result == test_results


@pytest.mark.asyncio
async def test_search_relevant_chunks_success(qdrant_service):
    """Test successful search for relevant chunks."""
    with patch.object(qdrant_service, 'client') as mock_client:
        from unittest.mock import AsyncMock
        # Mock search results
        mock_result = Mock()
        mock_result.score = 0.8
        mock_result.payload = {
            "content": "test content",
            "source_file": "test_source.md",
            "file_name": "test_source.md",
            "chunk_index": 1
        }
        mock_response = Mock()
        mock_response.points = [mock_result]
        mock_client.query_points = AsyncMock(return_value=mock_response)

        results = await qdrant_service.search_relevant_chunks("test query", limit=1)

        assert len(results) == 1
        assert results[0]["content"] == "test content"
        assert results[0]["similarity_score"] == 0.8
        assert results[0]["file_name"] == "test_source.md"


@pytest.mark.asyncio
async def test_search_relevant_chunks_low_similarity_filtered(qdrant_service):
    """Test that results below similarity threshold are filtered out."""
    with patch.object(qdrant_service, 'client') as mock_client:
        # Mock results with low scores
        mock_result1 = Mock()
        mock_result1.score = 0.2  # Below threshold
        mock_result1.payload = {
            "content": "low score content",
            "source_file": "test_source.md",
            "file_name": "test_source.md",
            "chunk_index": 1
        }

        mock_result2 = Mock()
        mock_result2.score = 0.8  # Above threshold
        mock_result2.payload = {
            "content": "high score content",
            "source_file": "test_source.md",
            "file_name": "test_source.md",
            "chunk_index": 2
        }

        mock_client.search.return_value = [mock_result1, mock_result2]

        results = await qdrant_service.search_relevant_chunks("test query", limit=5, min_similarity=0.3)

        # Only the high score result should be returned
        assert len(results) == 1
        assert results[0]["content"] == "high score content"
        assert results[0]["similarity_score"] == 0.8


@pytest.mark.asyncio
async def test_search_relevant_chunks_error_handling(qdrant_service):
    """Test that search handles errors gracefully."""
    with patch.object(qdrant_service, 'client') as mock_client:
        mock_client.search.side_effect = Exception("Connection error")

        results = await qdrant_service.search_relevant_chunks("test query")

        # Should return empty list on error
        assert results == []


@pytest.mark.asyncio
async def test_health_check_success(qdrant_service):
    """Test health check when Qdrant is accessible."""
    with patch.object(qdrant_service, 'client') as mock_client:
        mock_client.get_collection.return_value = True

        result = await qdrant_service.health_check()
        assert result is True


@pytest.mark.asyncio
async def test_health_check_failure(qdrant_service):
    """Test health check when Qdrant is not accessible."""
    with patch.object(qdrant_service, 'client') as mock_client:
        mock_client.get_collection.side_effect = Exception("Connection error")

        result = await qdrant_service.health_check()
        assert result is False


@pytest.mark.asyncio
async def test_cache_expiration(qdrant_service):
    """Test that cache entries expire after TTL."""
    import datetime
    from unittest.mock import PropertyMock

    test_query = "test query"
    test_results = [{"content": "test content", "score": 0.8}]

    # Manually add an expired entry to cache
    expired_time = datetime.datetime.now() - datetime.timedelta(seconds=qdrant_service.cache_ttl + 1)
    qdrant_service.cache["test query:5"] = (test_results, expired_time)

    # Should return None since cache is expired
    result = qdrant_service._get_from_cache(test_query)
    assert result is None

    # Cache entry should be removed
    assert "test query:5" not in qdrant_service.cache


@pytest.mark.asyncio
async def test_search_relevant_chunks_with_multiple_results(qdrant_service):
    """Test search with multiple relevant chunks that are properly sorted by similarity."""
    with patch.object(qdrant_service, 'client') as mock_client:
        # Create multiple mock results with different scores
        mock_results = []
        scores = [0.9, 0.7, 0.95, 0.6, 0.8]  # Unsorted scores
        for i, score in enumerate(scores):
            mock_result = Mock()
            mock_result.score = score
            mock_result.payload = {
                "content": f"content {i}",
                "source_file": f"source_{i}.md",
                "file_name": f"file_{i}.md",
                "chunk_index": i
            }
            mock_results.append(mock_result)

        mock_client.search.return_value = mock_results

        results = await qdrant_service.search_relevant_chunks("test query", limit=5)

        # Results should be sorted by similarity score (descending)
        assert len(results) == 5  # All results above default threshold
        for i in range(len(results) - 1):
            assert results[i]["similarity_score"] >= results[i + 1]["similarity_score"]


@pytest.mark.asyncio
async def test_search_relevant_chunks_limit_respected(qdrant_service):
    """Test that the limit parameter is properly respected."""
    with patch.object(qdrant_service, 'client') as mock_client:
        # Create more results than the limit
        mock_results = []
        for i in range(10):  # 10 results
            mock_result = Mock()
            mock_result.score = 0.8  # All above threshold
            mock_result.payload = {
                "content": f"content {i}",
                "source_file": f"source_{i}.md",
                "file_name": f"file_{i}.md",
                "chunk_index": i
            }
            mock_results.append(mock_result)

        mock_client.search.return_value = mock_results

        results = await qdrant_service.search_relevant_chunks("test query", limit=3)

        # Should respect the limit
        assert len(results) == 3


@pytest.mark.asyncio
async def test_search_relevant_chunks_embedding_generation_failure(qdrant_service):
    """Test behavior when embedding generation fails."""
    with patch.object(qdrant_service, 'client'), \
         patch.object(qdrant_service.embedding_model, 'embed', return_value=[]):
        # Mock embed to return empty list (failure case)
        results = await qdrant_service.search_relevant_chunks("test query")

        # Should return empty list when embeddings can't be generated
        assert results == []


@pytest.mark.asyncio
async def test_search_relevant_chunks_empty_payload_handling(qdrant_service):
    """Test handling of results with missing payload fields."""
    with patch.object(qdrant_service, 'client') as mock_client:
        # Mock result with empty/missing payload
        mock_result = Mock()
        mock_result.score = 0.8
        mock_result.payload = {}  # Empty payload

        mock_client.search.return_value = [mock_result]

        results = await qdrant_service.search_relevant_chunks("test query")

        # Should handle empty payload gracefully
        assert len(results) == 1
        assert results[0]["content"] == ""
        assert results[0]["source_file"] == ""
        assert results[0]["file_name"] == ""
        assert results[0]["chunk_index"] == 0
        assert results[0]["similarity_score"] == 0.8


@pytest.mark.asyncio
async def test_fallback_when_no_relevant_documents_found(qdrant_service):
    """Test fallback behavior when no documents meet the similarity threshold."""
    with patch.object(qdrant_service, 'client') as mock_client:
        # Mock results with scores below threshold
        mock_result = Mock()
        mock_result.score = 0.1  # Below default threshold of 0.3
        mock_result.payload = {
            "content": "low relevance content",
            "source_file": "source.md",
            "file_name": "source.md",
            "chunk_index": 1
        }
        mock_client.search.return_value = [mock_result]

        results = await qdrant_service.search_relevant_chunks("test query", min_similarity=0.3)

        # Should return empty list when no results meet threshold
        assert results == []


@pytest.mark.asyncio
async def test_fallback_on_qdrant_connection_error(qdrant_service):
    """Test fallback behavior when Qdrant connection fails."""
    with patch.object(qdrant_service, 'client') as mock_client:
        mock_client.search.side_effect = Exception("Connection failed")

        results = await qdrant_service.search_relevant_chunks("test query")

        # Should return empty list on connection error
        assert results == []


@pytest.mark.asyncio
async def test_fallback_on_embedding_generation_failure(qdrant_service):
    """Test fallback behavior when embedding generation fails."""
    with patch.object(qdrant_service, 'client'), \
         patch.object(qdrant_service.embedding_model, 'embed', return_value=[]):

        results = await qdrant_service.search_relevant_chunks("test query")

        # Should return empty list when embeddings can't be generated
        assert results == []


@pytest.mark.asyncio
async def test_health_check_fallback_on_error(qdrant_service):
    """Test that health check returns False on connection errors."""
    with patch.object(qdrant_service, 'client') as mock_client:
        mock_client.get_collection.side_effect = Exception("Connection error")

        result = await qdrant_service.health_check()

        # Should return False on error
        assert result is False