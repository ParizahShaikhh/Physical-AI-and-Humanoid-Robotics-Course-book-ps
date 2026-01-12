"""
Unit tests for the RAG Agent.
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from agent import RAGAgent, RetrievalResult


class TestRAGAgent:
    """Test class for RAGAgent functionality."""

    def test_rate_limiting_functionality(self):
        """Test rate limiting functionality."""
        # Test that rate limiting methods exist and work
        agent = RAGAgent.__new__(RAGAgent)  # Create without __init__ to control setup
        agent.rate_limit_enabled = True
        agent.max_requests_per_minute = 2
        agent.request_timestamps = []

        # First request should be allowed
        assert agent._check_rate_limit() is True

        # Second request should be allowed
        assert agent._check_rate_limit() is True

        # Third request should exceed limit
        assert agent._check_rate_limit() is False

    def test_cache_functionality(self):
        """Test caching functionality."""
        agent = RAGAgent.__new__(RAGAgent)  # Create without __init__ to control setup
        agent.enable_caching = True
        agent.cache_ttl = 3600
        agent.response_cache = {}

        # Create a cache key
        cache_key = agent._create_cache_key("test query", "full-book", {}, 5)

        # Cache a response
        agent._cache_response(cache_key, "test response")

        # Retrieve from cache
        cached_response = agent._get_cached_response(cache_key)
        assert cached_response == "test response"

        # Test cache expiration by setting past timestamp
        agent.response_cache[cache_key] = {
            "response": "expired response",
            "timestamp": 0  # Very old timestamp
        }
        expired_response = agent._get_cached_response(cache_key)
        assert expired_response is None  # Should be expired and removed

    def test_parameter_validation(self):
        """Test parameter validation functionality."""
        agent = RAGAgent.__new__(RAGAgent)  # Create without __init__ to control setup

        # Test empty query validation
        with pytest.raises(ValueError):
            agent.search_book_content("", mode="full-book", top_k=5)

        # Test whitespace query validation
        with pytest.raises(ValueError):
            agent.search_book_content("   ", mode="full-book", top_k=5)

        # Test invalid top_k validation
        with pytest.raises(ValueError):
            agent.search_book_content("test", mode="full-book", top_k=0)

        with pytest.raises(ValueError):
            agent.search_book_content("test", mode="full-book", top_k=25)

        # Test invalid mode validation
        with pytest.raises(ValueError):
            agent.search_book_content("test", mode="invalid-mode", top_k=5)

        # Valid parameters should not raise exceptions
        try:
            # Mock the actual search call to avoid real API calls
            agent.retriever = Mock()
            agent.retriever.search = Mock(return_value=[])
            agent.logger = Mock()
            agent.search_book_content("valid query", mode="full-book", top_k=5)
        except ValueError:
            pytest.fail("Valid parameters raised ValueError unexpectedly")

    def test_confidence_level_calculation(self):
        """Test confidence level calculation functionality."""
        agent = RAGAgent.__new__(RAGAgent)  # Create without __init__ to control setup

        # Test different confidence levels
        assert agent._get_confidence_level(0.9) == "High"
        assert agent._get_confidence_level(0.7) == "Medium"
        assert agent._get_confidence_level(0.5) == "Low"
        assert agent._get_confidence_level(0.2) == "Very Low"

    def test_create_cache_key(self):
        """Test cache key creation."""
        agent = RAGAgent.__new__(RAGAgent)  # Create without __init__ to control setup

        # Test that identical parameters produce the same key
        key1 = agent._create_cache_key("test query", "full-book", {}, 5)
        key2 = agent._create_cache_key("test query", "full-book", {}, 5)
        assert key1 == key2

        # Test that different parameters produce different keys
        key3 = agent._create_cache_key("different query", "full-book", {}, 5)
        assert key1 != key3


if __name__ == "__main__":
    pytest.main([__file__])