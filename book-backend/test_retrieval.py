"""
Unit tests for the semantic retrieval pipeline.
"""

import pytest
import os
from unittest.mock import Mock, patch
from retrieve import SemanticRetriever, RetrievalResult


class TestSemanticRetriever:
    """Test class for SemanticRetriever functionality."""

    @patch('retrieve.cohere.Client')
    @patch('retrieve.QdrantClient')
    @patch('retrieve.load_dotenv')
    def test_initialization(self, mock_load_dotenv, mock_qdrant, mock_cohere):
        """Test initialization of SemanticRetriever."""
        # Mock environment variables
        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.io'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'

        # Create instance
        retriever = SemanticRetriever()

        # Verify initialization
        assert retriever.collection_name == 'book_embeddings'
        mock_load_dotenv.assert_called_once()
        mock_cohere.assert_called_once()
        mock_qdrant.assert_called_once()

        # Clean up environment variables
        del os.environ['COHERE_API_KEY']
        del os.environ['QDRANT_URL']
        del os.environ['QDRANT_API_KEY']

    @patch('retrieve.cohere.Client')
    @patch('retrieve.QdrantClient')
    @patch('retrieve.load_dotenv')
    def test_initialization_missing_env_vars(self, mock_load_dotenv, mock_qdrant, mock_cohere):
        """Test initialization fails when environment variables are missing."""
        # Ensure environment variables are not set
        if 'COHERE_API_KEY' in os.environ:
            del os.environ['COHERE_API_KEY']
        if 'QDRANT_URL' in os.environ:
            del os.environ['QDRANT_URL']
        if 'QDRANT_API_KEY' in os.environ:
            del os.environ['QDRANT_API_KEY']

        # Should raise ValueError
        with pytest.raises(ValueError):
            SemanticRetriever()

    @patch('retrieve.cohere.Client')
    @patch('retrieve.QdrantClient')
    @patch('retrieve.load_dotenv')
    def test_embed_query(self, mock_load_dotenv, mock_qdrant, mock_cohere):
        """Test query embedding functionality."""
        # Mock environment variables
        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.io'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'

        # Mock cohere response
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.return_value = Mock()
        mock_cohere_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]
        mock_cohere.return_value = mock_cohere_instance

        retriever = SemanticRetriever()
        embedding = retriever._embed_query("test query")

        # Verify the call was made
        mock_cohere_instance.embed.assert_called_once_with(
            texts=["test query"],
            model="embed-english-v3.0",
            input_type="search_query"
        )

        assert embedding == [0.1, 0.2, 0.3]

        # Clean up
        del os.environ['COHERE_API_KEY']
        del os.environ['QDRANT_URL']
        del os.environ['QDRANT_API_KEY']

    def test_format_results(self):
        """Test formatting of Qdrant query_points results."""
        # Create mock Qdrant response object with points
        class MockPoint:
            def __init__(self, id, score, payload):
                self.id = id
                self.score = score
                self.payload = payload

        class MockResponse:
            def __init__(self, points):
                self.points = points

        mock_points = [
            MockPoint(
                id='test-id-1',
                score=0.85,
                payload={
                    'text': 'This is a test text',
                    'source_url': 'https://example.com/page1',
                    'source_title': 'Test Page 1',
                    'text_chunk_id': 'chunk-1',
                    'content_preview': 'This is a test',
                    'created_at': '2023-01-01',
                    'original_id': 'orig-1'
                }
            )
        ]

        mock_response = MockResponse(mock_points)

        results = SemanticRetriever._format_results(None, mock_response)

        assert len(results) == 1
        result = results[0]
        assert result.id == 'test-id-1'
        assert result.score == 0.85
        assert result.text == 'This is a test text'
        assert result.source_url == 'https://example.com/page1'
        assert result.source_title == 'Test Page 1'
        assert result.text_chunk_id == 'chunk-1'
        assert result.content_preview == 'This is a test'
        assert result.created_at == '2023-01-01'
        assert result.original_id == 'orig-1'

    @patch('retrieve.cohere.Client')
    @patch('retrieve.QdrantClient')
    @patch('retrieve.load_dotenv')
    def test_validate_retrieval(self, mock_load_dotenv, mock_qdrant, mock_cohere):
        """Test content validation functionality."""
        # Mock environment variables
        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.io'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'

        retriever = SemanticRetriever()

        # Create test results
        test_results = [
            RetrievalResult(
                id='test-1',
                text='This is a test text for validation',
                score=0.8,
                source_url='https://example.com',
                source_title='Test Page',
                text_chunk_id='chunk-1',
                content_preview='This is a test',
                created_at='2023-01-01',
                original_id='orig-1'
            )
        ]

        # Should pass validation
        is_valid = retriever.validate_retrieval("test query", test_results)
        assert is_valid is True

        # Create result with mismatched content
        bad_result = [
            RetrievalResult(
                id='test-2',
                text='This is completely different content',
                score=0.8,
                source_url='https://example.com',
                source_title='Test Page',
                text_chunk_id='chunk-2',
                content_preview='This is a test',
                created_at='2023-01-01',
                original_id='orig-2'
            )
        ]

        # Should fail validation
        is_valid = retriever.validate_retrieval("test query", bad_result)
        assert is_valid is False

        # Clean up
        del os.environ['COHERE_API_KEY']
        del os.environ['QDRANT_URL']
        del os.environ['QDRANT_API_KEY']


if __name__ == "__main__":
    pytest.main([__file__])