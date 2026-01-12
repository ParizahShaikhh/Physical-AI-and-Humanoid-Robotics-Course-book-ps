"""
Semantic retrieval pipeline for book content using Cohere embeddings and Qdrant vector search.

This module provides functionality to:
- Convert natural language queries to embeddings using Cohere
- Perform semantic similarity search in Qdrant
- Filter results by metadata (source URL, page, section)
- Validate retrieved content against original book content
"""

import os
import logging
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


# Set up logging configuration
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class RetrievalResult:
    """Data structure for query results with metadata."""
    id: str
    text: str
    score: float
    source_url: str
    source_title: str
    text_chunk_id: str
    content_preview: str
    created_at: str
    original_id: str


class SemanticRetriever:
    """Main class for semantic retrieval functionality."""

    def __init__(self):
        """Initialize Cohere and Qdrant clients."""
        # Load environment variables
        load_dotenv()

        # Initialize Cohere client
        cohere_api_key = os.getenv('COHERE_API_KEY')
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable not found")
        self.cohere_client = cohere.Client(cohere_api_key)

        # Initialize Qdrant client
        qdrant_url = os.getenv('QDRANT_URL')
        qdrant_api_key = os.getenv('QDRANT_API_KEY')
        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL or QDRANT_API_KEY environment variables not found")

        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False
        )

        # Collection name used in the ingestion pipeline
        self.collection_name = 'book_embeddings'

        logger.info("SemanticRetriever initialized successfully")

    def _embed_query(self, query: str) -> List[float]:
        """
        Convert a natural language query to an embedding vector using Cohere.

        Args:
            query: Natural language query string

        Returns:
            Embedding vector as a list of floats
        """
        try:
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",  # Using a valid Cohere embedding model
                input_type="search_query"    # Required for v3 models
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error embedding query: {str(e)}")
            raise

    def _format_results(self, search_response) -> List[RetrievalResult]:
        """
        Format Qdrant query_points results into RetrievalResult objects.

        Args:
            search_response: Response from Qdrant query_points call

        Returns:
            List of RetrievalResult objects
        """
        results = []
        # In the new API, results are in search_response.points
        for point in search_response.points:
            payload = point.payload
            result = RetrievalResult(
                id=str(point.id),
                text=payload.get('text', payload.get('content_preview', '')),  # Use stored text content, fallback to preview
                score=point.score,
                source_url=payload.get('source_url', ''),
                source_title=payload.get('source_title', ''),
                text_chunk_id=payload.get('text_chunk_id', ''),
                content_preview=payload.get('content_preview', ''),
                created_at=payload.get('created_at', ''),
                original_id=payload.get('original_id', '')
            )
            results.append(result)
        return results

    def search(
        self,
        query: str,
        top_k: int = 5,
        filters: Optional[Dict[str, str]] = None
    ) -> List[RetrievalResult]:
        """
        Perform semantic search on the book content.

        Args:
            query: Natural language query string
            top_k: Number of top results to return
            filters: Optional metadata filters (e.g., {'source_url': '...'})

        Returns:
            List of RetrievalResult objects sorted by relevance
        """
        logger.info(f"Performing semantic search for query: '{query[:50]}...'")

        # Convert query to embedding
        query_embedding = self._embed_query(query)

        # Build filters if provided
        qdrant_filters = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                # Create exact match conditions for metadata fields
                # Based on the ingestion pipeline, metadata is stored at the top level
                filter_conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )

            if filter_conditions:
                qdrant_filters = models.Filter(
                    must=filter_conditions
                )

        # Perform search in Qdrant using the new API
        try:
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                query_filter=qdrant_filters,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            formatted_results = self._format_results(search_results)
            logger.info(f"Retrieved {len(formatted_results)} results for query")
            return formatted_results

        except Exception as e:
            logger.error(f"Error during semantic search: {str(e)}")
            raise

    def validate_retrieval(self, query: str, results: List[RetrievalResult]) -> bool:
        """
        Validate that retrieved content matches original book content.

        Args:
            query: Original query string
            results: List of retrieval results to validate

        Returns:
            True if all results pass validation, False otherwise
        """
        logger.info(f"Validating {len(results)} retrieval results")

        for result in results:
            # Check if the content preview matches the beginning of the full text
            if result.content_preview and result.text:
                # Normalize both strings for comparison
                preview_normalized = result.content_preview.strip().lower()
                text_normalized = result.text.strip().lower()

                # Check if the text starts with the preview
                if not text_normalized.startswith(preview_normalized):
                    logger.warning(f"Content mismatch detected for result {result.id}")
                    return False

        logger.info("All retrieval results passed validation")
        return True


def example_usage():
    """Example usage of the SemanticRetriever class."""
    print("Initializing SemanticRetriever...")
    retriever = SemanticRetriever()

    print("\nExample 1: Basic semantic search")
    query1 = "What is ROS 2 and how does it differ from ROS 1?"
    results1 = retriever.search(query1, top_k=3)

    print(f"Query: {query1}")
    for i, result in enumerate(results1, 1):
        print(f"Result {i}:")
        print(f"  Source: {result.source_title}")
        print(f"  URL: {result.source_url}")
        print(f"  Score: {result.score:.3f}")
        print(f"  Preview: {result.content_preview[:100]}...")
        print()

    print("Example 2: Search with metadata filtering")
    query2 = "How does NVIDIA Isaac work?"
    filters = {"source_url": "https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/docs/module-3-ai-robot-brain/"}
    results2 = retriever.search(query2, top_k=2, filters=filters)

    print(f"Query: {query2}")
    print(f"Filters: {filters}")
    for i, result in enumerate(results2, 1):
        print(f"Result {i}:")
        print(f"  Source: {result.source_title}")
        print(f"  URL: {result.source_url}")
        print(f"  Score: {result.score:.3f}")
        print(f"  Preview: {result.content_preview[:100]}...")
        print()

    print("Example 3: Validation of retrieved content")
    is_valid = retriever.validate_retrieval(query1, results1)
    print(f"Content validation passed: {is_valid}")


if __name__ == "__main__":
    import sys
    import argparse

    # Check if command line arguments are provided
    if len(sys.argv) > 1:
        # Parse command line arguments
        parser = argparse.ArgumentParser(description='Semantic retrieval for book content')
        parser.add_argument('--query', '-q', type=str, help='Natural language query to search for')
        parser.add_argument('--top_k', type=int, default=5, help='Number of top results to return (default: 5)')
        parser.add_argument('--filter_url', type=str, help='Filter results by source URL')

        args = parser.parse_args()

        if args.query:
            try:
                print(f"Searching for: '{args.query}'")
                print()

                retriever = SemanticRetriever()

                # Prepare filters if provided
                filters = {}
                if args.filter_url:
                    filters = {'source_url': args.filter_url}

                # Perform search
                results = retriever.search(args.query, top_k=args.top_k, filters=filters or None)

                if results:
                    print(f"Found {len(results)} results for query: '{args.query}'")
                    print()

                    for i, result in enumerate(results, 1):
                        print(f"Result {i}:")
                        print(f"  Title: {result.source_title}")
                        print(f"  URL: {result.source_url}")
                        print(f"  Score: {result.score:.3f}")
                        print(f"  Content Preview: {result.content_preview[:500]}...")
                        print(f"  Text Content: {result.text[:1000] if result.text else 'Content not stored in this embedding (only metadata available)'}")
                        print(f"  Source: {result.source_url}")
                        print()
                else:
                    print("No results found for the query.")
            except Exception as e:
                logger.error(f"Error during search: {str(e)}")
                if "429" in str(e) or "Too Many Requests" in str(e):
                    print("API rate limit exceeded. Please wait a few minutes and try again.")
                else:
                    print(f"Error: {e}")
        else:
            # No query provided, run example usage
            example_usage()
    else:
        # No arguments provided, run example usage
        example_usage()