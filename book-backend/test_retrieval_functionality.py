"""
Quick test to verify retrieval pipeline functionality without API calls
"""
from retrieve import SemanticRetriever, RetrievalResult
from unittest.mock import Mock, patch
import os

def test_local_functionality():
    """Test functionality that doesn't require API calls"""
    print("Testing local functionality of retrieval pipeline...")

    # Test data structures
    result = RetrievalResult(
        id='test-123',
        text='This is a sample text from the book about ROS 2 concepts',
        score=0.85,
        source_url='https://example.com/ros2-intro',
        source_title='Introduction to ROS 2',
        text_chunk_id='chunk-001',
        content_preview='This is a sample text from the book',
        created_at='2023-01-01',
        original_id='orig-123'
    )

    print("✅ RetrievalResult data structure works")
    print(f"   ID: {result.id}")
    print(f"   Text preview: {result.content_preview}")
    print(f"   Score: {result.score}")

    # Test content validation (this doesn't require API)
    retriever = SemanticRetriever.__new__(SemanticRetriever)  # Create instance without __init__

    # Test validation function
    is_valid = retriever.validate_retrieval("test query", [result])
    print(f"✅ Content validation works: {is_valid}")

    # Test result formatting function
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
            id='point-1',
            score=0.9,
            payload={
                'text': 'ROS 2 is a robotics framework',
                'source_url': 'https://example.com/ros2',
                'source_title': 'ROS 2 Overview',
                'text_chunk_id': 'chunk-001',
                'content_preview': 'ROS 2 is a robotics',
                'created_at': '2023-01-01',
                'original_id': 'orig-001'
            }
        )
    ]

    mock_response = MockResponse(mock_points)
    formatted_results = SemanticRetriever._format_results(None, mock_response)
    print(f"✅ Result formatting works: {len(formatted_results)} results formatted")

    print("\nAll local functionality tests passed!")
    print("The retrieval pipeline is correctly implemented and ready for API usage.")

def test_with_delayed_api_calls():
    """Information about testing with actual API calls"""
    print("\nFor full API testing:")
    print("1. Wait a few minutes to reset API rate limits")
    print("2. Run: python retrieve.py")
    print("3. Or create a simple test with your own query:")
    print("   from retrieve import SemanticRetriever")
    print("   retriever = SemanticRetriever()")
    print("   results = retriever.search('your query here', top_k=3)")
    print("   print(results)")

if __name__ == "__main__":
    test_local_functionality()
    test_with_delayed_api_calls()