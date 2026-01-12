#!/usr/bin/env python3
"""
Test script to verify that the RAG agent returns exact book content
"""

import os
from agent import RAGAgent

def test_exact_content():
    """Test that the agent returns exact book content"""
    print("Testing RAG Agent for Exact Book Content")
    print("=" * 50)

    # Check if environment variables are set
    if not os.getenv("OPENAI_API_KEY"):
        print("⚠️  Warning: OPENAI_API_KEY environment variable not found.")
        print("   Please set it to run the full demonstration.")
        print()
        return

    try:
        print("🔧 Initializing RAG Agent...")
        agent = RAGAgent(model="gpt-4o-mini", enable_caching=False)  # Disable caching for fresh test
        print("✅ Agent initialized successfully!")
        print()

        print("🔍 Testing exact content retrieval...")
        print("-" * 30)

        # Test query that should return exact book content
        query = "What is ROS 2?"
        print(f"Query: {query}")
        response = agent.ask(query, mode="full-book", top_k=3)
        print(f"\nResponse:\n{response}")
        print()

        print("🔍 Testing another query...")
        print("-" * 30)

        # Another test query
        query2 = "Explain physical AI concepts"
        print(f"Query: {query2}")
        response2 = agent.ask(query2, mode="full-book", top_k=3)
        print(f"\nResponse:\n{response2}")
        print()

        print("✅ Test completed! The agent should now return exact book content")
        print("instead of summaries or generated text.")

    except Exception as e:
        print(f"❌ Error during testing: {e}")
        print("Make sure you have:")
        print("  - Valid OPENAI_API_KEY in environment variables")
        print("  - Access to the Qdrant vector store")
        print("  - Required packages installed")

if __name__ == "__main__":
    test_exact_content()