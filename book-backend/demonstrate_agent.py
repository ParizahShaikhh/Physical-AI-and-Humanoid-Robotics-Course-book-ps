#!/usr/bin/env python3
"""
Demonstration script for the RAG Agent.
This script shows the key features of the implemented agent.
"""

import os
from agent import RAGAgent

def demo_features():
    """Demonstrate the key features of the RAG Agent."""
    print("📚 RAG Agent Feature Demonstration")
    print("=" * 50)

    # Check if environment variables are set
    if not os.getenv("OPENAI_API_KEY"):
        print("⚠️  Warning: OPENAI_API_KEY environment variable not found.")
        print("   Please set it to run the full demonstration.")
        print()
        return

    try:
        print("🔧 Initializing RAG Agent...")
        agent = RAGAgent(model="gpt-3.5-turbo", enable_caching=True, cache_ttl=3600)
        print("✅ Agent initialized successfully!")
        print()

        print("✨ Features Demonstrated:")
        print("  • Query parameter validation")
        print("  • Caching for performance")
        print("  • Confidence scoring")
        print("  • Source attribution")
        print("  • Rate limiting protection")
        print("  • Error handling")
        print()

        print("💡 Example Query: 'What is ROS 2?'")
        print("-" * 30)

        # Example query
        response = agent.ask("What is ROS 2?", mode="full-book", top_k=3)
        print(f"Response:\n{response}")
        print()

        print("💡 Same query again (should use cache if enabled):")
        print("-" * 30)

        # Same query again to demonstrate caching
        response2 = agent.ask("What is ROS 2?", mode="full-book", top_k=3)
        print(f"Response:\n{response2}")
        print()

        print("🎯 Selected-text mode example:")
        print("-" * 30)

        # Example with filters (would work if specific URLs were known)
        response3 = agent.ask("Explain physical AI concepts",
                             mode="selected-text",
                             filters={"source_url": "https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/"})
        print(f"Response:\n{response3}")
        print()

        print("🔒 Parameter validation examples:")
        print("-" * 30)

        # Demonstrate parameter validation
        try:
            # This should raise an error due to empty query
            agent.search_book_content("", mode="full-book", top_k=5)
        except ValueError as e:
            print(f"✅ Parameter validation caught error: {e}")

        try:
            # This should raise an error due to invalid mode
            agent.search_book_content("test", mode="invalid-mode", top_k=5)
        except ValueError as e:
            print(f"✅ Parameter validation caught error: {e}")

        print()
        print("🏆 Agent features summary:")
        print("  ✓ Full-book and selected-text query modes")
        print("  ✓ Content retrieval from Qdrant vector store")
        print("  ✓ Response generation with OpenAI")
        print("  ✓ Confidence scoring based on relevance")
        print("  ✓ Source attribution for transparency")
        print("  ✓ Caching for improved performance")
        print("  ✓ Rate limiting to respect API quotas")
        print("  ✓ Input validation and error handling")
        print("  ✓ Graceful degradation when APIs are unavailable")

    except Exception as e:
        print(f"❌ Error during demonstration: {e}")
        print("Make sure you have:")
        print("  - Valid OPENAI_API_KEY in environment variables")
        print("  - Access to the Qdrant vector store")
        print("  - Required packages installed")

if __name__ == "__main__":
    demo_features()