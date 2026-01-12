"""
RAG Agent with OpenRouter API

This agent connects to the Qdrant-backed retrieval pipeline to answer questions
about book content. It supports both full-book and selected-text retrieval modes
and generates responses strictly grounded in the retrieved content.
"""
import os
import json
import logging
from typing import List, Dict, Optional, Any
from dataclasses import dataclass
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import OpenAI compatible library for OpenRouter
try:
    from openai import OpenAI
    OPENROUTER_AVAILABLE = True
except ImportError:
    OPENROUTER_AVAILABLE = False
    print("OpenAI library not found. Install with: pip install openai")

# Import the retrieval pipeline from the same directory
try:
    from retrieve import SemanticRetriever
except ImportError:
    print("retrieve module not found. Make sure retrieve.py is in the same directory")


@dataclass
class RetrievalResult:
    """Data structure for retrieval results"""
    id: str
    text: str
    score: float
    source_url: str
    source_title: str
    text_chunk_id: str
    content_preview: str
    created_at: str
    original_id: str


class RAGAgent:
    """
    RAG Agent that connects to Qdrant-backed retrieval pipeline
    and uses OpenRouter to generate grounded responses.
    """

    def __init__(self, model: str = "mistralai/mistral-7b-instruct:free", enable_caching: bool = True, cache_ttl: int = 3600):
        """
        Initialize the RAG Agent.

        Args:
            model: OpenRouter model to use for response generation
            enable_caching: Whether to enable caching of responses
            cache_ttl: Cache time-to-live in seconds
        """
        if not OPENROUTER_AVAILABLE:
            raise ImportError("OpenAI library is required but not installed")

        # Initialize OpenRouter client
        api_key = os.getenv("OPENROUTER_API_KEY") or os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENROUTER_API_KEY or OPENAI_API_KEY environment variable not found")

        # Initialize with explicit HTTP client to avoid proxy issues in cloud environments
        import httpx
        http_client = httpx.Client(
            timeout=60.0,  # Increase timeout for cloud environments
        )

        self.client = OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1",
            http_client=http_client
        )
        self.model = model

        # Initialize the retrieval pipeline
        self.retriever = SemanticRetriever()

        # Initialize caching
        self.enable_caching = enable_caching
        self.cache_ttl = cache_ttl
        self.response_cache = {}  # Simple in-memory cache

        # Initialize rate limiting
        self.rate_limit_enabled = True
        self.max_requests_per_minute = 30  # Adjust based on API limits
        self.request_timestamps = []

        # Initialize batching settings
        self.use_batching = True
        self.batch_size = 20  # Maximum number of items to process in a batch

        # Set up logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

        self.logger.info("RAG Agent initialized successfully with OpenRouter")

    def _check_rate_limit(self) -> bool:
        """
        Check if the current request would exceed the rate limit.

        Returns:
            True if request is within rate limit, False otherwise
        """
        import time
        if not self.rate_limit_enabled:
            return True

        current_time = time.time()

        # Remove timestamps older than 1 minute
        self.request_timestamps = [ts for ts in self.request_timestamps if current_time - ts < 60]

        # Check if we're under the limit
        if len(self.request_timestamps) < self.max_requests_per_minute:
            self.request_timestamps.append(current_time)
            return True

        # Rate limit exceeded
        return False

    def _wait_for_rate_limit(self):
        """
        Wait until the rate limit allows another request.
        """
        import time
        while not self._check_rate_limit():
            time.sleep(1)  # Wait 1 second before checking again

    def search_book_content(
        self,
        query: str,
        mode: str = "full-book",
        filters: Optional[Dict[str, str]] = None,
        top_k: int = 5
    ) -> List[RetrievalResult]:
        """
        Search the book content using the retrieval pipeline.

        Args:
            query: Natural language query text
            mode: Query mode - 'full-book' or 'selected-text'
            filters: Additional filters for selected-text mode
            top_k: Number of results to return

        Returns:
            List of retrieval results
        """
        # Validate query parameters
        if not query or not query.strip():
            raise ValueError("Query cannot be empty or whitespace only")

        if top_k <= 0 or top_k > 20:
            raise ValueError("top_k must be between 1 and 20")

        if mode not in ["full-book", "selected-text"]:
            raise ValueError("Mode must be either 'full-book' or 'selected-text'")

        self.logger.info(f"Searching book content for query: '{query[:50]}...' in mode: {mode}")

        # Apply filters based on mode
        search_filters = filters if filters else {}

        # For selected-text mode, we might want to apply specific filters
        if mode == "selected-text" and not search_filters:
            # If no specific filters provided for selected-text mode,
            # we could apply broader filters or use the query differently
            pass

        try:
            results = self.retriever.search(query, top_k=top_k, filters=search_filters)
            self.logger.info(f"Retrieved {len(results)} results for query")
            return results
        except Exception as e:
            self.logger.error(f"Error during content search: {str(e)}")
            raise

    def generate_response(self, query: str, retrieved_content: List[RetrievalResult]) -> str:
        """
        Generate a response based on the query and retrieved content using OpenRouter API.

        Args:
            query: Original user query
            retrieved_content: List of retrieved content objects

        Returns:
            Generated response grounded in retrieved content
        """
        self.logger.info(f"Generating response for query: '{query[:50]}...'")

        # Calculate confidence score based on average of retrieved content scores
        if retrieved_content:
            avg_score = sum([result.score for result in retrieved_content]) / len(retrieved_content)
            confidence_level = self._get_confidence_level(avg_score)
        else:
            avg_score = 0.0
            confidence_level = "Low"

        # Format the retrieved content for context
        context_str = ""
        sources = []

        for i, result in enumerate(retrieved_content, 1):
            context_str += f"\n--- Content {i} ---\n"
            context_str += f"Source: {result.source_title}\n"
            context_str += f"URL: {result.source_url}\n"
            context_str += f"Content: {result.text}\n"
            context_str += f"Score: {result.score}\n"
            context_str += "---\n"

            sources.append(f"{result.source_title} ({result.source_url})")

        # Create system message to ground the response in content
        system_message = """You are a helpful assistant that answers questions based strictly on the provided book content.
        Your responses must be grounded only in the retrieved content provided.
        Return the exact text from the book that answers the question.
        Do not summarize, paraphrase, or generate new content.
        Always cite sources when possible."""

        # Create user message with query and context
        user_message = f"""
        Question: {query}

        Retrieved Content:
        {context_str}

        Please return the EXACT text from the book that answers the question.
        Do not summarize, paraphrase, or add your own text.
        Return the precise content from the book that addresses the question.
        If the content doesn't contain the information needed to answer the question,
        please state that the information is not available in the provided content.
        """

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.1  # Lower temperature for more consistent, fact-based responses
            )

            generated_response = response.choices[0].message.content
            self.logger.info("Response generated successfully")

            # Add source attribution and confidence to the response
            if sources:
                generated_response += f"\n\nSources referenced: {', '.join(sources[:3])}"  # Limit to first 3 sources
            generated_response += f"\n\nConfidence Level: {confidence_level} (Average Score: {avg_score:.2f})"

            return generated_response

        except Exception as e:
            self.logger.error(f"Error generating response: {str(e)}")
            raise

    def _get_confidence_level(self, score: float) -> str:
        """
        Convert numerical score to confidence level.

        Args:
            score: Numerical score between 0.0 and 1.0

        Returns:
            Confidence level as string
        """
        if score >= 0.8:
            return "High"
        elif score >= 0.6:
            return "Medium"
        elif score >= 0.4:
            return "Low"
        else:
            return "Very Low"

    def validate_response(self, response: str, content: List[RetrievalResult]) -> Dict[str, Any]:
        """
        Validate that the response is grounded in the provided content.

        Args:
            response: Generated response
            content: Content used to generate response

        Returns:
            Validation results
        """
        # Simple validation: check if response mentions content from sources
        content_text = " ".join([c.text for c in content])
        response_lower = response.lower()
        content_lower = content_text.lower()

        # Count how much of the response is supported by the content
        words_in_response = len(response_lower.split())
        supported_words = 0

        for word in response_lower.split():
            if word in content_lower:
                supported_words += 1

        support_ratio = supported_words / words_in_response if words_in_response > 0 else 0

        return {
            "is_valid": support_ratio > 0.5,  # At least 50% of content should be supported
            "support_ratio": support_ratio,
            "issues": [] if support_ratio > 0.5 else ["Response contains information not supported by provided content"]
        }

    def format_response(self, response: str, sources: List[RetrievalResult]) -> str:
        """
        Format the response with proper attribution and structure.

        Args:
            response: Generated response
            sources: Source objects to attribute

        Returns:
            Properly formatted response with attribution
        """
        # The response is already formatted with sources in generate_response
        # This method can be used for additional formatting if needed
        return response

    def ask(
        self,
        query: str,
        mode: str = "full-book",
        filters: Optional[Dict[str, str]] = None,
        top_k: int = 5
    ) -> str:
        """
        Main method to ask a question and get a response.

        Args:
            query: Natural language question
            mode: Query mode - 'full-book' or 'selected-text'
            filters: Additional filters for selected-text mode
            top_k: Number of results to retrieve

        Returns:
            Answer to the question based on book content
        """
        self.logger.info(f"Processing query: '{query}' in mode: {mode}")

        # Check rate limit
        if not self._check_rate_limit():
            self.logger.warning("Rate limit exceeded, waiting...")
            self._wait_for_rate_limit()

        # Create cache key
        cache_key = self._create_cache_key(query, mode, filters, top_k)

        # Check cache if enabled
        if self.enable_caching:
            cached_response = self._get_cached_response(cache_key)
            if cached_response:
                self.logger.info("Returning cached response")
                return cached_response

        try:
            # Step 1: Retrieve relevant content
            retrieved_content = self.search_book_content(query, mode, filters, top_k)

            if not retrieved_content:
                response = "I couldn't find any relevant content in the book to answer your question."
            else:
                # Step 2: Generate response based on retrieved content
                response = self.generate_response(query, retrieved_content)

                # Step 3: Validate response is grounded in content
                validation = self.validate_response(response, retrieved_content)

                if not validation["is_valid"]:
                    self.logger.warning(f"Response validation failed: {validation['issues']}")
                    response = "I found relevant content but couldn't generate a properly grounded response. Please try rephrasing your question."

            # Step 4: Format and return response
            formatted_response = self.format_response(response, retrieved_content)

            # Store in cache if enabled
            if self.enable_caching:
                self._cache_response(cache_key, formatted_response)

            self.logger.info("Query processed successfully")
            return formatted_response

        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            return f"Sorry, I encountered an error processing your query: {str(e)}"

    def _create_cache_key(self, query: str, mode: str, filters: Optional[Dict[str, str]], top_k: int) -> str:
        """
        Create a cache key from query parameters.

        Args:
            query: Query string
            mode: Query mode
            filters: Filters applied
            top_k: Number of results

        Returns:
            Hashed cache key string
        """
        import hashlib
        cache_params = f"{query}_{mode}_{str(filters)}_{top_k}"
        return hashlib.md5(cache_params.encode()).hexdigest()

    def _get_cached_response(self, cache_key: str) -> Optional[str]:
        """
        Get response from cache if it exists and hasn't expired.

        Args:
            cache_key: Cache key to look up

        Returns:
            Cached response if found and valid, None otherwise
        """
        import time
        if cache_key in self.response_cache:
            cached_item = self.response_cache[cache_key]
            timestamp = cached_item.get("timestamp", 0)

            # Check if cache has expired
            if time.time() - timestamp < self.cache_ttl:
                return cached_item.get("response")
            else:
                # Remove expired entry
                del self.response_cache[cache_key]
        return None

    def _cache_response(self, cache_key: str, response: str) -> None:
        """
        Store response in cache.

        Args:
            cache_key: Cache key to store under
            response: Response to cache
        """
        import time
        self.response_cache[cache_key] = {
            "response": response,
            "timestamp": time.time()
        }


def main():
    """
    Main function to demonstrate the RAG Agent usage.
    """
    print("Initializing RAG Agent with OpenRouter...")

    try:
        agent = RAGAgent(enable_caching=True, cache_ttl=3600)  # Enable caching with 1-hour TTL
        print("RAG Agent initialized successfully!")
        print("\nYou can now ask questions about the book content.")
        print("Examples:")
        print("  - 'Explain ROS 2 concepts'")
        print("  - 'What is physical AI?'")
        print("  - 'How does simulation transfer work?'")
        print("\nType 'quit' to exit.\n")

        while True:
            query = input("Your question: ").strip()

            if query.lower() in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break

            if not query:
                continue

            print("\nSearching for answer...")
            response = agent.ask(query)
            print(f"\nAnswer: {response}\n")

    except KeyboardInterrupt:
        print("\nGoodbye!")
    except Exception as e:
        print(f"Error initializing agent: {e}")
        print("Make sure you have:")
        print("  - OpenRouter API key in environment variables (OPENROUTER_API_KEY or OPENAI_API_KEY)")
        print("  - Properly configured book-backend with retrieval pipeline")
        print("  - Required packages installed (openai)")
        print("  - Internet connection to access OpenRouter API")


if __name__ == "__main__":
    main()