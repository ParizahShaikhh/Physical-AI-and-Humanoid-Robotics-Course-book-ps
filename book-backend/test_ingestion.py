"""
Test script to verify the ingestion pipeline works correctly
with a smaller set of pages for faster testing.
"""
import os
import time
import logging
from typing import List
from dotenv import load_dotenv
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_ingestion_pipeline():
    """Test the ingestion pipeline with a small subset of pages"""
    logger.info("Starting test ingestion pipeline")

    # Get configuration
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    target_website_url = os.getenv("TARGET_WEBSITE_URL", "https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/")

    # Test with a few specific pages
    test_urls = [
        f"{target_website_url}",
        f"{target_website_url}/docs/intro",
        f"{target_website_url}/docs/module-1-ros2/chapter-1-introduction"
    ]

    logger.info(f"Testing with URLs: {test_urls}")

    # Initialize Cohere client
    cohere_client = cohere.Client(cohere_api_key)

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False
    )

    # Extract content from test pages
    logger.info("Phase 1: Extracting content from test pages")
    extracted_contents = []

    for url in test_urls:
        try:
            logger.info(f"Extracting content from: {url}")
            response = requests.get(url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, 'html.parser')

            # Extract title
            title_tag = soup.find('title')
            title = title_tag.text.strip() if title_tag else "No Title"

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Extract main content
            main_content = soup.find('main') or soup.find('article') or soup
            text_content = main_content.get_text(separator=' ', strip=True)

            # Clean up excessive whitespace
            import re
            text_content = re.sub(r'\s+', ' ', text_content)

            extracted_contents.append({
                'url': url,
                'title': title,
                'content': text_content[:2000]  # Limit content for testing
            })

            time.sleep(0.5)  # Rate limiting

        except Exception as e:
            logger.error(f"Error extracting content from {url}: {str(e)}")
            continue

    logger.info(f"Extracted content from {len(extracted_contents)} pages")

    # Chunk content
    logger.info("Phase 2: Chunking content")
    chunks = []
    chunk_size = 500  # Smaller chunks for testing
    overlap = 50

    for content_obj in extracted_contents:
        text = content_obj['content']
        text_length = len(text)

        start = 0
        chunk_index = 0

        while start < text_length:
            end = start + chunk_size
            if end > text_length:
                end = text_length

            chunk_text = text[start:end]

            chunk = {
                'id': f"test_chunk_{content_obj['url'].replace('/', '_').replace(':', '_')}_{chunk_index}",
                'content': chunk_text,
                'source_url': content_obj['url'],
                'source_title': content_obj['title'],
                'chunk_index': chunk_index
            }

            chunks.append(chunk)
            start = end - overlap
            chunk_index += 1

    logger.info(f"Created {len(chunks)} chunks")

    # Generate embeddings
    logger.info("Phase 3: Generating embeddings")
    embeddings = []

    # Process in small batches
    batch_size = 10  # Small batch for testing

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        texts = [chunk['content'] for chunk in batch]

        try:
            response = cohere_client.embed(
                texts=texts,
                model='embed-english-v3.0',
                input_type='search_document'
            )

            for idx, embedding_vector in enumerate(response.embeddings):
                chunk = batch[idx]

                embedding = {
                    'id': f"test_embedding_{chunk['id']}",
                    'vector': embedding_vector,
                    'source_url': chunk['source_url'],
                    'source_title': chunk['source_title']
                }

                embeddings.append(embedding)

            logger.info(f"Generated embeddings for batch {i//batch_size + 1}")

        except Exception as e:
            logger.error(f"Error generating embeddings for batch: {str(e)}")
            continue

    logger.info(f"Generated {len(embeddings)} embeddings")

    # Store in Qdrant
    logger.info("Phase 4: Storing embeddings in Qdrant")

    points = []
    for embedding in embeddings:
        point = models.PointStruct(
            id=embedding['id'],
            vector=embedding['vector'],
            payload={
                "source_url": embedding['source_url'],
                "source_title": embedding['source_title'],
                "content_preview": embedding['source_title'][:100],
                "created_at": time.strftime('%Y-%m-%dT%H:%M:%SZ')
            }
        )
        points.append(point)

    # Upload points
    qdrant_client.upsert(
        collection_name="book_embeddings",
        points=points
    )

    logger.info(f"Successfully stored {len(embeddings)} embeddings in Qdrant")

    # Verify storage by checking collection info
    collection_info = qdrant_client.get_collection("book_embeddings")
    logger.info(f"Collection now has {collection_info.points_count} total points")

    return collection_info.points_count

if __name__ == "__main__":
    try:
        points_count = test_ingestion_pipeline()
        print(f"\nTest completed successfully! Collection has {points_count} points.")
    except Exception as e:
        logger.error(f"Test failed: {str(e)}")
        raise