"""
Book Content Ingestion Pipeline

This script implements a complete pipeline for:
1. Crawling a Docusaurus book website
2. Extracting content from web pages
3. Chunking text for embedding
4. Generating embeddings with Cohere
5. Storing vectors in Qdrant Cloud
"""
import os
import time
import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from urllib.parse import urljoin, urlparse
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


@dataclass
class WebsiteContent:
    """Represents content extracted from a website page"""
    url: str
    title: str
    content: str
    metadata: Dict[str, Any]
    created_at: str


@dataclass
class TextChunk:
    """Represents a chunk of text prepared for embedding"""
    id: str
    content: str
    source_url: str
    source_title: str
    chunk_index: int
    created_at: str


@dataclass
class Embedding:
    """Represents a vector embedding of text content"""
    id: str
    vector: List[float]
    text_chunk_id: str
    source_url: str
    source_title: str
    created_at: str
    text_content: str = ""  # Store the actual text content


class WebCrawler:
    """Handles crawling and content extraction from the target website"""

    def __init__(self, base_url: str, delay: float = 1.0):
        self.base_url = base_url
        self.delay = delay
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; BookIngestionBot/1.0)'
        })
        self.visited_urls = set()
        self.discovered_urls = set()

    def discover_urls(self) -> List[str]:
        """Discover all URLs on the target website"""
        logger.info(f"Discovering URLs on {self.base_url}")
        self.discovered_urls = set()

        # First, crawl the website to find links
        self._crawl_recursive(self.base_url)

        # Then, also check the sitemap for additional URLs
        self._discover_from_sitemap()

        return list(self.discovered_urls)

    def _discover_from_sitemap(self):
        """Discover URLs from the website's sitemap"""
        sitemap_url = urljoin(self.base_url, "sitemap.xml")
        logger.info(f"Checking sitemap at {sitemap_url}")

        try:
            response = self.session.get(sitemap_url, timeout=10)
            response.raise_for_status()

            from xml.etree import ElementTree as ET
            root = ET.fromstring(response.content)

            # Handle both regular sitemap and sitemap index
            if root.tag.endswith('sitemapindex'):
                # This is a sitemap index, need to fetch individual sitemaps
                for sitemap_elem in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}sitemap/{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
                    sitemap_loc = sitemap_elem.text
                    self._parse_individual_sitemap(sitemap_loc)
            else:
                # This is a regular sitemap with URLs
                self._parse_sitemap_urls(root)

        except Exception as e:
            logger.warning(f"Could not fetch or parse sitemap: {str(e)}")
            logger.info("Continuing with URLs found through crawling only")

    def _parse_individual_sitemap(self, sitemap_url):
        """Parse an individual sitemap from a sitemap index"""
        try:
            response = self.session.get(sitemap_url, timeout=10)
            response.raise_for_status()

            from xml.etree import ElementTree as ET
            root = ET.fromstring(response.content)
            self._parse_sitemap_urls(root)

        except Exception as e:
            logger.warning(f"Could not fetch or parse sitemap {sitemap_url}: {str(e)}")

    def _parse_sitemap_urls(self, root):
        """Parse URLs from a sitemap XML"""
        for url_elem in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url/{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
            url = url_elem.text.strip()
            if self._is_same_domain(url):
                self.discovered_urls.add(url)

    def _crawl_recursive(self, url: str, depth: int = 0, max_depth: int = 5):
        """Recursively crawl the website to discover URLs"""
        if depth > max_depth or url in self.visited_urls:
            return

        self.visited_urls.add(url)
        logger.info(f"Crawling: {url}")

        try:
            response = self.session.get(url, timeout=10)
            response.raise_for_status()

            # Add the URL to discovered URLs
            self.discovered_urls.add(url)

            # Parse HTML and find links
            soup = BeautifulSoup(response.text, 'html.parser')

            # Find all links
            for link in soup.find_all('a', href=True):
                href = link['href']
                full_url = urljoin(url, href)

                # Only process URLs from the same domain
                if self._is_same_domain(full_url):
                    if full_url not in self.visited_urls and full_url not in self.discovered_urls:
                        time.sleep(self.delay)  # Rate limiting
                        self._crawl_recursive(full_url, depth + 1, max_depth)

        except Exception as e:
            logger.error(f"Error crawling {url}: {str(e)}")

    def _is_same_domain(self, url: str) -> bool:
        """Check if the URL is from the same domain as the base URL"""
        base_domain = urlparse(self.base_url).netloc
        url_domain = urlparse(url).netloc
        return base_domain == url_domain


class ContentExtractor:
    """Extracts clean text content from HTML pages"""

    def extract_content(self, url: str) -> Optional[WebsiteContent]:
        """Extract content from a single URL"""
        logger.info(f"Extracting content from: {url}")

        try:
            response = requests.get(url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, 'html.parser')

            # Extract title
            title = soup.find('title')
            title = title.text.strip() if title else "No Title"

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Extract main content - focusing on content areas in Docusaurus
            main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='container') or soup

            # Get text content
            text_content = main_content.get_text(separator=' ', strip=True)

            # Clean up excessive whitespace
            import re
            text_content = re.sub(r'\s+', ' ', text_content)

            # Extract metadata
            metadata = self._extract_metadata(soup)

            return WebsiteContent(
                url=url,
                title=title,
                content=text_content,
                metadata=metadata,
                created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ')
            )

        except Exception as e:
            logger.error(f"Error extracting content from {url}: {str(e)}")
            return None

    def _extract_metadata(self, soup: BeautifulSoup) -> Dict[str, Any]:
        """Extract metadata from the page"""
        metadata = {}

        # Extract description
        desc_tag = soup.find('meta', attrs={'name': 'description'})
        if desc_tag and desc_tag.get('content'):
            metadata['description'] = desc_tag['content']

        # Extract keywords
        keywords_tag = soup.find('meta', attrs={'name': 'keywords'})
        if keywords_tag and keywords_tag.get('content'):
            metadata['keywords'] = [k.strip() for k in keywords_tag['content'].split(',')]

        # Extract last modified
        modified_tag = soup.find('meta', attrs={'name': 'last-modified'})
        if modified_tag and modified_tag.get('content'):
            metadata['last_modified'] = modified_tag['content']

        return metadata


class TextChunker:
    """Handles chunking of text content for embedding"""

    def __init__(self, chunk_size: int = 1500, overlap: int = 200):
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk_content(self, content: WebsiteContent) -> List[TextChunk]:
        """Chunk the content into appropriate sizes"""
        logger.info(f"Chunking content from {content.url}")

        chunks = []
        text = content.content

        # Split text into sentences to avoid breaking in the middle of sentences
        import re
        sentences = re.split(r'(?<=[.!?]) +', text)

        current_chunk = ""
        chunk_index = 0

        for sentence in sentences:
            # Check if adding this sentence would exceed chunk size
            if len(current_chunk) + len(sentence) <= self.chunk_size:
                current_chunk += sentence + " "
            else:
                # If the current chunk is empty but the sentence is too long,
                # we need to split the sentence
                if len(current_chunk) == 0 and len(sentence) > self.chunk_size:
                    # Split the long sentence into smaller pieces
                    for i in range(0, len(sentence), self.chunk_size):
                        piece = sentence[i:i + self.chunk_size]
                        chunk = TextChunk(
                            id=f"chunk_{content.url.replace('/', '_').replace(':', '_')}_{chunk_index}",
                            content=piece,
                            source_url=content.url,
                            source_title=content.title,
                            chunk_index=chunk_index,
                            created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ')
                        )
                        chunks.append(chunk)
                        chunk_index += 1
                else:
                    # Save the current chunk and start a new one
                    if current_chunk.strip():
                        chunk = TextChunk(
                            id=f"chunk_{content.url.replace('/', '_').replace(':', '_')}_{chunk_index}",
                            content=current_chunk.strip(),
                            source_url=content.url,
                            source_title=content.title,
                            chunk_index=chunk_index,
                            created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ')
                        )
                        chunks.append(chunk)
                        chunk_index += 1

                    # Start new chunk with the current sentence
                    current_chunk = sentence + " "

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunk = TextChunk(
                id=f"chunk_{content.url.replace('/', '_').replace(':', '_')}_{chunk_index}",
                content=current_chunk.strip(),
                source_url=content.url,
                source_title=content.title,
                chunk_index=chunk_index,
                created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ')
            )
            chunks.append(chunk)

        logger.info(f"Created {len(chunks)} chunks from {content.url}")
        return chunks


class EmbeddingGenerator:
    """Generates embeddings using Cohere API"""

    def __init__(self, api_key: str):
        self.client = cohere.Client(api_key)

    def generate_embeddings(self, chunks: List[TextChunk]) -> List[Embedding]:
        """Generate embeddings for a list of text chunks"""
        logger.info(f"Generating embeddings for {len(chunks)} chunks")

        embeddings = []

        # Process in batches to respect API limits
        batch_size = 20  # Reduced batch size to avoid rate limits
        delay_between_batches = 1.0  # Add delay between batches

        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            texts = [chunk.content for chunk in batch]

            try:
                response = self.client.embed(
                    texts=texts,
                    model='embed-english-v3.0',
                    input_type='search_document'
                )

                for idx, embedding_vector in enumerate(response.embeddings):
                    chunk = batch[idx]

                    embedding = Embedding(
                        id=f"embedding_{chunk.id}",
                        vector=embedding_vector,
                        text_chunk_id=chunk.id,
                        source_url=chunk.source_url,
                        source_title=chunk.source_title,
                        created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ'),
                        text_content=chunk.content  # Store the actual text content
                    )

                    embeddings.append(embedding)

            except Exception as e:
                # Check if it's a Cohere rate limit error
                if "429" in str(e) or "TooManyRequests" in str(e) or "rate limit" in str(e).lower():
                    logger.warning(f"Rate limit hit, waiting before retry: {str(e)}")
                    time.sleep(10)  # Wait longer for rate limit
                    # Try again once after waiting
                    try:
                        response = self.client.embed(
                            texts=texts,
                            model='embed-english-v3.0',
                            input_type='search_document'
                        )

                        for idx, embedding_vector in enumerate(response.embeddings):
                            chunk = batch[idx]

                            embedding = Embedding(
                                id=f"embedding_{chunk.id}",
                                vector=embedding_vector,
                                text_chunk_id=chunk.id,
                                source_url=chunk.source_url,
                                source_title=chunk.source_title,
                                created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ')
                            )

                            embeddings.append(embedding)
                    except Exception as retry_error:
                        logger.error(f"Error generating embeddings for batch after retry: {str(retry_error)}")
                        continue
                else:
                    logger.error(f"Cohere API error: {str(e)}")
                    continue

            # Add delay between batches to avoid rate limits
            if i + batch_size < len(chunks):
                time.sleep(delay_between_batches)

        logger.info(f"Generated {len(embeddings)} embeddings")
        return embeddings


class VectorStorage:
    """Handles storage of embeddings in Qdrant Cloud"""

    def __init__(self, url: str, api_key: str, collection_name: str = "book_embeddings"):
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            prefer_grpc=False
        )
        self.collection_name = collection_name

        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """Create the vector collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except Exception:
            # Create collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1024,  # Cohere v3 model returns 1024-dim vectors
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection {self.collection_name}")

    def store_embeddings(self, embeddings: List[Embedding]):
        """Store embeddings in Qdrant Cloud"""
        logger.info(f"Storing {len(embeddings)} embeddings in Qdrant")

        points = []

        for embedding in embeddings:
            # Create a valid Qdrant ID by using a hash of the original ID
            import hashlib
            point_id = hashlib.md5(embedding.id.encode()).hexdigest()

            point = models.PointStruct(
                id=point_id,
                vector=embedding.vector,
                payload={
                    "source_url": embedding.source_url,
                    "source_title": embedding.source_title,
                    "text_chunk_id": embedding.text_chunk_id,
                    "content_preview": embedding.text_content[:200] if embedding.text_content else embedding.source_title[:100],  # First 200 chars of actual content as preview
                    "text": embedding.text_content,  # Store the full text content
                    "created_at": embedding.created_at,
                    "original_id": embedding.id  # Keep original ID in payload for reference
                }
            )
            points.append(point)

        # Upload points in batches
        batch_size = 32  # Reduced batch size to avoid timeout issues
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            try:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=batch
                )
                logger.info(f"Uploaded batch of {len(batch)} points to Qdrant")
            except Exception as e:
                logger.error(f"Error uploading batch to Qdrant: {str(e)}")
                # Try uploading individual points if batch upload fails
                for point in batch:
                    try:
                        self.client.upsert(
                            collection_name=self.collection_name,
                            points=[point]
                        )
                        logger.info(f"Uploaded individual point {point.id} to Qdrant")
                    except Exception as individual_error:
                        logger.error(f"Failed to upload individual point {point.id}: {str(individual_error)}")
                        continue
                continue

        logger.info(f"Successfully stored embeddings in Qdrant")


def main():
    """Main function to orchestrate the entire ingestion pipeline"""
    logger.info("Starting book content ingestion pipeline")

    # Get configuration from environment
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    target_website_url = os.getenv("TARGET_WEBSITE_URL", "https://example.com")
    rate_limit_delay = float(os.getenv("RATE_LIMIT_DELAY", "1.0"))
    chunk_size = int(os.getenv("CHUNK_SIZE", "1500"))

    # Validate configuration
    if not all([cohere_api_key, qdrant_api_key, qdrant_url, target_website_url]):
        raise ValueError("Missing required environment variables")

    # Initialize components
    crawler = WebCrawler(target_website_url, delay=rate_limit_delay)
    extractor = ContentExtractor()
    chunker = TextChunker(chunk_size=chunk_size)
    embedder = EmbeddingGenerator(cohere_api_key)
    storage = VectorStorage(qdrant_url, qdrant_api_key)

    try:
        # Phase 1: Discover URLs
        logger.info("Phase 1: Discovering URLs")
        urls = crawler.discover_urls()
        logger.info(f"Discovered {len(urls)} URLs")

        # Phase 2: Extract content
        logger.info("Phase 2: Extracting content")
        all_contents = []
        for idx, url in enumerate(urls):
            content = extractor.extract_content(url)
            if content:
                all_contents.append(content)
            time.sleep(rate_limit_delay)  # Rate limiting

            # Memory management: Clear memory periodically
            if (idx + 1) % 10 == 0:
                import gc
                gc.collect()
                logger.info(f"Processed {idx + 1}/{len(urls)} URLs, memory cleared")

        logger.info(f"Extracted content from {len(all_contents)} pages")

        # Phase 3: Chunk content
        logger.info("Phase 3: Chunking content")
        all_chunks = []
        for idx, content in enumerate(all_contents):
            chunks = chunker.chunk_content(content)
            all_chunks.extend(chunks)

            # Memory management: Clear memory periodically
            if (idx + 1) % 10 == 0:
                import gc
                gc.collect()
                logger.info(f"Chunked {idx + 1}/{len(all_contents)} contents, memory cleared")

        logger.info(f"Created {len(all_chunks)} text chunks")

        # Phase 4: Generate embeddings
        logger.info("Phase 4: Generating embeddings")
        embeddings = embedder.generate_embeddings(all_chunks)
        logger.info(f"Generated {len(embeddings)} embeddings")

        # Memory management before storing
        import gc
        gc.collect()
        logger.info("Memory cleared after embedding generation")

        # Phase 5: Store in vector database
        logger.info("Phase 5: Storing embeddings in Qdrant")
        storage.store_embeddings(embeddings)

        logger.info("Ingestion pipeline completed successfully!")

    except Exception as e:
        logger.error(f"Error in ingestion pipeline: {str(e)}")
        raise


if __name__ == "__main__":
    main()