"""
Script to ingest book content into Qdrant vector database for RAG system.

This script will:
1. Load book content (from text files, PDFs, etc.)
2. Chunk the content into manageable pieces
3. Generate embeddings using Cohere
4. Store the embeddings in Qdrant with metadata
"""

import os
import asyncio
from typing import List, Dict
import logging
from pathlib import Path

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
import cohere
from dotenv import load_dotenv

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

class BookIngestor:
    def __init__(self):
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

        # Collection name for book embeddings
        self.collection_name = 'book_embeddings'

        logger.info("BookIngestor initialized successfully")

    def chunk_text(self, text: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict]:
        """
        Split text into overlapping chunks for better retrieval.

        Args:
            text: Input text to chunk
            chunk_size: Size of each chunk in characters
            overlap: Overlap between chunks in characters

        Returns:
            List of dictionaries containing text chunks with metadata
        """
        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size
            chunk_text = text[start:end]

            # Create chunk with metadata
            chunk_data = {
                'text': chunk_text,
                'start_pos': start,
                'end_pos': end,
                'chunk_id': f"chunk_{len(chunks)}",
                'source_title': 'Physical AI and Humanoid Robotics Course Book',
                'source_url': 'https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/docs/',
                'section': f'Section_{len(chunks)//10 + 1}',  # Group every 10 chunks into a section
                'page_num': (start // 2000) + 1  # Estimate page number
            }

            chunks.append(chunk_data)
            start = end - overlap  # Move with overlap

        logger.info(f"Text split into {len(chunks)} chunks")
        return chunks

    def embed_chunks(self, chunks: List[Dict]) -> List[List[float]]:
        """
        Generate embeddings for text chunks using Cohere.

        Args:
            chunks: List of text chunks with metadata

        Returns:
            List of embedding vectors
        """
        # Extract just the text parts for embedding
        texts_to_embed = [chunk['text'] for chunk in chunks]

        logger.info(f"Generating embeddings for {len(texts_to_embed)} text chunks...")

        try:
            response = self.cohere_client.embed(
                texts=texts_to_embed,
                model="embed-english-v3.0",
                input_type="search_document"  # Use 'search_document' for documents being searched
            )

            embeddings = response.embeddings
            logger.info(f"Generated {len(embeddings)} embeddings successfully")
            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise

    def create_collection_if_not_exists(self):
        """
        Create the book embeddings collection in Qdrant if it doesn't exist.
        """
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                logger.info(f"Creating collection '{self.collection_name}'...")

                # Create collection with appropriate vector size for Cohere embeddings
                # Cohere's embed-english-v3.0 produces 1024-dimension vectors by default
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
                )

                # Create payload index for faster filtering
                self.qdrant_client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="source_url",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                self.qdrant_client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="source_title",
                    field_schema=models.PayloadSchemaType.TEXT
                )

                logger.info(f"Collection '{self.collection_name}' created successfully")
            else:
                logger.info(f"Collection '{self.collection_name}' already exists")

        except Exception as e:
            logger.error(f"Error creating collection: {str(e)}")
            raise

    def ingest_book_content(self, book_content: str, source_title: str = "Physical AI and Humanoid Robotics Course Book", source_url: str = "https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/docs/"):
        """
        Main method to ingest book content into Qdrant.

        Args:
            book_content: Full text content of the book
            source_title: Title of the source
            source_url: URL of the source
        """
        logger.info("Starting book content ingestion process...")

        # Create collection if it doesn't exist
        self.create_collection_if_not_exists()

        # Chunk the book content
        logger.info("Chunking book content...")
        chunks = self.chunk_text(book_content)

        if not chunks:
            logger.warning("No content to ingest. Book content may be empty.")
            return

        # Update source metadata for all chunks
        for chunk in chunks:
            chunk['source_title'] = source_title
            chunk['source_url'] = source_url

        # Generate embeddings
        logger.info("Generating embeddings...")
        embeddings = self.embed_chunks(chunks)

        # Prepare points for insertion
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point = PointStruct(
                id=i,
                vector=embedding,
                payload={
                    'text': chunk['text'],
                    'source_title': chunk['source_title'],
                    'source_url': chunk['source_url'],
                    'text_chunk_id': chunk['chunk_id'],
                    'section': chunk['section'],
                    'page_num': chunk['page_num'],
                    'content_preview': chunk['text'][:200] + "..." if len(chunk['text']) > 200 else chunk['text'],
                    'original_id': f"book_chunk_{i}",
                    'created_at': '2024-01-01T00:00:00Z'  # Replace with actual timestamp
                }
            )
            points.append(point)

        logger.info(f"Uploading {len(points)} points to Qdrant...")

        # Upload points to Qdrant in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i+batch_size]
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            logger.info(f"Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")

        logger.info(f"Ingestion completed successfully! Uploaded {len(points)} vectors to collection '{self.collection_name}'")

def load_sample_book_content():
    """
    Load sample book content. This is a placeholder that creates sample content
    based on the course title. In a real scenario, this would load from actual book files.
    """
    sample_content = """
Physical AI and Humanoid Robotics Course Book

Chapter 1: Introduction to Physical AI
Physical AI represents a paradigm shift in artificial intelligence, focusing on embodied intelligence that interacts with the physical world. Unlike traditional AI systems that operate primarily in digital domains, Physical AI systems are designed to understand, manipulate, and learn from physical interactions.

The core principles of Physical AI include:
- Embodiment: Intelligence emerges from the interaction between computation and physical form
- Interaction: Learning through physical engagement with the environment
- Adaptation: Ability to adjust behavior based on physical feedback
- Grounding: Understanding rooted in sensory-motor experiences

Chapter 2: Fundamentals of Humanoid Robotics
Humanoid robots are designed to mimic human form and behavior. They represent one of the most challenging areas of robotics due to the complexity of human-like movement and interaction.

Key components of humanoid robots include:
- Actuators: Motors and servos that enable movement
- Sensors: Cameras, tactile sensors, and proprioceptive sensors
- Control systems: Algorithms that coordinate movement and balance
- Cognitive systems: AI that enables perception and decision-making

Balance and locomotion remain among the most difficult challenges in humanoid robotics. Modern approaches utilize:
- Zero Moment Point (ZMP) control
- Capture Point theory
- Model Predictive Control (MPC)
- Machine learning approaches for gait optimization

Chapter 3: ROS 2 for Robot Development
Robot Operating System 2 (ROS 2) provides a flexible framework for developing robot applications. It offers distributed computing, hardware abstraction, and device drivers.

Core concepts in ROS 2 include:
- Nodes: Individual processes that perform computation
- Topics: Named buses over which nodes exchange messages
- Services: Synchronous request/response communication
- Actions: Goal-oriented communication with feedback
- Parameters: Configuration values that can be changed at runtime

Quality of Service (QoS) profiles in ROS 2 allow developers to specify requirements for reliability, durability, and liveliness of communication between nodes.

Chapter 4: NVIDIA Isaac Platform
NVIDIA Isaac is a comprehensive platform for developing, simulating, and deploying AI-powered robots. It combines hardware acceleration with software tools to accelerate robot development.

The Isaac platform includes:
- Isaac Sim: High-fidelity simulation environment
- Isaac ROS: Hardware-accelerated perception and navigation packages
- Isaac Apps: Reference applications demonstrating robot capabilities
- Isaac Lab: Framework for reinforcement learning for robotic manipulation

Isaac Sim enables researchers and developers to train robots in realistic simulated environments before deploying to real hardware, significantly reducing development time and costs.

Chapter 5: Simulation Transfer and Reality Gap
Simulation transfer refers to the challenge of transferring policies learned in simulation to real-world robots. The reality gap—the difference between simulated and real environments—poses significant challenges.

Techniques to address simulation transfer include:
- Domain randomization: Training in diverse simulated environments
- System identification: Accurately modeling real-world dynamics
- Fine-tuning: Adapting simulated policies with limited real-world data
- Sim-to-real algorithms: Methods specifically designed for transfer

Recent advances in sim-to-real transfer have shown promising results, particularly when combined with domain adaptation techniques and meta-learning approaches.

Chapter 6: Advanced Topics in Physical AI
Advanced topics in Physical AI include multimodal learning, where robots learn from multiple sensory modalities simultaneously. This enables more robust and flexible behavior.

Research directions in Physical AI include:
- Interactive learning: Learning through social interaction
- Intrinsically motivated learning: Curiosity-driven exploration
- Morphological computation: Exploiting body properties for computation
- Collective intelligence: Coordination among multiple physical agents

The future of Physical AI lies in creating systems that can learn and adapt continuously in real-world environments, bridging the gap between artificial and natural intelligence.

Appendix A: ROS 2 Installation and Setup
To install ROS 2, follow these steps:
1. Add the ROS 2 apt repository
2. Install ROS 2 packages
3. Source the ROS 2 environment
4. Verify installation with basic tutorials

Appendix B: Isaac Sim Best Practices
Best practices for using Isaac Sim include:
- Starting with provided examples and gradually modifying
- Using USD (Universal Scene Description) for scene composition
- Leveraging GPU acceleration for physics simulation
- Implementing proper logging and debugging workflows
"""
    return sample_content

def main():
    """
    Main function to run the book ingestion process.
    """
    try:
        logger.info("Initializing Book Ingestor...")
        ingestor = BookIngestor()

        # Load book content (in a real scenario, this would come from actual book files)
        logger.info("Loading book content...")
        book_content = load_sample_book_content()

        if not book_content.strip():
            logger.error("No book content found to ingest!")
            return

        logger.info(f"Loaded book content with {len(book_content)} characters")

        # Ingest the content
        ingestor.ingest_book_content(
            book_content=book_content,
            source_title="Physical AI and Humanoid Robotics Course Book",
            source_url="https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/docs/"
        )

        logger.info("Book ingestion completed successfully!")

    except Exception as e:
        logger.error(f"Error during book ingestion: {str(e)}")
        raise

if __name__ == "__main__":
    main()