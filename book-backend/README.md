# Book Content Ingestion and Retrieval Pipeline

This project implements a complete pipeline for processing book content from a Docusaurus website, including both ingestion and retrieval components.

- **Ingestion**: Crawls a Docusaurus book website, extracts content, generates embeddings using Cohere, and stores them in Qdrant Cloud
- **Retrieval**: Provides semantic search capabilities over the stored embeddings using natural language queries

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Create a `.env` file with your API keys:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url
   TARGET_WEBSITE_URL=https://your-docusaurus-site.example.com
   ```

## Usage

### Ingestion Pipeline
Run the ingestion pipeline to crawl, embed, and store book content:
```bash
python main.py
```

### Retrieval Pipeline
Use the retrieval pipeline to perform semantic searches:
```bash
python retrieve.py
```

Or integrate the SemanticRetriever class in your own code:
```python
from retrieve import SemanticRetriever

retriever = SemanticRetriever()
results = retriever.search("your natural language query", top_k=5)
```

## Components

- `main.py`: The ingestion pipeline that crawls, extracts, chunks, embeds, and stores content
- `retrieve.py`: The retrieval pipeline that performs semantic search and filtering
- `test_ingestion.py`: Tests for the ingestion pipeline
- `test_retrieval.py`: Tests for the retrieval pipeline