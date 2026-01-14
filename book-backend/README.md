---
title: Book Assistant
emoji: 📚
colorFrom: blue
colorTo: yellow
sdk: docker
pinned: false
license: apache-2.0
short_description: RAG Chatbot for Physical AI and Humanoid Robotics Course
---

# Book Assistant
Ask questions about the book content and get answers grounded in the text.

## Setup Instructions

This RAG (Retrieval-Augmented Generation) chatbot requires several API keys to function properly:

### Required Environment Variables (as Secrets in Space Settings):

1. `OPENROUTER_API_KEY` - API key for OpenRouter (for LLM responses)
2. `COHERE_API_KEY` - API key for Cohere (for text embeddings)
3. `QDRANT_URL` - URL for your Qdrant vector database
4. `QDRANT_API_KEY` - API key for your Qdrant database

### Before Using the Chatbot:

The book content needs to be ingested into the Qdrant vector database. If you have access to the Space's terminal, run:

```bash
python ingest_books.py
```

This will:
- Create the 'book_embeddings' collection in Qdrant
- Chunk the book content into searchable pieces
- Generate embeddings using Cohere
- Store the embeddings in Qdrant with metadata

### Features:
- Full-book query mode: Search across the entire book content
- Context-aware responses grounded in the source material
- Source attribution for all responses
- Mobile-friendly interface

### Technology Stack:
- FastAPI backend
- Cohere for embeddings
- Qdrant for vector storage
- OpenRouter for LLM responses
- Semantic search capabilities
