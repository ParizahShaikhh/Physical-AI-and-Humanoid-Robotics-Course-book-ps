# Data Model for RAG Agent

## Core Entities

### AgentConfiguration
- `assistant_id`: string - OpenAI Assistant ID
- `model`: string - OpenAI model name (e.g., gpt-4, gpt-3.5-turbo)
- `retrieval_pipeline`: object - Reference to the book content retrieval system
- `temperature`: number - Model temperature setting (0.0 to 1.0)

### RetrievalQuery
- `query_text`: string - Natural language query from user
- `mode`: enum ['full-book', 'selected-text'] - Query mode
- `filters`: object - Optional filters for selected-text mode (e.g., source_url, chapter)
- `top_k`: number - Number of results to retrieve (default: 5)

### RetrievalResult
- `id`: string - Unique identifier for the result
- `text`: string - The retrieved text content
- `score`: number - Relevance score (0.0 to 1.0)
- `source_url`: string - Source URL of the content
- `source_title`: string - Title of the source
- `text_chunk_id`: string - ID of the specific text chunk
- `content_preview`: string - Preview of the content
- `created_at`: string - Timestamp of creation
- `original_id`: string - Original ID reference

### AgentResponse
- `response_text`: string - The generated response
- `sources_used`: array of RetrievalResult - Sources used to generate response
- `confidence_score`: number - Confidence in response accuracy
- `timestamp`: string - When response was generated

## Relationships

- AgentConfiguration contains references to retrieval functions
- RetrievalQuery produces multiple RetrievalResult objects
- AgentResponse is generated using multiple RetrievalResult objects

## Validation Rules

- Query text must be non-empty
- Mode must be either 'full-book' or 'selected-text'
- Top_k must be between 1 and 20
- Temperature must be between 0.0 and 1.0
- Source attribution must be included in responses