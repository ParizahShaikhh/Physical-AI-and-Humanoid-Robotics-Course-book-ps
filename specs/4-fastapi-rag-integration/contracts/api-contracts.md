# API Contracts: FastAPI Integration Between Frontend and RAG Backend

## Overview
This document defines the API contracts for the FastAPI integration between the frontend and RAG backend system. All endpoints follow RESTful principles and use JSON for request/response payloads.

## Base URL
- Development: `http://localhost:8000`
- Production: `[configured via environment]`

## Common Headers
- `Content-Type: application/json`
- `Accept: application/json`

## Authentication
No authentication required for initial implementation (may be added in future iterations)

## API Endpoints

### Chat Query Endpoint

#### `POST /api/chat`
Process a general book content query through the RAG agent.

##### Request
```json
{
  "query": "What are the core principles of ROS 2?",
  "mode": "full-book",
  "top_k": 5
}
```

**Request Schema**:
```json
{
  "type": "object",
  "properties": {
    "query": {
      "type": "string",
      "description": "The user's question or query text",
      "minLength": 1
    },
    "mode": {
      "type": "string",
      "enum": ["full-book", "selected-text"],
      "default": "full-book",
      "description": "Query mode for the RAG agent"
    },
    "filters": {
      "type": "object",
      "description": "Additional filters for selected-text mode",
      "default": {}
    },
    "top_k": {
      "type": "integer",
      "minimum": 1,
      "maximum": 20,
      "default": 5,
      "description": "Number of results to retrieve"
    }
  },
  "required": ["query"]
}
```

##### Response - 200 OK
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
  "sources": [
    {
      "title": "Introduction to ROS 2 | Physical AI and Humanoid Robotics Course",
      "url": "https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/docs/module-1-ros2/chapter-1-introduction",
      "preview": "ROS 2 is the next generation of the Robot Operating System...",
      "score": 0.85
    }
  ],
  "confidence_level": "High",
  "confidence_score": 0.85,
  "query_id": "query_abc123",
  "timestamp": "2026-01-09T10:30:00.000Z",
  "processing_time_ms": 1250
}
```

**Response Schema**:
```json
{
  "type": "object",
  "properties": {
    "answer": {
      "type": "string",
      "description": "The answer text from the RAG agent"
    },
    "sources": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "title": { "type": "string" },
          "url": { "type": "string", "format": "uri" },
          "preview": { "type": "string" },
          "score": { "type": "number", "minimum": 0, "maximum": 1 }
        },
        "required": ["title", "url", "score"]
      }
    },
    "confidence_level": {
      "type": "string",
      "enum": ["Very Low", "Low", "Medium", "High"]
    },
    "confidence_score": {
      "type": "number",
      "minimum": 0,
      "maximum": 1
    },
    "query_id": {
      "type": "string",
      "description": "Unique identifier for the query"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "processing_time_ms": {
      "type": "number",
      "description": "Time taken to process the query in milliseconds"
    }
  },
  "required": ["answer", "sources", "confidence_level", "confidence_score", "timestamp"]
}
```

##### Error Responses
- `400 Bad Request`: Invalid request parameters
- `500 Internal Server Error`: Server-side error during processing

### Selected Text Query Endpoint

#### `POST /api/selected-text`
Process a query based on selected text context through the RAG agent.

##### Request
```json
{
  "query": "Explain the concept mentioned in this text",
  "selection_context": "The concept of middleware in robotics is crucial for communication between different components...",
  "source_url": "https://example.com/page-with-selection",
  "top_k": 3
}
```

**Request Schema**:
```json
{
  "type": "object",
  "properties": {
    "query": {
      "type": "string",
      "description": "The user's question about the selected text",
      "minLength": 1
    },
    "selection_context": {
      "type": "string",
      "description": "The selected text that provides context",
      "minLength": 1
    },
    "source_url": {
      "type": "string",
      "format": "uri",
      "description": "URL of the source page where text was selected"
    },
    "top_k": {
      "type": "integer",
      "minimum": 1,
      "maximum": 20,
      "default": 5
    }
  },
  "required": ["query", "selection_context"]
}
```

##### Response - 200 OK
Same response format as the `/api/chat` endpoint.

### Health Check Endpoint

#### `GET /api/health`
Health check endpoint to verify API availability.

##### Response - 200 OK
```json
{
  "status": "healthy",
  "timestamp": "2026-01-09T10:30:00.000Z",
  "version": "1.0.0"
}
```

**Response Schema**:
```json
{
  "type": "object",
  "properties": {
    "status": {
      "type": "string",
      "enum": ["healthy", "unhealthy"]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "version": {
      "type": "string",
      "description": "API version"
    }
  },
  "required": ["status", "timestamp", "version"]
}
```

## Error Response Format

#### Common Error Response Schema
```json
{
  "type": "object",
  "properties": {
    "error_code": {
      "type": "string",
      "description": "Standard error code (e.g., VALIDATION_ERROR, INTERNAL_ERROR)"
    },
    "message": {
      "type": "string",
      "description": "Human-readable error message"
    },
    "details": {
      "type": "object",
      "description": "Additional error details (optional)"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    }
  },
  "required": ["error_code", "message", "timestamp"]
}
```

### Example Error Responses

#### 400 Bad Request
```json
{
  "error_code": "VALIDATION_ERROR",
  "message": "Invalid query parameter: query is required and cannot be empty",
  "details": {
    "field_errors": [
      {
        "field": "query",
        "error": "Value is required"
      }
    ]
  },
  "timestamp": "2026-01-09T10:30:00.000Z"
}
```

#### 500 Internal Server Error
```json
{
  "error_code": "INTERNAL_ERROR",
  "message": "An unexpected error occurred while processing your request",
  "timestamp": "2026-01-09T10:30:00.000Z"
}
```

## Cross-Origin Resource Sharing (CORS)
The API supports CORS for frontend integration:
- Allowed origins: Configurable via environment variables
- Allowed methods: GET, POST
- Allowed headers: Content-Type, Authorization (if added later)

## Rate Limiting
- Rate limits may be enforced based on IP address
- Limits: Configurable via environment variables (default: 30 requests per minute)
- Exceeded requests return 429 status code