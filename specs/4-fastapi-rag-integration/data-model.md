# Data Model: FastAPI Integration Between Frontend and RAG Backend

## Request Models

### QueryRequest
**Description**: Represents a query request from the frontend to the RAG agent

**Fields**:
- `query`: string (required) - The user's question or query text
- `mode`: string (optional) - Query mode, either "full-book" or "selected-text" (default: "full-book")
- `filters`: dict (optional) - Additional filters for selected-text mode
- `top_k`: integer (optional) - Number of results to retrieve (default: 5, min: 1, max: 20)

**Validation**:
- Query must not be empty
- Mode must be one of allowed values
- top_k must be between 1 and 20

### SelectedTextRequest
**Description**: Specialized request for selected-text queries

**Fields**:
- `query`: string (required) - The user's question
- `selection_context`: string (required) - The selected text that provides context
- `source_url`: string (optional) - URL of the source page if applicable
- `filters`: dict (optional) - Additional filters

**Validation**:
- All required fields must be present
- Selection context must not be empty

## Response Models

### QueryResponse
**Description**: Represents the response from the RAG agent

**Fields**:
- `answer`: string (required) - The answer text from the RAG agent
- `sources`: array[SourceReference] (required) - List of sources used in the response
- `confidence_level`: string (required) - Confidence level ("Very Low", "Low", "Medium", "High")
- `confidence_score`: number (required) - Numerical confidence score (0.0 to 1.0)
- `query_id`: string (optional) - Unique identifier for the query
- `timestamp`: string (required) - ISO 8601 timestamp of response generation
- `processing_time_ms`: number (optional) - Time taken to process the query

### SourceReference
**Description**: Represents a reference to a source document

**Fields**:
- `title`: string (required) - Title of the source
- `url`: string (required) - URL of the source
- `preview`: string (optional) - Brief preview of the content
- `score`: number (required) - Relevance score (0.0 to 1.0)

### ErrorResponse
**Description**: Represents an error response

**Fields**:
- `error_code`: string (required) - Standard error code
- `message`: string (required) - Human-readable error message
- `details`: object (optional) - Additional error details
- `timestamp`: string (required) - ISO 8601 timestamp of error

## Session Models (Optional)

### ConversationSession
**Description**: Tracks conversation state (if needed for advanced features)

**Fields**:
- `session_id`: string (required) - Unique session identifier
- `created_at`: string (required) - ISO 8601 timestamp
- `last_activity`: string (required) - ISO 8601 timestamp
- `queries`: array[QueryHistory] (optional) - Historical queries in the session

### QueryHistory
**Description**: Represents a historical query in a session

**Fields**:
- `query`: string (required) - The original query
- `response_summary`: string (required) - Brief summary of the response
- `timestamp`: string (required) - ISO 8601 timestamp

## API Endpoint Specifications

### POST /api/chat
**Description**: Process a general book content query

**Request**: QueryRequest
**Response**: 200 QueryResponse | 400 ErrorResponse | 500 ErrorResponse

### POST /api/selected-text
**Description**: Process a query based on selected text context

**Request**: SelectedTextRequest
**Response**: 200 QueryResponse | 400 ErrorResponse | 500 ErrorResponse

### GET /api/health
**Description**: Health check endpoint

**Response**: 200 { "status": "healthy", "timestamp": "..." } | 503 { "status": "unhealthy", ... }

## State Transitions

### Query Processing Flow
1. Request received → Validation
2. Validation passed → RAG agent processing
3. RAG agent processed → Response formatting
4. Response formatted → Response sent
5. Any error → Error response

### Session Lifecycle (if implemented)
1. Session initiated → Active
2. Query processed → Active (with updated history)
3. Session timeout → Expired
4. Manual termination → Ended