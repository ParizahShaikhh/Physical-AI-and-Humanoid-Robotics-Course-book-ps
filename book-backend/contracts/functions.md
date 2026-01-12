# Function Contracts for RAG Agent

## search_book_content(query, mode, filters, top_k)

### Description
Search the book content using the retrieval pipeline and return relevant results.

### Parameters
- `query` (string): Natural language query text
- `mode` (string): Query mode - 'full-book' or 'selected-text'
- `filters` (object, optional): Additional filters for selected-text mode
  - `source_url` (string, optional): Filter by specific URL
  - `chapter` (string, optional): Filter by specific chapter
- `top_k` (number, optional): Number of results to return (default: 5)

### Returns
- `results` (array): Array of retrieval result objects
  - `id` (string): Unique identifier
  - `text` (string): Retrieved text content
  - `score` (number): Relevance score (0.0-1.0)
  - `source_url` (string): Source URL
  - `source_title` (string): Source title
  - `content_preview` (string): Content preview

### Errors
- `InvalidQueryError`: When query is empty or invalid
- `RetrievalError`: When retrieval pipeline fails

## generate_response(query, retrieved_content)

### Description
Generate a response based on the query and retrieved content.

### Parameters
- `query` (string): Original user query
- `retrieved_content` (array): Array of retrieved content objects

### Returns
- `response` (string): Generated response grounded in retrieved content
- `sources` (array): Array of source objects used

### Errors
- `GenerationError`: When response generation fails
- `NoContentError`: When no relevant content is retrieved

## validate_response(response, content)

### Description
Validate that the response is grounded in the provided content.

### Parameters
- `response` (string): Generated response
- `content` (array): Content used to generate response

### Returns
- `is_valid` (boolean): Whether response is properly grounded
- `issues` (array): Array of validation issues found

## format_response(response, sources)

### Description
Format the response with proper attribution and structure.

### Parameters
- `response` (string): Generated response
- `sources` (array): Source objects to attribute

### Returns
- `formatted_response` (string): Properly formatted response with attribution