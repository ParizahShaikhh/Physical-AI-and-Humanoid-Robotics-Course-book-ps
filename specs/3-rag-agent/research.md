# Research for RAG Agent Implementation

## OpenAI Agents SDK Integration

### Decision: Use OpenAI Assistant API for RAG functionality
**Rationale**: The OpenAI Assistant API provides built-in tools functionality that allows custom functions to be called, making it ideal for connecting to our retrieval pipeline.

### Alternatives Considered:
- OpenAI Completions API with manual context injection
- OpenAI Chat Completions API with function calling
- LangChain with OpenAI integration
- Direct use of OpenAI Assistants API

### OpenAI Assistant Tools Approach Selected
The Assistant API with custom tools is the best approach because:
- It handles conversation memory automatically
- Supports file uploads and custom functions
- Provides structured tool calling
- Integrates well with retrieval-augmented generation patterns

## Qdrant Integration Patterns

### Decision: Create a custom tool function for Qdrant retrieval
**Rationale**: The OpenAI Assistant tools allow us to create custom functions that can be called during the conversation flow, enabling real-time retrieval from Qdrant.

### Implementation Pattern:
- Create a `search_book_content` function that accepts query parameters
- Map this to the existing retrieval pipeline
- Return formatted results to the assistant for response generation

## Retrieval Modes Implementation

### Decision: Implement two distinct search modes
**Rationale**: The specification requires both full-book and selected-text query modes.

### Full-book Mode:
- Search across all available book content
- Return comprehensive results from the entire corpus

### Selected-text Mode:
- Allow filtering by specific sections, chapters, or URLs
- Provide more targeted results based on user selection

## Response Generation Strategy

### Decision: Ground responses strictly in retrieved content
**Rationale**: The specification requires responses to be grounded only in retrieved book content to avoid hallucination.

### Implementation:
- Pass retrieved content as context to the assistant
- Use system prompt to enforce grounding in provided content
- Include source attribution in responses