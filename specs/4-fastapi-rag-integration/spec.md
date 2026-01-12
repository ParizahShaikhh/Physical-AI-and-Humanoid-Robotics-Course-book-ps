# Feature Specification: FastAPI Integration Between Frontend and RAG Backend

## Overview

### Feature Description
Expose the RAG agent through a FastAPI service so the book frontend can send queries and receive grounded responses in real time.

### Target System
The OpenAI RAG agent built in Spec-3 and the Docusaurus frontend deployed on Vercel.

### Focus
Create a FastAPI service layer that connects the frontend to the RAG agent, enabling real-time querying and response delivery.

## Success Criteria

- FastAPI endpoints are available for chat and selected-text queries
- Requests from the frontend successfully reach the RAG agent
- Structured JSON responses are delivered to the frontend within 10 seconds
- Service supports both local development and production environments
- 95% of queries result in successful responses with source attribution

## User Scenarios & Testing

### Primary User Scenario
As a user of the book frontend, I want to submit questions about book content and receive accurate, source-attributed responses in real time.

### User Flow
1. User submits a query through the frontend interface
2. Frontend sends the query to the FastAPI endpoint
3. FastAPI service processes the request and forwards it to the RAG agent
4. RAG agent retrieves relevant content and generates a grounded response
5. FastAPI service returns the response in structured JSON format
6. Frontend displays the response to the user with source attribution

### Testing Scenarios
- Submit various types of queries (simple, complex, edge cases)
- Verify response structure and content accuracy
- Test both full-book and selected-text query modes
- Validate error handling for malformed requests
- Confirm source attribution is properly included in responses

## Functional Requirements

### FR-1: Query Endpoint
**Requirement**: The system shall provide an endpoint to accept user queries.
- **Acceptance Criteria**:
  - Accepts GET/POST requests with query parameters
  - Supports both chat and selected-text query modes
  - Validates input parameters before processing
- **Test**: Send various query types and verify successful processing

### FR-2: Response Format
**Requirement**: The system shall return responses in a standardized JSON structure.
- **Acceptance Criteria**:
  - Response includes the answer text
  - Response includes source attribution
  - Response includes confidence level
  - Response includes processing metadata (timestamp, query ID)
- **Test**: Submit queries and verify response structure matches specification

### FR-3: RAG Agent Integration
**Requirement**: The system shall integrate with the existing RAG agent without duplicating logic.
- **Acceptance Criteria**:
  - Imports agent logic from agent.py without code duplication
  - Maintains all existing RAG functionality
  - Preserves caching and rate limiting features
- **Test**: Verify all RAG agent features work through the API

### FR-4: Environment Support
**Requirement**: The system shall support both local development and production environments.
- **Acceptance Criteria**:
  - Configurable through environment variables
  - Works with different API key configurations
  - Supports CORS for frontend integration
- **Test**: Deploy in both local and production-like environments

### FR-5: Error Handling
**Requirement**: The system shall handle errors gracefully and return informative messages.
- **Acceptance Criteria**:
  - Invalid queries return appropriate error codes
  - API errors are logged for debugging
  - Graceful degradation when RAG agent is unavailable
- **Test**: Submit invalid requests and verify proper error responses

## Non-Functional Requirements

### Performance
- Query response time: < 10 seconds for 95% of requests
- Support for concurrent users: minimum 100 simultaneous connections

### Security
- API keys stored securely in environment variables
- Input validation to prevent injection attacks
- Rate limiting to prevent abuse

### Scalability
- Horizontal scaling capability
- Efficient resource utilization during peak loads

## Key Entities

### Query Request
- query: string containing the user's question
- mode: string indicating query mode ("full-book" or "selected-text")
- filters: optional object for selected-text mode
- top_k: integer for number of results to retrieve

### Response Object
- answer: string containing the response text
- sources: array of source references
- confidence_level: string indicating confidence level
- confidence_score: numeric confidence score
- query_id: unique identifier for the request
- timestamp: time of response generation

## Dependencies and Assumptions

### Dependencies
- RAG agent implementation in agent.py
- Qdrant vector database
- Cohere embedding service
- OpenRouter API access

### Assumptions
- The RAG agent is properly configured with API keys
- Network connectivity exists between services
- Frontend will handle the JSON response structure appropriately
- Environment variables are properly configured for different deployments

## Constraints

- Must use FastAPI as the API framework
- Cannot duplicate retrieval or embedding logic
- Agent logic must be imported from existing agent.py file
- API keys must be secured via environment variables
- No user authentication required
- No UI components to be developed
- No vector ingestion or indexing functionality needed

## Out of Scope

- User authentication and authorization
- Frontend UI components
- Vector database ingestion or indexing processes
- Advanced user management features
- Complex analytics or reporting