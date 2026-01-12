# FastAPI Integration Between Frontend and RAG Backend

## Overview
This document describes the integration between the Docusaurus frontend and the RAG (Retrieval-Augmented Generation) backend using FastAPI.

## Architecture
- **Frontend**: Docusaurus-based documentation site with React components
- **Backend**: FastAPI service that interfaces with the RAG agent
- **RAG Agent**: Uses Qdrant for semantic search and OpenRouter for response generation

## Backend API Endpoints

### `/api/chat` (POST)
- Processes general book content queries
- Request body: `{ "query": "string", "mode": "full-book|selected-text", "filters": {}, "top_k": 5 }`
- Response: QueryResponse with answer, sources, confidence, etc.

### `/api/selected-text` (POST)
- Processes queries with selected text context
- Request body: Same as `/api/chat`
- Response: QueryResponse with context-aware answer

### `/api/health` (GET)
- System status monitoring
- Response: Health status and component availability

### `/api/metrics` (GET)
- Performance metrics endpoint
- Response: Request counts, response times, error rates

## Frontend Integration

### Chatbot Component
- Located at `src/components/Chatbot/Chatbot.tsx`
- Provides a real-time chat interface
- Uses API service for backend communication
- Displays source citations and confidence levels

### Navigation
- Added "Chat" link to the navbar in `docusaurus.config.ts`
- Links to `/chat` page which uses the chatbot component

## Features Implemented

1. **Logging**: Comprehensive request/response logging with different levels
2. **Rate Limiting**: API abuse prevention with slowapi (10 requests/minute for chat endpoints)
3. **Performance Metrics**: Response time tracking, request counts, error rates
4. **Error Handling**: Proper error responses and validation
5. **Frontend Integration**: Full-featured chatbot component with UI

## Security Considerations
- CORS configured to allow frontend integration
- Rate limiting to prevent API abuse
- Input validation for all endpoints
- Secure API key handling (via environment variables)

## Testing
- Created test scripts for error scenarios
- Component tests for the chatbot UI
- End-to-end functionality validation

## Deployment Notes
- Backend runs on port 8000 by default
- Frontend can be built and deployed separately
- Environment variables required for API keys
- Production deployment should configure proper CORS origins