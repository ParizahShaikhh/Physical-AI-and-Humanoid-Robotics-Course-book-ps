# Quickstart Guide: FastAPI Integration Between Frontend and RAG Backend

## Overview
This guide provides instructions for setting up and running the FastAPI integration between the frontend and RAG backend.

## Prerequisites
- Python 3.8+
- Node.js (for frontend development)
- Access to OpenRouter API (or configured alternative)
- Qdrant vector database running
- Existing RAG agent setup

## Setup Instructions

### 1. Backend Setup (`book-backend`)

#### Install Dependencies
```bash
cd book-backend
pip install fastapi uvicorn pydantic python-multipart python-dotenv
```

#### Environment Configuration
Create a `.env` file in the `book-backend` directory:
```env
OPENROUTER_API_KEY=your_openrouter_api_key_here
# Or alternatively:
OPENAI_API_KEY=your_openai_api_key_here

QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here

# Server configuration
HOST=0.0.0.0
PORT=8000
DEBUG=False
```

#### Run the API Server
```bash
cd book-backend
uvicorn api:app --host 0.0.0.0 --port 8000 --reload
```

The API will be available at `http://localhost:8000`

### 2. Frontend Setup (`book-frontend`)

#### Install Dependencies
```bash
cd book-frontend
npm install
```

#### Environment Configuration
Update your Docusaurus configuration to include the backend API URL:
```javascript
// In docusaurus.config.js or similar
const config = {
  // ... existing config
  themeConfig: {
    // ... existing theme config
    apiBaseUrl: process.env.API_BASE_URL || 'http://localhost:8000',
  },
};
```

#### Add Chatbot Component
The chatbot UI component should be integrated into relevant pages. Example React component:

```jsx
// Example chatbot component
import React, { useState } from 'react';

const ChatInterface = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);

    try {
      const res = await fetch('/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query })
      });

      const data = await res.json();
      setResponse(data);
    } catch (error) {
      console.error('Error:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="chat-interface">
      <form onSubmit={handleSubmit}>
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask about the book content..."
        />
        <button type="submit" disabled={loading}>
          {loading ? 'Loading...' : 'Send'}
        </button>
      </form>

      {response && (
        <div className="response">
          <p>{response.answer}</p>
          {response.sources && (
            <div className="sources">
              <h4>Sources:</h4>
              <ul>
                {response.sources.slice(0, 3).map((source, index) => (
                  <li key={index}>
                    <a href={source.url}>{source.title}</a>
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default ChatInterface;
```

### 3. API Endpoints

#### Available Endpoints
- `POST /api/chat` - General book content queries
- `POST /api/selected-text` - Queries based on selected text context
- `GET /api/health` - Health check

#### Example API Usage
```bash
# General query
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What are the core principles of ROS 2?", "mode": "full-book"}'

# Selected text query
curl -X POST http://localhost:8000/api/selected-text \
  -H "Content-Type: application/json" \
  -d '{"query": "Explain this concept", "selection_context": "The concept of middleware...", "top_k": 3}'
```

### 4. Development Mode
To run in development mode with hot reloading:
```bash
# Backend
cd book-backend
uvicorn api:app --reload

# Frontend (in separate terminal)
cd book-frontend
npm start
```

### 5. Production Deployment
For production deployment:
1. Set `DEBUG=False` in environment
2. Use a production WSGI server like Gunicorn:
   ```bash
   gunicorn api:app -w 4 -k uvicorn.workers.UvicornWorker
   ```
3. Configure reverse proxy (nginx, Apache) for SSL termination
4. Set up proper logging and monitoring

## Troubleshooting

### Common Issues
1. **CORS errors**: Ensure frontend origin is allowed in FastAPI CORS configuration
2. **API key errors**: Verify all required API keys are correctly set in environment variables
3. **Connection timeouts**: Check network connectivity to Qdrant and external APIs
4. **Import errors**: Verify all required packages are installed

### Health Checks
Use the health endpoint to verify the API is running:
```bash
curl http://localhost:8000/api/health
```

## Configuration Options
- `HOST`: Server host (default: 0.0.0.0)
- `PORT`: Server port (default: 8000)
- `DEBUG`: Enable debug mode (default: False)
- `MAX_REQUESTS_PER_MINUTE`: Rate limiting threshold (default: 30)