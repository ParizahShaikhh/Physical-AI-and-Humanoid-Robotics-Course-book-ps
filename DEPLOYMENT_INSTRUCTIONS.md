# Deployment Instructions

## Deploy Backend on Hugging Face Spaces

1. Create a Hugging Face account if you don't have one
2. Create a new Space with the "Docker" SDK option
3. Push the contents of the `book-backend` directory to your Space repository
4. Update the `.env.production` file in the frontend with your actual Hugging Face Space URL
5. The backend should automatically start when deployed

### Required Files for Hugging Face Space:
- `api.py` - Main FastAPI application
- `agent.py` - RAG Agent implementation
- `api_models.py` - Pydantic models
- `retrieve.py` - Retrieval pipeline
- `requirements.txt` - Python dependencies
- `Dockerfile` - Container configuration
- `app.py` - Hugging Face compatibility file

## Deploy Frontend on Vercel

1. The frontend is already deployed at: https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/
2. If you need to redeploy, make sure to update the environment variables with your Hugging Face Space URL
3. In Vercel dashboard, add the environment variable:
   - Key: `REACT_APP_BACKEND_URL`
   - Value: `https://your-username-your-space-name.hf.space`

## Environment Variables Needed

### For Frontend (Vercel):
- `REACT_APP_BACKEND_URL`: URL of your Hugging Face Space backend (e.g., `https://your-username-your-space-name.hf.space`)

### For Backend (Hugging Face Space):
- `OPENROUTER_API_KEY` or `OPENAI_API_KEY`: Your API key for the LLM service
- `COHERE_API_KEY`: Your Cohere API key for embeddings
- `QDRANT_URL`: Your Qdrant database URL
- `QDRANT_API_KEY`: Your Qdrant API key

## CORS Configuration

The backend is configured to accept requests from:
- `https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app` (your deployed frontend)
- `http://localhost:3000-3006` (for local development)

## Testing the Connection

After both deployments are live:
1. Visit your frontend at Vercel
2. The floating chatbot should connect to your Hugging Face backend
3. Test by asking questions about the book content
4. Verify that responses come through correctly

## Troubleshooting

If the frontend can't connect to the backend:
1. Check that your Hugging Face Space is running
2. Verify the `REACT_APP_BACKEND_URL` is correctly set
3. Check browser developer tools for CORS errors
4. Ensure API keys are properly configured in the backend