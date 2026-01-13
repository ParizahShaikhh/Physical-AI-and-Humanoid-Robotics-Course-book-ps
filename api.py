"""
FastAPI integration for RAG agent
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi import Request
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from agent import RAGAgent
from api_models import QueryResponse, QueryRequest, SelectedTextRequest
from datetime import datetime
import time
import uuid
import logging
from logging.handlers import RotatingFileHandler
import json
from collections import defaultdict
import threading

# Configure logging
def setup_logging():
    # Create logs directory if it doesn't exist
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Create logger
    logger = logging.getLogger("rag_api")
    logger.setLevel(logging.INFO)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create file handler with rotation
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, "rag_api.log"),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)

    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)

    # Add handlers to logger
    if not logger.handlers:
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

    return logger

logger = setup_logging()

# Performance metrics tracking
class PerformanceMetrics:
    def __init__(self):
        self.metrics_lock = threading.Lock()
        self.request_counts = defaultdict(int)
        self.response_times = defaultdict(list)
        self.error_counts = defaultdict(int)

    def record_request(self, endpoint: str):
        with self.metrics_lock:
            self.request_counts[endpoint] += 1

    def record_response_time(self, endpoint: str, response_time: float):
        with self.metrics_lock:
            self.response_times[endpoint].append(response_time)
            # Keep only the last 100 measurements to avoid memory issues
            if len(self.response_times[endpoint]) > 100:
                self.response_times[endpoint] = self.response_times[endpoint][-100:]

    def record_error(self, endpoint: str):
        with self.metrics_lock:
            self.error_counts[endpoint] += 1

    def get_metrics(self, endpoint: str = None):
        with self.metrics_lock:
            if endpoint:
                return {
                    "requests": self.request_counts[endpoint],
                    "avg_response_time": sum(self.response_times[endpoint]) / len(self.response_times[endpoint]) if self.response_times[endpoint] else 0,
                    "p95_response_time": self._calculate_percentile(self.response_times[endpoint], 0.95) if self.response_times[endpoint] else 0,
                    "errors": self.error_counts[endpoint],
                    "error_rate": self.error_counts[endpoint] / self.request_counts[endpoint] if self.request_counts[endpoint] > 0 else 0
                }
            else:
                all_metrics = {}
                for ep in set(list(self.request_counts.keys()) + list(self.response_times.keys()) + list(self.error_counts.keys())):
                    all_metrics[ep] = {
                        "requests": self.request_counts[ep],
                        "avg_response_time": sum(self.response_times[ep]) / len(self.response_times[ep]) if self.response_times[ep] else 0,
                        "p95_response_time": self._calculate_percentile(self.response_times[ep], 0.95) if self.response_times[ep] else 0,
                        "errors": self.error_counts[ep],
                        "error_rate": self.error_counts[ep] / self.request_counts[ep] if self.request_counts[ep] > 0 else 0
                    }
                return all_metrics

    def _calculate_percentile(self, data, percentile):
        if not data:
            return 0
        sorted_data = sorted(data)
        index = int(len(sorted_data) * percentile)
        return sorted_data[min(index, len(sorted_data) - 1)]

metrics = PerformanceMetrics()

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
app = FastAPI(title="RAG Agent API", version="1.0.0")

# Add rate limit exception handler
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app",  # Your Vercel domain
        "http://localhost:3000",  # Local development
        "http://localhost:3001",  # Alternative local port
        "http://localhost:3002",  # Alternative local port
        "http://localhost:3003",  # Alternative local port
        "http://localhost:3004",  # Alternative local port
        "http://localhost:3005",  # Alternative local port
        "http://localhost:3006",  # Alternative local port
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize the RAG agent
rag_agent = RAGAgent()

def format_rag_response(rag_result: str, start_time: float) -> QueryResponse:
    """
    Format the RAG agent response into the standardized QueryResponse model.
    This function extracts sources and other metadata from the RAG result.
    """
    # Calculate processing time
    processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds

    # Extract sources and confidence from the RAG agent response
    # The RAG agent response contains source information and confidence levels
    # For simplicity, we'll assume the response format includes this information

    # In a real implementation, the RAG agent would return structured data
    # For now, we'll extract information from the response string
    answer = rag_result
    # Extract sources from the RAG agent response
    sources = []
    confidence_level = "Medium"  # This would be extracted from the response
    confidence_score = 0.7  # This would be extracted from the response

    return QueryResponse(
        answer=answer,
        sources=[],  # Would be populated with actual sources
        confidence_level=confidence_level,
        confidence_score=confidence_score,
        query_id=str(uuid.uuid4()),
        timestamp=datetime.now().isoformat(),
        processing_time_ms=processing_time
    )

def validate_query_request(query: str) -> bool:
    """
    Validate the query request parameters.
    """
    if not query or not query.strip():
        return False
    if len(query.strip()) < 3:  # Minimum length check
        return False
    return True

@app.get("/")
def read_root():
    return {"message": "RAG Agent API is running"}

@app.post("/api/chat")
@limiter.limit("10/minute")  # Limit to 10 requests per minute per IP
async def chat_endpoint(request: Request):
    """
    Process a general book content query through the RAG agent.
    """
    from fastapi import HTTPException
    from starlette.status import HTTP_400_BAD_REQUEST, HTTP_500_INTERNAL_SERVER_ERROR
    import json

    # Parse the request body to get the query request
    body = await request.json()
    query_request = QueryRequest(**body)

    query_id = str(uuid.uuid4())
    start_time = time.time()

    # Record request for metrics
    metrics.record_request("/api/chat")

    # Log incoming request
    logger.info(f"Chat endpoint called - Query ID: {query_id}, Client IP: {request.client.host if hasattr(request, 'client') else 'unknown'}")
    logger.debug(f"Request details - Query: {query_request.query[:100]}..., Mode: {query_request.mode}, Filters: {query_request.filters}, Top K: {query_request.top_k}, Context: {query_request.context[:50] if query_request.context else 'None'}")

    # Validate the request
    if not validate_query_request(query_request.query):
        error_msg = f"Invalid query received - Query ID: {query_id}"
        logger.warning(error_msg)
        metrics.record_error("/api/chat")  # Record error for metrics
        raise HTTPException(status_code=HTTP_400_BAD_REQUEST,
                          detail="Query must be at least 3 characters long")

    # Process the query using the RAG agent
    try:
        logger.info(f"Processing query with RAG agent - Query ID: {query_id}")
        # Use context if provided, otherwise use the original approach
        if query_request.context:
            # For now, we'll just pass the context as part of the query
            # In a more advanced implementation, we could use the context to refine the search
            full_query = f"{query_request.query} Context: {query_request.context}" if query_request.context else query_request.query
            response_text = rag_agent.ask(full_query, query_request.mode, query_request.filters, query_request.top_k)
        else:
            response_text = rag_agent.ask(query_request.query, query_request.mode, query_request.filters, query_request.top_k)

        # Log successful processing
        processing_time = (time.time() - start_time) * 1000
        logger.info(f"Query processed successfully - Query ID: {query_id}, Processing time: {processing_time:.2f}ms")

        # Record response time for metrics
        metrics.record_response_time("/api/chat", processing_time)

        # Format the response using our utility function
        formatted_response = format_rag_response(response_text, start_time)

        return formatted_response
    except Exception as e:
        processing_time = (time.time() - start_time) * 1000
        error_msg = f"Error processing query - Query ID: {query_id}, Processing time: {processing_time:.2f}ms, Error: {str(e)}"
        logger.error(error_msg, exc_info=True)
        metrics.record_error("/api/chat")  # Record error for metrics
        raise HTTPException(status_code=HTTP_500_INTERNAL_SERVER_ERROR,
                          detail=f"Error processing query: {str(e)}")

@app.post("/api/selected-text")
@limiter.limit("10/minute")  # Limit to 10 requests per minute per IP
async def selected_text_endpoint(request: Request):
    """
    Process a query based on selected text context through the RAG agent.
    """
    from fastapi import HTTPException
    from starlette.status import HTTP_400_BAD_REQUEST, HTTP_500_INTERNAL_SERVER_ERROR

    # Parse the request body to get the query request
    body = await request.json()
    query_request = QueryRequest(**body)

    query_id = str(uuid.uuid4())
    start_time = time.time()

    # Record request for metrics
    metrics.record_request("/api/selected-text")

    # Log incoming request
    logger.info(f"Selected-text endpoint called - Query ID: {query_id}, Client IP: {request.client.host if hasattr(request, 'client') else 'unknown'}")
    logger.debug(f"Request details - Query: {query_request.query[:100]}..., Mode: selected-text, Filters: {query_request.filters}, Top K: {query_request.top_k}, Context: {query_request.context[:50] if query_request.context else 'None'}")

    # Validate the request
    if not validate_query_request(query_request.query):
        error_msg = f"Invalid query received for selected-text endpoint - Query ID: {query_id}"
        logger.warning(error_msg)
        metrics.record_error("/api/selected-text")  # Record error for metrics
        raise HTTPException(status_code=HTTP_400_BAD_REQUEST,
                          detail="Query must be at least 3 characters long")

    # Process the query using the RAG agent with selected-text mode
    try:
        logger.info(f"Processing selected-text query with RAG agent - Query ID: {query_id}")
        # Use context if provided, otherwise use the original approach
        if query_request.context:
            # For now, we'll just pass the context as part of the query
            full_query = f"{query_request.query} Context: {query_request.context}" if query_request.context else query_request.query
            response_text = rag_agent.ask(full_query, "selected-text", query_request.filters, query_request.top_k)
        else:
            response_text = rag_agent.ask(query_request.query, "selected-text", query_request.filters, query_request.top_k)

        # Log successful processing
        processing_time = (time.time() - start_time) * 1000
        logger.info(f"Selected-text query processed successfully - Query ID: {query_id}, Processing time: {processing_time:.2f}ms")

        # Record response time for metrics
        metrics.record_response_time("/api/selected-text", processing_time)

        # Format the response using our utility function
        formatted_response = format_rag_response(response_text, start_time)

        return formatted_response
    except Exception as e:
        processing_time = (time.time() - start_time) * 1000
        error_msg = f"Error processing selected-text query - Query ID: {query_id}, Processing time: {processing_time:.2f}ms, Error: {str(e)}"
        logger.error(error_msg, exc_info=True)
        metrics.record_error("/api/selected-text")  # Record error for metrics
        raise HTTPException(status_code=HTTP_500_INTERNAL_SERVER_ERROR,
                          detail=f"Error processing selected-text query: {str(e)}")

@app.get("/api/health")
@limiter.limit("30/minute")  # Limit to 30 requests per minute per IP (health checks can be more frequent)
async def health_check(request: Request):
    """
    Health check endpoint for system status monitoring.
    """
    import time
    start_time = time.time()

    # Record request for metrics
    metrics.record_request("/api/health")

    # Log health check request
    logger.info(f"Health check endpoint called")

    # Check if RAG agent is available
    try:
        # Perform a simple test by checking if the agent is initialized
        agent_available = rag_agent is not None

        if not agent_available:
            error_msg = "RAG agent not available during health check"
            logger.error(error_msg)
            metrics.record_error("/api/health")  # Record error for metrics
            from fastapi import HTTPException
            from starlette.status import HTTP_503_SERVICE_UNAVAILABLE
            raise HTTPException(status_code=HTTP_503_SERVICE_UNAVAILABLE,
                              detail="RAG agent not available")

        # Calculate response time
        response_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Log successful health check
        logger.info(f"Health check successful - Response time: {response_time:.2f}ms")

        # Record response time for metrics
        metrics.record_response_time("/api/health", response_time)

        return {
            "status": "healthy",
            "timestamp": datetime.now().isoformat(),
            "version": "1.0.0",
            "response_time_ms": round(response_time, 2),
            "components": {
                "rag_agent": "available",
                "database_connection": "not_implemented_yet"  # Placeholder for future DB health check
            }
        }
    except Exception as e:
        response_time = (time.time() - start_time) * 1000
        error_msg = f"Health check failed - Response time: {response_time:.2f}ms, Error: {str(e)}"
        logger.error(error_msg, exc_info=True)
        metrics.record_error("/api/health")  # Record error for metrics
        from fastapi import HTTPException
        from starlette.status import HTTP_503_SERVICE_UNAVAILABLE
        raise HTTPException(status_code=HTTP_503_SERVICE_UNAVAILABLE,
                          detail=f"Health check failed: {str(e)}")


@app.get("/api/metrics")
@limiter.limit("60/minute")  # Allow more frequent metric checks
async def get_metrics(request: Request):
    """
    Performance metrics endpoint to monitor system performance.
    """
    logger.info("Performance metrics endpoint called")

    # Get all metrics
    all_metrics = metrics.get_metrics()

    return {
        "timestamp": datetime.now().isoformat(),
        "version": "1.0.0",
        "metrics": all_metrics,
        "summary": {
            "total_requests": sum([v["requests"] for v in all_metrics.values()]),
            "total_errors": sum([v["errors"] for v in all_metrics.values()]),
            "overall_error_rate": sum([v["errors"] for v in all_metrics.values()]) / sum([v["requests"] for v in all_metrics.values()]) if sum([v["requests"] for v in all_metrics.values()]) > 0 else 0
        }
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)/ /   B u i l d   t r i g g e r   0 1 / 1 3 / 2 0 2 6   1 5 : 5 9 : 2 3  
 