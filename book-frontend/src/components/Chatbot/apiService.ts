// apiService.ts
interface QueryRequest {
  query: string;
  mode: 'full-book' | 'selected-text';
  filters: Record<string, any>;
  top_k: number;
  context?: string;
}

interface QueryResponse {
  answer: string;
  sources: Array<{
    title: string;
    url: string;
    preview?: string;
    score: number;
  }>;
  confidence_level: string;
  confidence_score: number;
  query_id?: string;
  timestamp: string;
  processing_time_ms?: number;
}

class ApiService {
  private baseUrl: string;

  constructor(baseUrl: string = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000') {
    this.baseUrl = baseUrl;
  }

  async sendQuery(request: QueryRequest, currentPageContext?: string): Promise<QueryResponse> {
    try {
      // Enhance the request with page context if available
      const enhancedRequest = {
        ...request,
        context: currentPageContext || '',
      };

      // Determine the endpoint based on the mode
      const endpoint = request.mode === 'selected-text' ? `${this.baseUrl}/api/selected-text` : `${this.baseUrl}/api/chat`;

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(enhancedRequest),
        credentials: 'include', // Include credentials for mobile compatibility
        mode: 'cors', // Explicitly set CORS mode
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`API Error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('API call failed:', error);
      throw error;
    }
  }

  async getHealth(): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/api/health`, {
        credentials: 'include', // Include credentials for mobile compatibility
        mode: 'cors' // Explicitly set CORS mode
      });
      if (!response.ok) {
        throw new Error(`Health check failed: ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error('Health check failed:', error);
      throw error;
    }
  }

  async getMetrics(): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/api/metrics`, {
        credentials: 'include', // Include credentials for mobile compatibility
        mode: 'cors' // Explicitly set CORS mode
      });
      if (!response.ok) {
        throw new Error(`Metrics request failed: ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error('Metrics request failed:', error);
      throw error;
    }
  }
}

export default ApiService;
export type { QueryRequest, QueryResponse };