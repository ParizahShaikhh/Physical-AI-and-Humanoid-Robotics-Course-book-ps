# Quickstart Guide: Web Content Ingestion and Embeddings

## Overview
This guide provides the essential steps to set up and run the web content ingestion pipeline that extracts content from a Docusaurus website, generates embeddings with Cohere, and stores them in Qdrant Cloud.

## Prerequisites
- Python 3.11 or higher
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key
- Access to the target Docusaurus website

## Setup Instructions

### 1. Create the project directory
```bash
mkdir book-backend
cd book-backend
```

### 2. Initialize the project with UV
```bash
uv init
```

### 3. Create the environment file
Create a `.env` file in the project root with the following content:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url
TARGET_WEBSITE_URL=https://your-docusaurus-site.example.com
```

### 4. Install dependencies
```bash
uv add requests beautifulsoup4 cohere qdrant-client python-dotenv pytest
```

### 5. Create the main application file
Create `main.py` with the complete ingestion pipeline implementation.

## Implementation Steps

### Phase 1: Basic Structure
1. Create `main.py` with necessary imports and configuration loading
2. Implement configuration loading from environment variables
3. Set up basic logging and error handling

### Phase 2: Web Crawling
1. Implement URL discovery to find all pages on the target website
2. Create HTTP request handling with proper headers and error handling
3. Add rate limiting to avoid overwhelming the target website

### Phase 3: Content Extraction
1. Implement HTML parsing to extract clean text content
2. Create metadata extraction (title, description, etc.)
3. Add content cleaning to remove navigation and template elements

### Phase 4: Text Chunking
1. Implement text chunking algorithm to divide content appropriately
2. Add overlap handling to preserve context across chunks
3. Create chunk metadata with source information

### Phase 5: Embedding Generation
1. Integrate with Cohere API for embedding generation
2. Implement batch processing for efficiency
3. Add error handling for API rate limits

### Phase 6: Vector Storage
1. Connect to Qdrant Cloud for vector storage
2. Create vector collection with appropriate settings
3. Store embeddings with proper metadata linking

### Phase 7: Pipeline Orchestration
1. Implement the main() function to orchestrate the full pipeline
2. Add progress reporting and status tracking
3. Create comprehensive error handling

## Running the Ingestion Pipeline

### Execute the full pipeline:
```bash
python main.py
```

### For development and testing:
```bash
# Run with specific target URL
python main.py --target-url https://example.com

# Run in debug mode
python main.py --debug

# Process only specific pages
python main.py --pages page1,page2,page3
```

## Testing

### Run the test suite:
```bash
pytest tests/
```

### Test specific components:
```bash
# Test the web crawler
python -m pytest tests/test_ingestion.py::test_web_crawler

# Test content extraction
python -m pytest tests/test_ingestion.py::test_content_extraction

# Test the full pipeline
python -m pytest tests/test_ingestion.py::test_full_pipeline
```

## Common Customization Examples

### Custom chunk size:
```python
# In main.py, adjust the CHUNK_SIZE constant
CHUNK_SIZE = 1500  # characters per chunk
```

### Custom rate limiting:
```python
# In main.py, adjust the delay between requests
REQUEST_DELAY = 1.0  # seconds between requests
```

### Custom metadata extraction:
```python
# In main.py, modify the extract_metadata function
def extract_metadata(soup):
    # Add custom metadata extraction logic
    pass
```

## Troubleshooting

### API rate limits exceeded:
- Check Cohere and Qdrant API usage
- Adjust the rate limiting parameters
- Consider upgrading to a higher tier if needed

### Website access issues:
- Verify the target website is accessible
- Check if the website requires authentication
- Adjust user agent or request headers as needed

### Memory issues with large websites:
- Process websites in smaller batches
- Increase system memory if possible
- Consider implementing streaming processing

## Next Steps

1. Review the implementation against the feature specification
2. Run comprehensive tests to validate functionality
3. Monitor the ingestion process for performance
4. Set up monitoring and alerting for ongoing operations