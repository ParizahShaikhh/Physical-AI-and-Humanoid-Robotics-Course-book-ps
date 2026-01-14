# app.py for Hugging Face Spaces - importing from api.py
import sys
import os
# Add the current directory to the path to ensure imports work
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Attempt to initialize the database with book content on startup
try:
    from ingest_books import BookIngestor, load_sample_book_content
    import logging

    logger = logging.getLogger(__name__)

    # Check if environment variables are available
    import os
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'OPENROUTER_API_KEY']
    env_vars_available = all(os.getenv(var) for var in required_vars)

    if env_vars_available:
        try:
            # Initialize the ingestor to check if connection works
            ingestor = BookIngestor()

            # Check if the collection exists and has content
            try:
                collection_info = ingestor.qdrant_client.get_collection(ingestor.collection_name)
                if collection_info.points_count == 0:
                    # Collection exists but is empty, so ingest sample content
                    logger.info("Collection is empty, ingesting sample book content...")
                    book_content = load_sample_book_content()
                    ingestor.ingest_book_content(
                        book_content=book_content,
                        source_title="Physical AI and Humanoid Robotics Course Book",
                        source_url="https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/docs/"
                    )
                    logger.info("Sample book content ingested successfully!")
                else:
                    logger.info(f"Collection already has {collection_info.points_count} points, skipping ingestion.")
            except:
                # Collection doesn't exist, so create and populate it
                logger.info("Collection doesn't exist, creating and ingesting sample book content...")
                book_content = load_sample_book_content()
                ingestor.ingest_book_content(
                    book_content=book_content,
                    source_title="Physical AI and Humanoid Robotics Course Book",
                    source_url="https://physical-ai-and-humanoid-robotics-c-sooty.vercel.app/docs/"
                )
                logger.info("Sample book content ingested successfully!")
        except Exception as e:
            logger.error(f"Error during auto-ingestion: {str(e)}")
            # Continue anyway, as the main API might still work if the database was populated manually
    else:
        logger.warning("Not all required environment variables are set. Database auto-ingestion skipped.")

except ImportError as e:
    print(f"Warning: Could not import ingestion modules: {str(e)}")
    print("Make sure all dependencies are installed.")

try:
    # Import the app from api.py
    from api import app
except ImportError as e:
    # Fallback: create a simple app if import fails
    from fastapi import FastAPI
    app = FastAPI(title="Fallback App", version="1.0.0")

    @app.get("/")
    def read_root():
        return {"error": f"Failed to import main app: {str(e)}"}

# Ensure the app object is available at the module level
app = app

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 7860))
    uvicorn.run(app, host="0.0.0.0", port=port)
