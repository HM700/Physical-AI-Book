import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    # OpenAI Configuration
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
    EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "text-embedding-ada-002")

    # Qdrant Configuration
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

    # Database Configuration
    DATABASE_URL = os.getenv("DATABASE_URL")

    # Application Configuration
    COLLECTION_NAME = "book_content_chunks"
    CHUNK_SIZE = 1000  # characters
    OVERLAP_SIZE = 200  # characters
    TOP_K = 5  # number of chunks to retrieve

    @classmethod
    def validate(cls):
        """Validate that all required environment variables are set"""
        required_vars = [
            'OPENAI_API_KEY',
            'QDRANT_URL',
            'DATABASE_URL'
        ]

        missing_vars = [var for var in required_vars if not getattr(cls, var)]
        if missing_vars:
            raise ValueError(f"Missing required environment variables: {missing_vars}")