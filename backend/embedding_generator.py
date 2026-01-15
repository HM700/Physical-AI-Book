"""
Embedding Generator for Physical AI Book

This script generates embeddings for document chunks using Cohere's embedding models.
"""

import os
import asyncio
from typing import List, Dict
import cohere
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class EmbeddingGenerator:
    def __init__(self, api_key: str = None):
        if api_key is None:
            api_key = os.getenv("COHERE_API_KEY")

        if not api_key:
            raise ValueError("Cohere API key is required. Set COHERE_API_KEY environment variable.")

        self.client = cohere.AsyncClient(api_key=api_key)
        self.model = "embed-multilingual-v3.0"  # Cohere's latest multilingual embedding model

    async def generate_embeddings(self, texts: List[str], batch_size: int = 96) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.

        Args:
            texts: List of text strings to embed
            batch_size: Maximum number of texts to process in one API call (Cohere's limit is 96)

        Returns:
            List of embedding vectors
        """
        all_embeddings = []

        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            try:
                response = await self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_document"  # Optimal for document search applications
                )

                batch_embeddings = response.embeddings
                all_embeddings.extend(batch_embeddings)

                print(f"Processed batch: {i//batch_size + 1}/{(len(texts) - 1)//batch_size + 1}")

            except Exception as e:
                print(f"Error generating embeddings for batch starting at {i}: {str(e)}")
                # Handle the error appropriately - for now, we'll raise it
                raise e

        return all_embeddings

    async def embed_documents(self, documents: List[Dict]) -> List[Dict]:
        """
        Generate embeddings for a list of documents.

        Args:
            documents: List of documents with 'content' field

        Returns:
            List of documents with added 'embedding' field
        """
        if not documents:
            return []

        # Extract text content from documents
        texts = [doc['content'] for doc in documents]

        print(f"Generating embeddings for {len(texts)} document chunks...")

        # Generate embeddings
        embeddings = await self.generate_embeddings(texts)

        # Add embeddings to documents
        for i, doc in enumerate(documents):
            doc_copy = doc.copy()
            doc_copy['embedding'] = embeddings[i]
            yield doc_copy

    def get_model_info(self):
        """
        Get information about the embedding model being used.
        """
        return {
            "model": self.model,
            "dimensions": 1024,  # Cohere's multilingual v3 model produces 1024-dimensional embeddings
            "description": "Cohere's multilingual embedding model v3.0"
        }


async def main():
    """
    Main function to test the embedding generator.
    """
    print("Initializing Cohere embedding generator...")

    # Initialize the generator
    try:
        generator = EmbeddingGenerator()
    except ValueError as e:
        print(f"Error: {e}")
        print("Please set your COHERE_API_KEY environment variable.")
        return

    # Get model info
    model_info = generator.get_model_info()
    print(f"Using model: {model_info['model']}")
    print(f"Embedding dimensions: {model_info['dimensions']}")

    # Example usage with dummy documents (this would normally come from document_crawler)
    dummy_docs = [
        {
            'id': 'doc1',
            'content': 'Introduction to Physical AI and Humanoid Robotics',
            'metadata': {'source': 'intro.md', 'page': 1}
        },
        {
            'id': 'doc2',
            'content': 'Machine Learning algorithms for robotic control systems',
            'metadata': {'source': 'chapter-1.md', 'page': 2}
        }
    ]

    print(f"Generating embeddings for {len(dummy_docs)} documents...")

    # Generate embeddings for the documents
    embedded_docs_generator = generator.embed_documents(dummy_docs)
    embedded_docs = []

    async for doc in embedded_docs_generator:
        embedded_docs.append(doc)
        print(f"Embedded document: {doc['id']}, embedding length: {len(doc['embedding'])}")

    print(f"Successfully embedded {len(embedded_docs)} documents")

    return embedded_docs


if __name__ == "__main__":
    # Make sure to install required dependencies
    import subprocess
    import sys

    try:
        import cohere
    except ImportError:
        print("Installing cohere...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "cohere"])
        import cohere

    try:
        import dotenv
    except ImportError:
        print("Installing python-dotenv...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "python-dotenv"])
        import dotenv

    # Run the async main function
    result = asyncio.run(main())