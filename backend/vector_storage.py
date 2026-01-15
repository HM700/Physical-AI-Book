"""
Vector Storage and Search for Physical AI Book

This script stores document embeddings in Qdrant vector database and performs similarity search.
"""

import os
import uuid
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import numpy as np
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class VectorStorage:
    def __init__(self, location: str = ":memory:", collection_name: str = "physical_ai_book"):
        """
        Initialize Qdrant client and collection.

        Args:
            location: Qdrant location - ":memory:" for in-memory, or URL for remote server
            collection_name: Name of the collection to store vectors
        """
        self.client = QdrantClient(location=location)
        self.collection_name = collection_name
        self.vector_size = 1024  # Cohere's multilingual v3 model produces 1024-dimensional embeddings

        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """
        Create a collection in Qdrant with appropriate configuration.
        """
        # Check if collection exists
        collections = self.client.get_collections().collections
        collection_names = [coll.name for coll in collections]

        if self.collection_name not in collection_names:
            # Create new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.vector_size,
                    distance=models.Distance.COSINE  # Cosine distance is good for embeddings
                )
            )
            print(f"Created collection: {self.collection_name}")
        else:
            print(f"Collection {self.collection_name} already exists")

    def store_vectors(self, documents: List[Dict]):
        """
        Store document vectors with metadata in Qdrant.

        Args:
            documents: List of documents with 'embedding' and 'metadata' fields
        """
        points = []

        for doc in documents:
            # Create a point for Qdrant
            point = PointStruct(
                id=str(uuid.uuid4()),  # Generate unique ID
                vector=doc['embedding'],  # The embedding vector
                payload={
                    'id': doc.get('id', ''),
                    'content': doc['content'],
                    'metadata': doc['metadata']
                }
            )
            points.append(point)

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        print(f"Stored {len(points)} vectors in collection '{self.collection_name}'")

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict]:
        """
        Perform similarity search in the vector database.

        Args:
            query_embedding: Embedding vector to search for similar items
            limit: Number of similar items to return

        Returns:
            List of similar documents with scores
        """
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        results = []
        for hit in search_result:
            result = {
                'score': hit.score,
                'payload': hit.payload,
                'id': hit.id
            }
            results.append(result)

        return results

    def validate_storage(self) -> bool:
        """
        Validate that vectors were stored correctly by checking collection info.

        Returns:
            True if validation passes, False otherwise
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            count = collection_info.points_count
            print(f"Collection '{self.collection_name}' contains {count} vectors")

            # If we have vectors stored, validation passes
            return count > 0
        except Exception as e:
            print(f"Validation failed: {str(e)}")
            return False

    def get_collection_info(self):
        """
        Get detailed information about the collection.
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                'name': collection_info.config.params.vectors.size,
                'vector_size': collection_info.config.params.vectors.size,
                'distance': collection_info.config.params.vectors.distance,
                'points_count': collection_info.points_count,
                'indexed_vectors_count': collection_info.indexed_vectors_count
            }
        except Exception as e:
            print(f"Error getting collection info: {str(e)}")
            return None


class SimilarityValidator:
    def __init__(self, vector_storage: VectorStorage):
        self.storage = vector_storage

    def validate_basic_search(self, sample_query: str = "Physical AI robotics",
                             sample_embedding: Optional[List[float]] = None) -> Dict:
        """
        Perform a basic similarity search to validate the system.

        Args:
            sample_query: Sample query text
            sample_embedding: Pre-generated embedding for the query

        Returns:
            Dictionary with validation results
        """
        if sample_embedding is None:
            # For testing purposes, we'll create a random embedding
            # In a real scenario, this would come from the embedding generator
            sample_embedding = np.random.random(1024).tolist()

        # Perform similarity search
        results = self.storage.search_similar(sample_embedding, limit=3)

        validation_results = {
            'query_performed': True,
            'results_count': len(results),
            'top_score': results[0]['score'] if results else 0,
            'results': results
        }

        print(f"Similarity search validation:")
        print(f"  - Found {validation_results['results_count']} similar documents")
        print(f"  - Top score: {validation_results['top_score']:.4f}")

        if results:
            print("  - Top 3 results:")
            for i, result in enumerate(results[:3]):
                content_preview = result['payload']['content'][:100] + "..." if len(result['payload']['content']) > 100 else result['payload']['content']
                print(f"    {i+1}. Score: {result['score']:.4f}")
                print(f"       Content: {content_preview}")

        return validation_results


def main():
    """
    Main function to test vector storage and search functionality.
    """
    print("Initializing Qdrant vector storage...")

    # Initialize vector storage (using in-memory for now)
    storage = VectorStorage(location=":memory:", collection_name="physical_ai_book")

    # Display collection info
    info = storage.get_collection_info()
    if info:
        print(f"Collection info: {info}")

    # Create sample documents with embeddings (normally these would come from embedding_generator)
    sample_docs = [
        {
            'id': 'doc1',
            'content': 'Introduction to Physical AI and Humanoid Robotics',
            'metadata': {'source': 'intro.md', 'page': 1},
            'embedding': np.random.random(1024).tolist()  # Random embedding for demo
        },
        {
            'id': 'doc2',
            'content': 'Machine Learning algorithms for robotic control systems',
            'metadata': {'source': 'chapter-1.md', 'page': 2},
            'embedding': np.random.random(1024).tolist()  # Random embedding for demo
        },
        {
            'id': 'doc3',
            'content': 'Advanced topics in humanoid robot locomotion',
            'metadata': {'source': 'chapter-2.md', 'page': 3},
            'embedding': np.random.random(1024).tolist()  # Random embedding for demo
        }
    ]

    print(f"Storing {len(sample_docs)} sample documents...")

    # Store vectors in Qdrant
    storage.store_vectors(sample_docs)

    # Validate storage
    is_valid = storage.validate_storage()
    print(f"Storage validation: {'PASSED' if is_valid else 'FAILED'}")

    # Perform similarity search validation
    validator = SimilarityValidator(storage)
    validation_results = validator.validate_basic_search()

    return storage, validation_results


if __name__ == "__main__":
    # Make sure to install required dependencies
    import subprocess
    import sys

    try:
        import qdrant_client
    except ImportError:
        print("Installing qdrant-client...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "qdrant-client"])
        import qdrant_client

    try:
        import numpy as np
    except ImportError:
        print("Installing numpy...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy"])
        import numpy as np

    try:
        import dotenv
    except ImportError:
        print("Installing python-dotenv...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "python-dotenv"])
        import dotenv

    # Run the main function
    storage, results = main()