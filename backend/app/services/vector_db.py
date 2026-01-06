import uuid
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from openai import OpenAI
from config import Config


class VectorDBService:
    """
    Service for interacting with Qdrant vector database
    """

    def __init__(self):
        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
        )
        self.openai_client = OpenAI(api_key=Config.OPENAI_API_KEY)
        self.collection_name = Config.COLLECTION_NAME
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Ensure the collection exists in Qdrant with proper configuration
        """
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1536,  # OpenAI embedding size
                    distance=models.Distance.COSINE
                )
            )

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for text using OpenAI
        """
        response = self.openai_client.embeddings.create(
            input=text,
            model=Config.EMBEDDING_MODEL
        )
        return response.data[0].embedding

    def store_content_chunk(self, content: str, metadata: Dict[str, Any]) -> str:
        """
        Store a content chunk in the vector database
        Returns the ID of the stored point
        """
        embedding = self.embed_text(content)
        point_id = str(uuid.uuid4())

        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                "content": content,
                "metadata": metadata
            }
        )

        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )

        return point_id

    def store_content_chunks(self, chunks: List[Dict[str, Any]]) -> List[str]:
        """
        Store multiple content chunks in the vector database
        Returns list of IDs for the stored points
        """
        points = []
        ids = []

        for chunk in chunks:
            content = chunk['content']
            metadata = chunk.get('metadata', {})
            embedding = self.embed_text(content)
            point_id = str(uuid.uuid4())

            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": content,
                    "metadata": metadata
                }
            )

            points.append(point)
            ids.append(point_id)

        # Batch upsert for better performance
        if points:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

        return ids

    def search_similar_content(self, query: str, top_k: int = None) -> List[Dict[str, Any]]:
        """
        Search for similar content in the vector database
        """
        if top_k is None:
            top_k = Config.TOP_K

        query_embedding = self.embed_text(query)

        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k
        )

        results = []
        for result in search_results:
            payload = result.payload
            results.append({
                'content': payload.get('content', ''),
                'metadata': payload.get('metadata', {}),
                'score': result.score
            })

        return results

    def delete_collection(self):
        """
        Delete the entire collection (useful for reindexing)
        """
        try:
            self.client.delete_collection(self.collection_name)
        except:
            pass  # Collection might not exist