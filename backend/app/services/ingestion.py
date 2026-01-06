from typing import List, Dict, Any
from app.utils.content_parser import DocusaurusContentParser
from app.utils.text_chunker import TextChunker
from app.services.vector_db import VectorDBService
from app.services.metadata_db import MetadataDBService
from app.models.content import ContentChunkCreate


class IngestionService:
    """
    Service for ingesting book content into the system
    """

    def __init__(self):
        self.content_parser = DocusaurusContentParser("docs")  # Default path, will be configurable
        self.text_chunker = TextChunker()
        self.vector_db = VectorDBService()
        self.metadata_db = MetadataDBService()

    async def ingest_book_content(self, book_path: str) -> int:
        """
        Ingest book content from the specified path
        Returns the number of chunks processed
        """
        # Initialize database tables
        await self.metadata_db.initialize_database()

        # Parse all markdown files
        raw_content = self.content_parser.parse_all_markdown_files()

        # Chunk the content
        chunked_content = self.text_chunker.chunk_multiple_contents(raw_content)

        # Prepare for storage
        vector_chunks = []
        metadata_chunks = []

        for chunk_data in chunked_content:
            content = chunk_data['content']
            metadata = chunk_data.get('metadata', {})

            # Prepare data for vector DB
            vector_chunk = {
                'content': content,
                'metadata': metadata
            }
            vector_chunks.append(vector_chunk)

            # Prepare data for metadata DB
            metadata_chunk = ContentChunkCreate(
                chapter=metadata.get('chapter'),
                section=metadata.get('section'),
                url=metadata.get('url', ''),
                title=metadata.get('title', ''),
                content=content
            )
            metadata_chunks.append({
                'chapter': metadata.get('chapter'),
                'section': metadata.get('section'),
                'url': metadata.get('url', ''),
                'title': metadata.get('title', ''),
                'content': content
            })

        # Store in vector database first
        qdrant_ids = self.vector_db.store_content_chunks(vector_chunks)

        # Store in metadata database
        await self.metadata_db.store_content_chunks(metadata_chunks, qdrant_ids)

        return len(vector_chunks)