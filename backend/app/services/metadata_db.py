import asyncpg
import uuid
from typing import List, Dict, Any, Optional
from config import Config
from app.models.content import ContentChunkCreate, ContentChunkResponse


class MetadataDBService:
    """
    Service for interacting with Neon Postgres database to store metadata
    """

    def __init__(self):
        self.database_url = Config.DATABASE_URL

    async def get_connection(self):
        """
        Get a connection to the database
        """
        return await asyncpg.connect(self.database_url)

    async def initialize_database(self):
        """
        Initialize the database tables if they don't exist
        """
        conn = await self.get_connection()
        try:
            # Create book_content_chunks table
            await conn.execute('''
                CREATE TABLE IF NOT EXISTS book_content_chunks (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    chapter VARCHAR(255),
                    section VARCHAR(255),
                    url VARCHAR(500),
                    title VARCHAR(500),
                    content TEXT,
                    qdrant_point_id UUID,
                    created_at TIMESTAMP DEFAULT NOW(),
                    updated_at TIMESTAMP DEFAULT NOW()
                )
            ''')

            # Create chat_sessions table
            await conn.execute('''
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    created_at TIMESTAMP DEFAULT NOW(),
                    updated_at TIMESTAMP DEFAULT NOW()
                )
            ''')

            # Create chat_messages table
            await conn.execute('''
                CREATE TABLE IF NOT EXISTS chat_messages (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    session_id UUID REFERENCES chat_sessions(id),
                    role VARCHAR(50), -- 'user' or 'assistant'
                    content TEXT,
                    created_at TIMESTAMP DEFAULT NOW()
                )
            ''')

        finally:
            await conn.close()

    async def store_content_chunk(self, chunk: ContentChunkCreate, qdrant_point_id: str) -> str:
        """
        Store a content chunk in the database
        Returns the ID of the stored record
        """
        conn = await self.get_connection()
        try:
            record = await conn.fetchrow('''
                INSERT INTO book_content_chunks (chapter, section, url, title, content, qdrant_point_id)
                VALUES ($1, $2, $3, $4, $5, $6)
                RETURNING id
            ''', chunk.chapter, chunk.section, chunk.url, chunk.title, chunk.content, qdrant_point_id)

            return str(record['id'])
        finally:
            await conn.close()

    async def store_content_chunks(self, chunks: List[Dict[str, Any]], qdrant_point_ids: List[str]) -> List[str]:
        """
        Store multiple content chunks in the database
        Returns list of IDs for the stored records
        """
        if len(chunks) != len(qdrant_point_ids):
            raise ValueError("Number of chunks must match number of qdrant_point_ids")

        conn = await self.get_connection()
        try:
            ids = []
            for chunk, qdrant_id in zip(chunks, qdrant_point_ids):
                record = await conn.fetchrow('''
                    INSERT INTO book_content_chunks (chapter, section, url, title, content, qdrant_point_id)
                    VALUES ($1, $2, $3, $4, $5, $6)
                    RETURNING id
                ''',
                chunk.get('chapter'),
                chunk.get('section'),
                chunk.get('url'),
                chunk.get('title'),
                chunk.get('content'),
                qdrant_id)

                ids.append(str(record['id']))

            return ids
        finally:
            await conn.close()

    async def get_content_by_qdrant_id(self, qdrant_point_id: str) -> Optional[Dict[str, Any]]:
        """
        Get content chunk by its Qdrant point ID
        """
        conn = await self.get_connection()
        try:
            record = await conn.fetchrow('''
                SELECT id, chapter, section, url, title, content, created_at, updated_at
                FROM book_content_chunks
                WHERE qdrant_point_id = $1
            ''', qdrant_point_id)

            if record:
                return {
                    'id': str(record['id']),
                    'chapter': record['chapter'],
                    'section': record['section'],
                    'url': record['url'],
                    'title': record['title'],
                    'content': record['content'],
                    'created_at': record['created_at'],
                    'updated_at': record['updated_at']
                }
            return None
        finally:
            await conn.close()

    async def get_content_by_url(self, url: str) -> List[Dict[str, Any]]:
        """
        Get content chunks by URL
        """
        conn = await self.get_connection()
        try:
            records = await conn.fetch('''
                SELECT id, chapter, section, url, title, content, created_at, updated_at
                FROM book_content_chunks
                WHERE url = $1
            ''', url)

            return [{
                'id': str(record['id']),
                'chapter': record['chapter'],
                'section': record['section'],
                'url': record['url'],
                'title': record['title'],
                'content': record['content'],
                'created_at': record['created_at'],
                'updated_at': record['updated_at']
            } for record in records]
        finally:
            await conn.close()

    async def search_content(self, search_term: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for content chunks by content or title
        """
        conn = await self.get_connection()
        try:
            records = await conn.fetch('''
                SELECT id, chapter, section, url, title, content, created_at, updated_at
                FROM book_content_chunks
                WHERE content ILIKE $1 OR title ILIKE $1
                LIMIT $2
            ''', f'%{search_term}%', limit)

            return [{
                'id': str(record['id']),
                'chapter': record['chapter'],
                'section': record['section'],
                'url': record['url'],
                'title': record['title'],
                'content': record['content'],
                'created_at': record['created_at'],
                'updated_at': record['updated_at']
            } for record in records]
        finally:
            await conn.close()