from pydantic import BaseModel
from typing import Optional
from uuid import UUID
from datetime import datetime


class ContentChunk(BaseModel):
    id: Optional[str] = None
    chapter: Optional[str] = None
    section: Optional[str] = None
    url: Optional[str] = None
    title: Optional[str] = None
    content: str
    qdrant_point_id: Optional[str] = None
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None


class ContentChunkCreate(BaseModel):
    chapter: Optional[str] = None
    section: Optional[str] = None
    url: str
    title: str
    content: str


class ContentChunkResponse(BaseModel):
    id: str
    chapter: Optional[str]
    section: Optional[str]
    url: str
    title: str
    content: str
    created_at: datetime
    updated_at: datetime