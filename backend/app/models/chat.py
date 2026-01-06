from pydantic import BaseModel
from typing import List, Optional
from uuid import UUID, uuid4
from datetime import datetime


class ChatRequest(BaseModel):
    user_question: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None


class BookReference(BaseModel):
    title: str
    url: str
    section: str
    chapter: Optional[str] = None


class ChatResponse(BaseModel):
    answer: str
    references: List[BookReference] = []
    session_id: str


class IngestionRequest(BaseModel):
    book_path: str


class IngestionResponse(BaseModel):
    status: str
    chunks_processed: int