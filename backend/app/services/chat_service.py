from typing import List, Dict, Any, Optional
from uuid import uuid4
from app.services.vector_db import VectorDBService
from app.services.metadata_db import MetadataDBService
from app.models.chat import ChatResponse, BookReference
from config import Config
from openai import OpenAI


class ChatService:
    """
    Core service for handling chat interactions with book content
    """

    def __init__(self):
        self.vector_db = VectorDBService()
        self.metadata_db = MetadataDBService()
        self.openai_client = OpenAI(api_key=Config.OPENAI_API_KEY)

    def get_answer(self, user_question: str, selected_text: Optional[str] = None, session_id: Optional[str] = None) -> ChatResponse:
        """
        Get an answer to the user's question
        """
        # If selected text is provided, use selected-text-only mode
        if selected_text and selected_text.strip():
            return self._handle_selected_text_mode(user_question, selected_text, session_id)

        # Otherwise, use normal RAG mode
        return self._handle_rag_mode(user_question, session_id)

    def _handle_selected_text_mode(self, user_question: str, selected_text: str, session_id: Optional[str]) -> ChatResponse:
        """
        Handle the selected-text-only mode where only the selected text is used
        """
        # Use OpenAI to answer based only on the selected text
        system_prompt = """You are an embedded AI assistant for this book only.
You may answer questions using:
- User-selected text (exclusive mode).

If selected text is provided, ignore all other sources.
If the answer is not present in allowed sources, say so clearly.
Never hallucinate, speculate, or use external knowledge."""

        user_prompt = f"""Selected text: {selected_text}

Question: {user_question}

Please provide an answer based only on the selected text. If the selected text does not contain enough information to answer this question, respond with: "The selected text does not contain enough information to answer this question.""""

        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=500,
                temperature=0.3
            )

            answer = response.choices[0].message.content.strip()

            # Check if the answer indicates insufficient information
            if "does not contain enough information" in answer:
                return ChatResponse(
                    answer=answer,
                    references=[],
                    session_id=session_id or str(uuid4())
                )

            # In selected-text mode, we don't have specific references to book sections
            # but we can still return the response
            return ChatResponse(
                answer=answer,
                references=[],
                session_id=session_id or str(uuid4())
            )
        except Exception as e:
            return ChatResponse(
                answer="The selected text does not contain enough information to answer this question.",
                references=[],
                session_id=session_id or str(uuid4())
            )

    def _handle_rag_mode(self, user_question: str, session_id: Optional[str]) -> ChatResponse:
        """
        Handle the normal RAG mode where vector search is performed
        """
        # Perform semantic search in vector database
        search_results = self.vector_db.search_similar_content(user_question)

        if not search_results:
            return ChatResponse(
                answer="This question is not answered in the book.",
                references=[],
                session_id=session_id or str(uuid4())
            )

        # Prepare context from search results
        context_parts = []
        references = []
        used_qdrant_ids = set()

        for result in search_results:
            content = result['content']
            metadata = result['metadata']
            score = result['score']

            # Only include results with sufficient relevance score
            if score > 0.3:  # Threshold for relevance
                context_parts.append(f"Context: {content}")

                # Create reference if we haven't seen this source before
                qdrant_point_id = metadata.get('qdrant_point_id')
                if qdrant_point_id and qdrant_point_id not in used_qdrant_ids:
                    # Get full metadata from the database for better references
                    chunk_info = self.metadata_db.get_content_by_qdrant_id(qdrant_point_id)
                    if chunk_info:
                        reference = BookReference(
                            title=chunk_info.get('title', 'Unknown'),
                            url=chunk_info.get('url', ''),
                            section=chunk_info.get('section', ''),
                            chapter=chunk_info.get('chapter')
                        )
                        references.append(reference)
                        used_qdrant_ids.add(qdrant_point_id)

        if not context_parts:
            return ChatResponse(
                answer="This question is not answered in the book.",
                references=[],
                session_id=session_id or str(uuid4())
            )

        # Build the context string
        context_str = "\n\n".join(context_parts)

        # Use OpenAI to generate an answer based on the context
        system_prompt = """You are an embedded AI assistant for this book only.
You may answer questions using:
- Retrieved book passages from the vector database.

If the answer is not present in allowed sources, say so clearly.
Never hallucinate, speculate, or use external knowledge."""

        user_prompt = f"""Context from the book:
{context_str}

Question: {user_question}

Please provide an answer based on the context provided. If the context does not contain enough information to answer this question, respond with: "This question is not answered in the book." Include relevant references to sections/chapters when available."""

        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=500,
                temperature=0.3
            )

            answer = response.choices[0].message.content.strip()

            # Check if the answer indicates no relevant content
            if "not answered in the book" in answer.lower():
                return ChatResponse(
                    answer=answer,
                    references=[],
                    session_id=session_id or str(uuid4())
                )

            return ChatResponse(
                answer=answer,
                references=references,
                session_id=session_id or str(uuid4())
            )
        except Exception as e:
            return ChatResponse(
                answer="This question is not answered in the book.",
                references=[],
                session_id=session_id or str(uuid4())
            )