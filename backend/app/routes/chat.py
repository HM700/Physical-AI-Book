from fastapi import APIRouter, HTTPException
from app.models.chat import ChatRequest, ChatResponse
from app.services.chat_service import ChatService

router = APIRouter()
chat_service = ChatService()


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest) -> ChatResponse:
    """
    Chat endpoint to get answers to questions about the book content
    """
    try:
        response = chat_service.get_answer(
            user_question=chat_request.user_question,
            selected_text=chat_request.selected_text,
            session_id=chat_request.session_id
        )
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")