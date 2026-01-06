from fastapi import APIRouter, HTTPException
from app.models.chat import IngestionRequest, IngestionResponse
from app.services.ingestion import IngestionService

router = APIRouter()
ingestion_service = IngestionService()


@router.post("/ingest", response_model=IngestionResponse)
async def ingest_endpoint(ingestion_request: IngestionRequest) -> IngestionResponse:
    """
    Ingestion endpoint to process and index book content
    """
    try:
        chunks_processed = await ingestion_service.ingest_book_content(ingestion_request.book_path)
        return IngestionResponse(
            status="completed",
            chunks_processed=chunks_processed
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing ingestion request: {str(e)}")