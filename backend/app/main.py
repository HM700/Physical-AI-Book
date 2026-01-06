from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routes import chat, ingestion
from config import Config

# Validate configuration on startup
Config.validate()

app = FastAPI(
    title="Physical AI Book RAG Chatbot API",
    description="API for the integrated RAG chatbot for the Physical AI Book",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
app.include_router(ingestion.router, prefix="/api/v1", tags=["ingestion"])

@app.get("/")
async def root():
    return {"message": "Physical AI Book RAG Chatbot API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}