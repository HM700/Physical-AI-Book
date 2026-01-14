# Implementation Tasks: RAG Chatbot with Cohere Integration

## Overview
This document outlines the implementation tasks for the RAG chatbot feature that integrates both OpenAI and Cohere models for the Physical AI book. The implementation will follow the specifications in the `specs/002-rag-chatbot/` directory.

## Prerequisites
- Qdrant Cloud account with vector database
- Neon Postgres serverless database
- OpenAI API account
- Cohere API account
- Docusaurus-based book deployment

## Task Categories

### Setup Tasks [SETUP]
- [ ] Create project directory structure
- [ ] Initialize Python project with requirements.txt (including openai, cohere, fastapi, qdrant-client, psycopg2, python-dotenv)
- [ ] Set up configuration management (config.py)
- [ ] Create environment variable templates (.env.example)

### Core Development Tasks [CORE]

#### Database and Model Setup [P]
- [ ] Set up Qdrant vector database connection (backend/app/services/vector_db.py)
- [ ] Set up Neon Postgres connection (backend/app/services/metadata_db.py)
- [ ] Create OpenAI service integration (backend/app/services/openai_service.py)
- [ ] Create Cohere service integration (backend/app/services/cohere_service.py)
- [ ] Implement model selection logic (backend/app/services/model_service.py)

#### Content Ingestion Pipeline [P]
- [ ] Create Docusaurus markdown parser (backend/app/utils/content_parser.py)
- [ ] Implement text chunking utilities (backend/app/utils/text_chunker.py)
- [ ] Create content ingestion service (backend/app/services/ingestion.py)
- [ ] Implement ingestion endpoint (backend/app/routes/ingestion.py)

#### AI Services [P]
- [ ] Create chat request/response models (backend/app/models/chat.py)
- [ ] Implement core chat service logic (backend/app/services/chat_service.py)
- [ ] Add selected-text-only mode enforcement
- [ ] Add hallucination prevention logic
- [ ] Implement model switching (OpenAI/Cohere) functionality

#### API Endpoints [P]
- [ ] Create chat endpoint (backend/app/routes/chat.py)
- [ ] Add proper error handling and validation
- [ ] Implement session management models (backend/app/models/session.py)
- [ ] Create session management endpoints

#### Frontend Integration [P]
- [ ] Create ChatKit integration component (frontend/src/components/ChatWidget.jsx)
- [ ] Implement dual model selection (OpenAI/Cohere) in UI
- [ ] Add text selection support
- [ ] Add Docusaurus configuration for chat widget

### Testing Tasks [TEST]
- [ ] Unit tests for content parsing and chunking
- [ ] Unit tests for OpenAI service
- [ ] Unit tests for Cohere service
- [ ] Unit tests for chat service logic
- [ ] Integration tests for API endpoints
- [ ] End-to-end tests for complete flow with both models
- [ ] Performance tests for response times

### Documentation Tasks [DOC]
- [ ] Setup instructions (docs/setup-instructions.md)
- [ ] API reference documentation (docs/api-reference.md)
- [ ] Deployment guide
- [ ] User guide for model selection

## Dependencies and Execution Order

### Sequential Dependencies
1. Setup tasks must complete before any core development
2. Database connections (Qdrant/Postgres) before ingestion service
3. Core services before API endpoints
4. Backend API before frontend integration
5. Core functionality before testing
6. Implementation before documentation

### Parallel Tasks [P]
Tasks marked with [P] can be developed in parallel within their category:
- Database and model setup components
- Content ingestion pipeline components
- AI services components
- API endpoints
- Frontend integration components

## Success Criteria for Each Task

### Database and Model Setup
- [ ] Qdrant connection successfully established
- [ ] Neon Postgres connection successfully established
- [ ] OpenAI service properly integrated and tested
- [ ] Cohere service properly integrated and tested
- [ ] Model selection logic working correctly

### Content Ingestion Pipeline
- [ ] Successfully parse all Docusaurus markdown files
- [ ] Properly chunk content by sections and headings
- [ ] Store embeddings in Qdrant with correct metadata
- [ ] Store metadata in Neon Postgres with proper relationships
- [ ] Ingestion endpoint triggers complete pipeline successfully

### AI Services
- [ ] Chat service properly handles normal RAG queries with OpenAI
- [ ] Chat service properly handles normal RAG queries with Cohere
- [ ] Selected-text-only mode enforced correctly
- [ ] Hallucination prevention working (no external knowledge used)
- [ ] Proper response format with references when available
- [ ] Appropriate responses when no content found ("not answered in book")
- [ ] Model switching functionality working correctly

### API Endpoints
- [ ] Chat endpoint accepts proper request format
- [ ] Chat endpoint returns proper response format
- [ ] Error handling for invalid requests
- [ ] Session management working correctly
- [ ] Ingestion endpoint triggers processing successfully
- [ ] Both OpenAI and Cohere models accessible via API

### Frontend Integration
- [ ] Chat widget properly embedded in Docusaurus
- [ ] Dual model selection (OpenAI/Cohere) working in UI
- [ ] Text selection functionality working
- [ ] Communication with backend API working
- [ ] Proper display of responses and references
- [ ] Good user experience and interface

## Environment Setup Requirements

### Required Environment Variables
- `OPENAI_API_KEY`: OpenAI API key
- `COHERE_API_KEY`: Cohere API key
- `QDRANT_URL`: Qdrant Cloud URL
- `QDRANT_API_KEY`: Qdrant API key
- `DATABASE_URL`: Neon Postgres connection string
- `EMBEDDING_MODEL`: OpenAI embedding model name (default: text-embedding-ada-002)
- `DEFAULT_MODEL`: Default model to use (openai/cohere)

### External Dependencies
- Qdrant Cloud account with vector database
- Neon Postgres serverless database
- OpenAI API account with sufficient credits
- Cohere API account with sufficient credits
- Docusaurus-based book deployment

## Quality Standards
- All code follows Python best practices (PEP 8)
- Type hints included where appropriate
- Comprehensive error handling
- Proper input validation
- Security best practices implemented
- Performance considerations addressed
- Documentation for all public interfaces

## File Structure
```
backend/
├── app/
│   ├── main.py              # FastAPI app entry point
│   ├── models/
│   │   ├── chat.py          # Chat request/response models
│   │   ├── content.py       # Content chunk models
│   │   └── session.py       # Session models
│   ├── services/
│   │   ├── chat_service.py  # Core chat logic
│   │   ├── ingestion.py     # Content ingestion pipeline
│   │   ├── vector_db.py     # Qdrant integration
│   │   ├── metadata_db.py   # Neon Postgres integration
│   │   ├── openai_service.py # OpenAI integration
│   │   ├── cohere_service.py # Cohere integration
│   │   └── model_service.py # Model selection logic
│   ├── routes/
│   │   ├── chat.py          # Chat endpoints
│   │   └── ingestion.py     # Ingestion endpoints
│   └── utils/
│       ├── content_parser.py # Docusaurus markdown parser
│       └── text_chunker.py   # Text chunking utilities
├── requirements.txt
├── config.py                # Configuration management
└── .env.example             # Environment variables template

frontend/
├── src/
│   └── components/
│       └── ChatWidget.jsx   # ChatKit integration component with dual model support

docs/
├── setup-instructions.md    # Setup and deployment guide
└── api-reference.md         # API documentation
```