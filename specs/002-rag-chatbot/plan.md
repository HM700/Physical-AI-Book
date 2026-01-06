# Technical Plan: Integrated RAG Chatbot for Physical AI Book

**Feature**: 002-rag-chatbot
**Created**: 2026-01-06
**Status**: Draft

## Architecture Overview

### System Components

1. **Frontend Integration**:
   - OpenAI ChatKit embedded in Docusaurus
   - Text selection support
   - Session management

2. **Backend API**:
   - FastAPI application
   - Chat endpoint
   - Ingestion endpoint

3. **AI Processing**:
   - OpenAI API integration
   - System prompt for book-specific responses
   - Selected-text-only mode enforcement

4. **Data Storage**:
   - Qdrant Cloud for vector embeddings
   - Neon Serverless Postgres for metadata

### Tech Stack

- **Backend**: Python 3.11+, FastAPI
- **AI Framework**: OpenAI API
- **Vector DB**: Qdrant Cloud
- **Metadata DB**: Neon Postgres
- **Embeddings**: OpenAI text-embedding model
- **Frontend**: OpenAI ChatKit, Docusaurus
- **Deployment**: Independent API deployment

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
│   │   └── metadata_db.py   # Neon Postgres integration
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
│       └── ChatWidget.jsx   # ChatKit integration component
└── docusaurus.config.js     # Docusaurus configuration

docs/
├── setup-instructions.md    # Setup and deployment guide
└── api-reference.md         # API documentation
```

## Implementation Phases

### Phase 1: Project Setup and Configuration
- Set up FastAPI project structure
- Configure environment variables
- Set up Qdrant and Neon Postgres connections

### Phase 2: Content Ingestion Pipeline
- Parse Docusaurus markdown files
- Chunk content by sections
- Store embeddings in Qdrant
- Store metadata in Neon Postgres

### Phase 3: Core AI Services
- Implement chat service logic
- Create OpenAI integration
- Implement selected-text-only mode
- Add hallucination prevention

### Phase 4: API Endpoints
- Create /chat endpoint
- Create /ingest endpoint
- Add proper error handling

### Phase 5: Frontend Integration
- Embed ChatKit in Docusaurus
- Implement text selection support
- Add session management

### Phase 6: Testing and Documentation
- Unit tests for core services
- Integration tests
- Setup documentation
- Deployment guide

## Database Schema

### Neon Postgres Schema

```sql
-- Book content metadata
CREATE TABLE book_content_chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter VARCHAR(255),
    section VARCHAR(255),
    url VARCHAR(500),
    title VARCHAR(500),
    content TEXT,
    qdrant_point_id UUID,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Chat sessions
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Chat messages
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id),
    role VARCHAR(50), -- 'user' or 'assistant'
    content TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);
```

## API Design

### Chat Endpoint
```
POST /chat
Content-Type: application/json

{
  "user_question": "string",
  "selected_text": "string", // optional
  "session_id": "string" // optional
}

Response:
{
  "answer": "string",
  "references": [
    {
      "title": "string",
      "url": "string",
      "section": "string"
    }
  ]
}
```

### Ingestion Endpoint
```
POST /ingest
Content-Type: application/json

{
  "book_path": "string" // path to Docusaurus content
}

Response:
{
  "status": "completed",
  "chunks_processed": "number"
}
```

## Security Considerations

- Environment variables for API keys
- Rate limiting for API endpoints
- Input validation for user queries
- Sanitization of selected text
- Session management for chat history

## Performance Requirements

- Response time under 5 seconds for chat queries
- Support for concurrent users
- Efficient vector search in Qdrant
- Proper connection pooling for databases

## Deployment Strategy

- Backend API deployed independently
- Environment-specific configuration
- Health check endpoints
- Monitoring and logging setup