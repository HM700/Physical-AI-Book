---
id: 1
title: rag-chatbot-implementation
stage: implementation
date: 2026-01-06
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot
branch: 002-rag-chatbot
user: user
command: sp.implement
labels: ["rag", "chatbot", "ai", "docusaurus", "qdrant", "postgres"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/app/main.py"
  - "backend/app/models/chat.py"
  - "backend/app/models/content.py"
  - "backend/app/models/session.py"
  - "backend/app/services/chat_service.py"
  - "backend/app/services/ingestion.py"
  - "backend/app/services/vector_db.py"
  - "backend/app/services/metadata_db.py"
  - "backend/app/routes/chat.py"
  - "backend/app/routes/ingestion.py"
  - "backend/app/utils/content_parser.py"
  - "backend/app/utils/text_chunker.py"
  - "backend/config.py"
  - "backend/requirements.txt"
  - "backend/.env.example"
  - "backend/README.md"
  - "frontend/src/components/ChatWidget.jsx"
  - "docs/setup-instructions.md"
  - "docs/api-reference.md"
  - "specs/002-rag-chatbot/spec.md"
  - "specs/002-rag-chatbot/plan.md"
  - "specs/002-rag-chatbot/tasks.md"
  - "specs/002-rag-chatbot/checklists/requirements.md"
tests: []
---

# RAG Chatbot Implementation

## Prompt

```
sp.implement

You are implementing Task 2: Integrated RAG Chatbot Development for a Docusaurus-based book already deployed to GitHub Pages.

The goal is to build a production-ready, embedded Retrieval-Augmented Generation (RAG) chatbot that can answer questions about the book's content.

────────────────────────────────────────
SYSTEM ARCHITECTURE (MANDATORY)
────────────────────────────────────────

Implement the chatbot using the following stack:

- Backend API: FastAPI (Python)
- AI Framework: OpenAI Agents / ChatKit SDK
- Vector Database: Qdrant Cloud (Free Tier)
- Metadata & Session Storage: Neon Serverless Postgres
- Embedding Model: OpenAI text-embedding model
- Frontend Integration: OpenAI ChatKit embedded inside Docusaurus
- Deployment: API deployable independently from GitHub Pages

────────────────────────────────────────
CORE FUNCTIONAL REQUIREMENTS
────────────────────────────────────────

1. Book Content Ingestion
   - Parse all Docusaurus markdown files
   - Chunk content by section and heading
   - Store:
     - Embeddings in Qdrant
     - Metadata (chapter, section, URL) in Neon Postgres
   - Provide a reproducible ingestion script

2. Default RAG Question Answering
   - When no text is selected by the user:
     - Perform semantic vector search in Qdrant
     - Retrieve top-k relevant passages
     - Answer strictly using retrieved passages
     - Include section/chapter references when available
   - If no relevant content is found:
     - Respond: "This question is not answered in the book."

3. Selected-Text-Only Mode (Strict Enforcement)
   - If the user provides selected/highlighted text:
     - DO NOT query Qdrant
     - DO NOT use prior conversation
     - Answer using ONLY the selected text
   - If the selected text does not contain the answer:
     - Respond: "The selected text does not contain enough information to answer this question."

4. Hallucination Prevention
   - The assistant MUST NOT:
     - Use external knowledge
     - Infer missing information
     - Answer beyond book content
   - All answers must be grounded in retrieved or selected text

────────────────────────────────────────
AI AGENT PROMPT (IMPLEMENT AS SYSTEM PROMPT)
────────────────────────────────────────

"You are an embedded AI assistant for this book only.
You may answer questions using:
- Retrieved book passages from the vector database, OR
- User-selected text (exclusive mode).

If selected text is provided, ignore all other sources.
If the answer is not present in allowed sources, say so clearly.
Never hallucinate, speculate, or use external knowledge."

────────────────────────────────────────
API DESIGN
────────────────────────────────────────

Implement FastAPI endpoints:

- POST /chat
  - Inputs:
    - user_question
    - optional selected_text
    - session_id
  - Behavior:
    - Route to Selected-Text-Only mode OR RAG mode
    - Return answer + references (if applicable)

- POST /ingest
  - Triggers content ingestion pipeline
  - Protected or local-only endpoint

────────────────────────────────────────
FRONTEND INTEGRATION
────────────────────────────────────────

- Embed ChatKit UI inside Docusaurus
- Support:
  - Normal chat
  - Sending highlighted/selected text to backend
- Maintain lightweight session memory

────────────────────────────────────────
QUALITY & COMPLETENESS
────────────────────────────────────────

- Produce clean, documented, production-ready code
- Follow Spec-Kit Plus conventions
- Use clear folder structure
- Include README instructions for:
  - Environment variables
  - Qdrant setup
  - Neon Postgres setup
  - Running ingestion
  - Local development

Do NOT leave TODOs.
Do NOT provide pseudocode.
Deliver full implementation aligned with this specification.
```

## Response

Implemented a complete RAG chatbot system for the Physical AI Book with the following components:

### Backend (FastAPI)
- **Configuration**: Complete configuration management with environment variables
- **Models**: Pydantic models for chat requests/responses, content chunks, and sessions
- **Services**:
  - VectorDB service for Qdrant integration with embedding generation
  - MetadataDB service for Neon Postgres with proper schema
  - Chat service with both RAG and selected-text-only modes
  - Ingestion service for parsing and indexing Docusaurus content
- **Routes**: API endpoints for chat and ingestion with proper error handling
- **Utilities**: Content parser for Docusaurus markdown and text chunker with overlap

### Frontend
- **ChatWidget**: React component with text selection detection and chat interface
- **Integration**: Ready for embedding in Docusaurus with proper API communication

### Documentation
- **Setup Instructions**: Complete guide for installation and configuration
- **API Reference**: Detailed documentation for all endpoints
- **README**: Project overview and usage instructions

### Key Features Implemented
1. **Content Ingestion Pipeline**: Parses Docusaurus markdown files, chunks content by sections, stores embeddings in Qdrant and metadata in Neon Postgres
2. **Dual-Mode Chat**: Supports both RAG mode (vector search) and selected-text-only mode
3. **Hallucination Prevention**: Enforces strict adherence to book content with proper system prompts
4. **Reference Tracking**: Provides book section/chapter references when available
5. **Error Handling**: Proper responses when content isn't found or selected text is insufficient

The implementation follows all specified requirements with production-ready code, proper error handling, and comprehensive documentation.