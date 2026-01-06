# Physical AI Book RAG Chatbot Backend

This is the backend API for the Physical AI Book RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about the book content.

## Features

- **RAG-based Q&A**: Ask questions about the Physical AI Book content
- **Selected Text Mode**: Ask questions specifically about selected text on the page
- **Content Ingestion**: Automatically parse and index Docusaurus markdown files
- **Reference Tracking**: Answers include references to specific book sections
- **Hallucination Prevention**: Answers are grounded only in book content

## Tech Stack

- **Backend**: Python, FastAPI
- **AI Framework**: OpenAI API
- **Vector Database**: Qdrant Cloud
- **Metadata Database**: Neon Postgres
- **Embeddings**: OpenAI text-embedding model

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables (see `.env.example`)

3. Run the application:
   ```bash
   uvicorn app.main:app --reload --port 8000
   ```

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /api/v1/chat` - Chat endpoint for Q&A
- `POST /api/v1/ingest` - Content ingestion endpoint

## Environment Variables

- `OPENAI_API_KEY` - Your OpenAI API key
- `QDRANT_URL` - Qdrant Cloud URL
- `QDRANT_API_KEY` - Qdrant API key
- `DATABASE_URL` - Neon Postgres connection string
- `EMBEDDING_MODEL` - OpenAI embedding model (default: text-embedding-ada-002)

## Usage

1. Ingest your book content:
   ```bash
   curl -X POST http://localhost:8000/api/v1/ingest -H "Content-Type: application/json" -d '{"book_path": "./docs"}'
   ```

2. Ask questions about the content:
   ```bash
   curl -X POST http://localhost:8000/api/v1/chat -H "Content-Type: application/json" -d '{"user_question": "What is this book about?"}'
   ```

For detailed setup instructions, see [docs/setup-instructions.md](docs/setup-instructions.md).

For API reference, see [docs/api-reference.md](docs/api-reference.md).