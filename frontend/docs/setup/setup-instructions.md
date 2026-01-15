# Setup Instructions: Physical AI Book RAG Chatbot

## Prerequisites

- Python 3.11+
- Node.js (for frontend development)
- OpenAI API account with sufficient credits
- Qdrant Cloud account (free tier available)
- Neon Postgres serverless database

## Backend Setup

### 1. Environment Configuration

Create a `.env` file in the `backend` directory with the following variables:

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# Database Configuration
DATABASE_URL=your_neon_postgres_connection_string_here

# Optional Configuration
EMBEDDING_MODEL=text-embedding-ada-002
```

### 2. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 3. Run the Application

```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`

## Frontend Integration

### 1. Integrate with Docusaurus

To embed the chat widget in your Docusaurus site:

1. Add the ChatWidget component to your Docusaurus pages
2. Make sure the frontend can communicate with your backend API
3. The default API endpoint is `http://localhost:8000/api/v1/chat`

### 2. Text Selection Feature

The chat widget automatically detects selected text on the page and offers to use it for specific questions.

## Content Ingestion

### 1. Prepare Your Content

Ensure your Docusaurus markdown files are in the `docs` directory structure.

### 2. Run Ingestion

To ingest your book content into the vector database:

```bash
# Using the API directly
curl -X POST http://localhost:8000/api/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{"book_path": "./docs"}'
```

## API Endpoints

### Chat Endpoint
```
POST /api/v1/chat
```

Request body:
```json
{
  "user_question": "string",
  "selected_text": "string", // optional
  "session_id": "string" // optional
}
```

Response:
```json
{
  "answer": "string",
  "references": [
    {
      "title": "string",
      "url": "string",
      "section": "string",
      "chapter": "string" // optional
    }
  ],
  "session_id": "string"
}
```

### Ingestion Endpoint
```
POST /api/v1/ingest
```

Request body:
```json
{
  "book_path": "string"
}
```

Response:
```json
{
  "status": "completed",
  "chunks_processed": "number"
}
```

## Testing

### 1. Verify Backend

Check if the service is running:
```bash
curl http://localhost:8000/health
```

### 2. Test Chat Functionality

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question": "What is this book about?", "session_id": "test-session"}'
```

## Troubleshooting

### Common Issues

1. **Environment Variables Not Set**: Ensure all required environment variables are set
2. **Database Connection Issues**: Check your Neon Postgres connection string
3. **Qdrant Connection Issues**: Verify your Qdrant URL and API key
4. **OpenAI API Issues**: Confirm your API key is valid and has sufficient credits

### Logging

The application logs to stdout by default. Check your terminal for any error messages.

## Security Considerations

- Never commit your `.env` file to version control
- Use proper CORS configuration in production
- Implement rate limiting for API endpoints
- Consider adding authentication for sensitive operations