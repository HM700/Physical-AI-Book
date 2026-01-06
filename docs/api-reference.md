# API Reference: Physical AI Book RAG Chatbot

## Base URL

The API base URL is: `http://localhost:8000/api/v1` (or your deployed URL)

## Authentication

No authentication required for this implementation. In production, consider adding authentication headers.

## Endpoints

### POST /chat

Get an answer to a question about the book content.

#### Request

**Content-Type**: `application/json`

**Body**:
```json
{
  "user_question": "string",
  "selected_text": "string | null",
  "session_id": "string | null"
}
```

**Parameters**:
- `user_question` (required): The question to ask about the book content
- `selected_text` (optional): Text selected by the user that should be used for "selected-text-only" mode
- `session_id` (optional): Session identifier for maintaining conversation context

#### Response

**Status Code**: `200 OK`

**Body**:
```json
{
  "answer": "string",
  "references": [
    {
      "title": "string",
      "url": "string",
      "section": "string",
      "chapter": "string | null"
    }
  ],
  "session_id": "string"
}
```

**Response Fields**:
- `answer`: The AI-generated answer to the question
- `references`: Array of book sections/chapters referenced in the answer
- `session_id`: Session identifier (new or existing)

#### Error Response

**Status Code**: `500 Internal Server Error`

**Body**:
```json
{
  "detail": "string"
}
```

#### Examples

**Normal RAG Query**:
```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "user_question": "What are the key concepts in chapter 1?",
    "session_id": "abc123"
  }'
```

**Selected Text Query**:
```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "user_question": "Explain this concept?",
    "selected_text": "The core concept is that physical AI systems must integrate perception, planning, and action in real-time.",
    "session_id": "abc123"
  }'
```

### POST /ingest

Ingest book content into the vector database.

#### Request

**Content-Type**: `application/json`

**Body**:
```json
{
  "book_path": "string"
}
```

**Parameters**:
- `book_path` (required): Path to the book content directory

#### Response

**Status Code**: `200 OK`

**Body**:
```json
{
  "status": "completed",
  "chunks_processed": "number"
}
```

**Response Fields**:
- `status`: Status of the ingestion process
- `chunks_processed`: Number of content chunks processed and stored

#### Error Response

**Status Code**: `500 Internal Server Error`

**Body**:
```json
{
  "detail": "string"
}
```

#### Example

```bash
curl -X POST http://localhost:8000/api/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "book_path": "./docs"
  }'
```

### GET /health

Check the health status of the API.

#### Response

**Status Code**: `200 OK`

**Body**:
```json
{
  "status": "healthy"
}
```

### GET /

Root endpoint that returns a welcome message.

#### Response

**Status Code**: `200 OK`

**Body**:
```json
{
  "message": "Physical AI Book RAG Chatbot API"
}
```

## Error Codes

| Status Code | Description |
|-------------|-------------|
| 200 | Success |
| 400 | Bad Request - Invalid request parameters |
| 422 | Unprocessable Entity - Request validation failed |
| 500 | Internal Server Error - Something went wrong on the server |

## Rate Limiting

This implementation does not include rate limiting. In production, implement rate limiting to prevent abuse.

## Request/Response Examples

### Successful Chat Response
```json
{
  "answer": "The book covers physical AI and humanoid robotics, focusing on hands-on learning with ROS 2, Gazebo, Unity, and NVIDIA Isaac.",
  "references": [
    {
      "title": "Physical AI Book Introduction",
      "url": "/introduction",
      "section": "What is Physical AI?",
      "chapter": "Chapter 1"
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

### No Content Found Response
```json
{
  "answer": "This question is not answered in the book.",
  "references": [],
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

### Selected Text Mode Response
```json
{
  "answer": "Based on the selected text, this concept refers to the integration of perception, planning, and action systems.",
  "references": [],
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```