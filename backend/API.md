# API Documentation: Chatbot with OpenAI Agents

## Base URL
All API endpoints are relative to `http://localhost:8000` (or your configured host/port).

## Endpoints

### Health Check
`GET /`

**Description:** Check the health status of the API service.

**Response:**
```json
{
  "status": "healthy",
  "service": "chatbot-api"
}
```

### Create Chat Session
`POST /api/chatkit/session`

**Description:** Create a new ChatKit session and return a client secret.

**Response:**
```json
{
  "client_secret": "session_client_secret"
}
```

### Send Chat Message (Non-Streaming)
`POST /chat`

**Description:** Send a message to the AI assistant and receive a complete response.

**Request Body:**
```json
{
  "message": "Your message here",
  "thread_id": "optional_thread_id" // Omit to start a new conversation
}
```

**Response:**
```json
{
  "response": "AI response here",
  "thread_id": "thread_id_used",
  "timestamp": "ISO 8601 timestamp"
}
```

### Send Chat Message (Streaming)
`POST /chat/stream`

**Description:** Send a message to the AI assistant and receive a streaming response via Server-Sent Events.

**Request Body:**
```json
{
  "message": "Your message here",
  "thread_id": "optional_thread_id" // Omit to start a new conversation
}
```

**Response (Server-Sent Events):**
- Initial event with thread ID: `{"thread_id": "..."}`
- Content chunks: `{"content": "..."}` (may be partial response)
- Completion event: `{"done": true, "response": "complete response"}`
- Error event: `{"error": "error message"}`

## Error Responses

### 400 Bad Request
Returned when the request body is invalid or missing required fields.

**Response:**
```json
{
  "error": {
    "type": "http_error",
    "message": "Error details"
  }
}
```

### 429 Too Many Requests
Returned when rate limiting is exceeded.

**Response:**
```json
{
  "error": {
    "type": "http_error",
    "message": "Rate limit exceeded. Please try again later."
  }
}
```

### 500 Internal Server Error
Returned when an unexpected error occurs on the server.

**Response:**
```json
{
  "error": {
    "type": "internal_error",
    "message": "An internal error occurred"
  }
}
```

## Rate Limiting
The API implements rate limiting to prevent abuse. The default limit is 100 requests per hour per IP address. This can be configured via environment variables.

## Content Moderation
All user messages are subject to content moderation to prevent harmful or inappropriate content from being processed.

## Thread Management
The system maintains conversation context using thread IDs. Clients can:
- Start a new conversation by omitting the `thread_id`
- Continue an existing conversation by providing a `thread_id`
- Start a fresh conversation by generating a new `thread_id`
