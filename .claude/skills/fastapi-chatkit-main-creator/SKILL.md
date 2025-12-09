# FastAPI ChatKit Main Application Creator

This skill helps create the main FastAPI application that connects all components: agents, stores, and servers, following best practices for a complete ChatKit implementation.

## Usage Instructions

When a user needs to create a main FastAPI application that connects agents, stores, and servers, use this skill to generate:

1. FastAPI application setup with proper middleware
2. Store initialization for database persistence
3. Custom ChatKit server with agent integration
4. Chat endpoint with proper request/response handling
5. Health check and debugging endpoints
6. Proper logging configuration

## How Components Connect

The main application connects all components in this flow:
1. **Agent** (from `.agent` module) → AI processing logic
2. **Stores** (from `.services.stores` module) → Database persistence
3. **Server** (from `.services.server` module) → ChatKit interface and business logic
4. **FastAPI** → HTTP interface and routing

## Best Practices to Follow

- Initialize stores first, then the server with the stores and agent
- Set up proper CORS middleware for web applications
- Use proper logging configuration
- Handle both streaming and non-streaming responses from ChatKit
- Include health check endpoints for monitoring
- Add debugging endpoints for development
- Use relative imports for internal modules
- Handle exceptions gracefully in endpoints

## Template Structure

### Complete Main Application Template:
```python
import logging
from chatkit.server import StreamingResult

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response, StreamingResponse
from .agent import agent  # Import your agent
from .services.server import CustomChatKitServer  # Import your custom server
from .services.stores import YourStores  # Import your stores implementation
from .utils.logging_config import setup_logging  # Import logging setup

# Set up logging
setup_logging()
logger = logging.getLogger(__name__)

# Initialize stores for message and attachment persistence
stores = YourStores()

# Create the ChatKit server with custom implementation
server = CustomChatKitServer(
    agent=agent,
    stores=stores
)

# Initialize FastAPI app
app = FastAPI(title="Your ChatKit Application")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    """
    Main ChatKit endpoint that processes chat requests.
    Handles both streaming and non-streaming responses.
    """
    result = await server.process(await request.body(), {})
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")

@app.get("/health")
async def health():
    """
    Health check endpoint to verify the service is running.
    """
    return {"status": "ok", "model": "your-model-name"}

@app.get("/debug/threads")
async def debug_threads():
    """
    Debug endpoint to view all stored threads from database.
    NOTE: This should be protected in production environments.
    """
    try:
        # Load all threads from the database
        page_of_threads = await stores.load_threads(limit=100, after=None, order="asc", context={})

        result = {}
        for thread in page_of_threads.data:
            # Get messages for each thread
            messages = await stores.get_thread_messages(thread.id)

            result[thread.id] = {
                "thread": {
                    "id": thread.id,
                    "created_at": str(thread.created_at),
                    "title": thread.title
                },
                "messages": messages,
                "message_count": len(messages)
            }

        return result
    except Exception as e:
        logger.error(f"Error in debug_threads endpoint: {e}")
        return {"error": str(e)}

# Optional: Serve frontend if it exists
# from pathlib import Path
# from fastapi.staticfiles import StaticFiles
#
# ROOT_DIR = Path(__file__).resolve().parent.parent
# FRONTEND_DIR = ROOT_DIR / "frontend" / "dist"
# if FRONTEND_DIR.exists():
#     app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")

if __name__ == "__main__":
    import uvicorn
    print("Starting Your ChatKit Application server at http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)

# Run with: uv run uvicorn src.main:app --port 8000 --reload
```

## Key Connection Points

- **Agent Integration**: The agent is passed to the CustomChatKitServer to process messages
- **Store Integration**: The stores implementation is passed to both the server and used directly in debug endpoints
- **Server Integration**: The CustomChatKitServer processes requests and uses both agent and stores
- **FastAPI Integration**: The server's process method is called from the FastAPI endpoint

## Common Application Patterns

- API Gateway pattern with FastAPI as the entry point
- Dependency injection of stores and agents into the server
- Streaming response handling for real-time chat
- Health checks for container orchestration
- Debug endpoints for development and monitoring

## Security Considerations

- Protect debug endpoints in production
- Validate CORS origins appropriately
- Add authentication/authorization if needed
- Sanitize inputs before processing
- Implement rate limiting if necessary

## Output Requirements

1. Generate complete, working main application that connects all components
2. Include proper initialization of stores, server, and agent
3. Add appropriate endpoints for chat, health, and debugging
4. Include proper response handling for both streaming and static responses
5. Follow the exact patterns shown in the template
