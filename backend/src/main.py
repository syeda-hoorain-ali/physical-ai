import logging
from chatkit.server import StreamingResult

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response, StreamingResponse
from .agent import agent
from .config import settings
from .services.server import CustomChatKitServer
from .services.stores import NeonStores
from .services.qdrant_service import QdrantService
from .utils.logging_config import setup_logging

# Set up logging
setup_logging()
logger = logging.getLogger(__name__)

# Initialize stores for message and attachment persistence
stores = NeonStores()

# Initialize Qdrant service
qdrant_host = settings.QDRANT_HOST
qdrant_port = settings.QDRANT_PORT
qdrant_collection_name = settings.QDRANT_COLLECTION_NAME
qdrant_api_key = settings.QDRANT_API_KEY

# Handle the case where API key might be an empty string
if qdrant_api_key == "":
    qdrant_api_key = None

qdrant_service = QdrantService(
    host=qdrant_host,
    port=qdrant_port,
    collection_name=qdrant_collection_name,
    api_key=qdrant_api_key
)

# Create the ChatKit server with custom implementation
server = CustomChatKitServer(
    agent=agent,
    stores=stores,
    qdrant_service=qdrant_service
)


# Initialize FastAPI app
app = FastAPI(title="Physical AI Chatbot")

origins = [
    "http://localhost",
    "http://localhost:3000",
    "http://localhost:8080",
    settings.FRONTEND_BASE_URL,
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def home_page():
    return {"message": "Welcome to Physical AI Chatbot! Visit /docs for API documentation."}


@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    payload = await request.body()
    result = await server.process(payload, {"request": request})
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")


@app.get("/health")
async def health():
    return {"status": "ok", "model": "gemini-2.5-flash"}

if __name__ == "__main__":
    import uvicorn
    print("Starting ChatKit Gemini server at http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)

# uv run uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
