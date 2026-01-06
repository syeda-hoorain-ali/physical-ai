---
title: Physical AI Backend
emoji: 🤖
colorFrom: blue
colorTo: red
sdk: docker
pinned: true
python_version: 3.12
app_port: 8000
short_description: 
---

# Backend Service for Physical AI Chat Application

This backend service provides a ChatKit-compatible server with Neon PostgreSQL database integration for message and thread persistence. It's designed to work with OpenAI agents and Gemini models for conversational AI applications.

## Features

- **ChatKit Compatible API**: Implements the ChatKit server interface for compatibility with ChatKit clients
- **Neon PostgreSQL Database**: Uses Neon as the database backend for storing conversations and messages
- **OpenAI/Gemini Integration**: Supports both OpenAI and Google Gemini models
- **Thread Management**: Full CRUD operations for conversation threads
- **Message Persistence**: Stores user and assistant messages in the database
- **Real-time Streaming**: Supports streaming responses for better user experience

## Prerequisites

- Python 3.12+
- Neon PostgreSQL database account
- Gemini API key
- `uv` package manager (recommended) or `pip`

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd backend
   ```

2. Install dependencies using uv (recommended):
   ```bash
   uv sync
   ```

   Or using pip:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables by creating a `.env` file in the root of the backend directory:
   ```env
   GEMINI_API_KEY="your-gemini-api-key-here"
   neon_database_url="postgresql://username:password@region.aws.neon.tech/dbname?sslmode=require"
   HOST="0.0.0.0"
   PORT="8000"
   LOG_LEVEL="INFO"
   ```

## Configuration

The application uses Pydantic Settings for configuration management. Key settings include:

- `GEMINI_API_KEY`: Your Google Gemini API key
- `neon_database_url`: Connection string for Neon PostgreSQL database
- `HOST`: Host address to bind the server to (default: 0.0.0.0)
- `PORT`: Port number for the server (default: 8000)
- `LOG_LEVEL`: Logging level (default: INFO)

## Running the Application

### Using uv (recommended):
```bash
uv run python -m src.main
```

### Using uvicorn directly:
```bash
uvicorn src.main:app --port 8000 --reload
```

### Using the main script:
```bash
python -m src.main
```

The server will start at `http://localhost:8000` (or the configured host/port).

## API Endpoints

- `POST /chatkit`: Main ChatKit endpoint for chat interactions
- `GET /health`: Health check endpoint
- `GET /debug/threads`: Debug endpoint to view stored threads (requires appropriate authorization)

## Database Schema

The application automatically creates the necessary tables on startup:

- `threads`: Stores conversation thread metadata
  - id (UUID, primary key)
  - created_at (TIMESTAMP WITH TIME ZONE)
  - updated_at (TIMESTAMP WITH TIME ZONE)
  - title (TEXT)

- `messages`: Stores individual messages
  - id (UUID, primary key)
  - content (TEXT)
  - sender_type (VARCHAR - 'user' or 'agent')
  - thread_id (UUID, foreign key to threads)
  - timestamp (TIMESTAMP WITH TIME ZONE)

## Environment Variables

| Variable            | Description                                    | Required              |
|---------------------|------------------------------------------------|-----------------------|
| `GEMINI_API_KEY`    | Your Google Gemini API key                     | Yes                   |
| `NEON_DATABASE_URL` | Connection string for Neon PostgreSQL database | Yes                   |
| `HOST`              | Server host                                    | No (default: 0.0.0.0) |
| `PORT`              | Server port                                    | No (default: 8000)    |
| `LOG_LEVEL`         | Logging level                                  | No (default: INFO)    |

## Architecture

The backend consists of:
- `src/services/stores.py`: NeonStores class implementing the ChatKit Store interface
- `src/services/server.py`: CustomChatKitServer extending the base ChatKit server
- `src/agent.py`: AI agent implementation using Gemini
- `src/main.py`: FastAPI application with ChatKit integration

## Development

For development, you can run the server with auto-reload:

```bash
uv run uvicorn src.main:app --port 8000 --reload
```

## Troubleshooting

1. **Database Connection Issues**: Verify that your Neon database URL is correct and that you have network access to the database.

2. **API Key Issues**: Ensure your GEMINI_API_KEY is valid and has the necessary permissions.

3. **Port Already in Use**: If you get an address already in use error, change the PORT in your .env file or stop the other process using the port.

4. **SSL/TLS Issues**: Some Neon configurations require specific SSL settings in the connection string.

## Markdown to Qdrant Vector Conversion Tool

This backend also includes a tool to convert markdown files to vector embeddings for semantic search capabilities.

### Installation

The markdown to qdrant tool requires these additional dependencies:
- qdrant-client[fastembed]
- markdown
- beautifulsoup4

Install with:
```bash
uv sync
```

### Usage

```bash
python scripts/markdown_to_qdrant.py --source-dir <path-to-markdown-directory> --collection-name <qdrant-collection-name> [options]
```

### Command Line Options

- `--source-dir`: Directory containing markdown files to convert (required)
- `--collection-name`: Qdrant collection name to store vectors (required)
- `--qdrant-host`: Qdrant host (default: localhost)
- `--qdrant-port`: Qdrant port (default: 6333)
- `--qdrant-api-key`: Qdrant API key (optional)
- `--chunk-size`: Size of text chunks (default: 1000)
- `--overlap`: Overlap between chunks (default: 100)
- `--dry-run`: Process files without uploading to Qdrant (for testing)

### Examples

1. Basic usage:
```bash
python scripts/markdown_to_qdrant.py --source-dir ./docs --collection-name my_docs
```

2. With custom chunk size and overlap:
```bash
python scripts/markdown_to_qdrant.py --source-dir ./docs --collection-name my_docs --chunk-size 512 --overlap 50
```

3. Dry run for testing:
```bash
python scripts/markdown_to_qdrant.py --source-dir ./docs --collection-name my_docs --dry-run
```
