# Quickstart Guide: Chatbot with OpenAI Agents

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- OpenAI API key
- Git

## Setup

### Backend (Python/FastAPI)

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install uv package manager** (if not already installed)
   ```bash
   # Install uv via the official installer
   curl -LsSf https://astral.sh/uv/install.sh | sh
   # Or on Windows:
   powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
   ```

3. **Set up project with uv**
   ```bash
   cd backend
   # uv automatically creates and manages the virtual environment
   uv sync  # Creates virtual environment and installs dependencies from pyproject.toml
   ```

4. **Set up environment variables**
   Create a `.env` file in the project root:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   ```

5. **Run the backend server**
   ```bash
   uv run uvicorn main:app --reload --port 8000
   # Or activate the environment first and run directly:
   # source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   # uvicorn main:app --reload --port 8000
   ```

### Frontend (Docusaurus Documentation Site with Chat Widget)

1. **Navigate to the Docusaurus project**
   ```bash
   cd book-source
   ```

2. **Install Node.js dependencies**
   ```bash
   npm install
   ```

3. **Set up environment variables**
   Create a `.env` file in the book-source root:
   ```env
   REACT_APP_API_URL=http://localhost:8000
   ```

4. **Run the Docusaurus development server**
   ```bash
   npm run start
   # Runs the site in development mode at http://localhost:3000
   ```

5. **Build for production**
   ```bash
   npm run build
   # Builds the static website to the "build" folder
   ```

6. **Serve the production build locally**
   ```bash
   npm run serve
   # Serves the built website locally for testing
   ```

## API Endpoints

- `GET /` - Health check
- `POST /api/chatkit/session` - Create a new ChatKit session
- `POST /chat` - Send a message to the chatbot

## Usage

1. The application will be available at `http://localhost:3000`
2. The backend API will be available at `http://localhost:8000`
3. Start a conversation by typing in the chat interface
4. The chatbot will respond using OpenAI Agents

## Configuration

- Modify agent instructions in `backend/agent.py`
- Customize chat widget in `book-source/src/components/chat-widget/`
- Add persistent chat functionality via `book-source/src/theme/Root.tsx`
- Configure Docusaurus settings in `book-source/docusaurus.config.ts`

## Testing

### Backend
```bash
cd backend
uv run pytest  # Run tests in the managed environment
# Or activate the environment first:
# source .venv/bin/activate  # On Windows: .venv\Scripts\activate
# pytest
```

### Frontend
```bash
cd book-source
npm test
```