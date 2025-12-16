# API Contracts: Chatbot with OpenAI Agents

## Backend API (FastAPI)

### Session Management API
- **POST /api/chatkit/session** - Creates a new ChatKit session and returns client_secret
  - Input: None (uses authentication context)
  - Output: `{client_secret: string}`
  - Error responses: 500 for internal errors

### Chat API
- **POST /chat** - Send a message to the AI agent
  - Input: `{message: string}`
  - Output: `{response: string, timestamp: datetime}`
  - Error responses: 400 for invalid input, 429 for rate limits, 500 for internal errors

## Frontend API (Docusaurus Chat Widget)

### Chat Widget Integration
- **Client Secret Fetching**: Docusaurus frontend fetches client_secret from `/api/chatkit/session` to initialize ChatKit
- **Persistent UI**: Chat widget appears as a circle icon on every page via Root.tsx component
- **Real-time Communication**: ChatKit handles real-time message streaming when widget is opened
- **Session Management**: ChatKit manages conversation context and session state

## Data Flow

1. Docusaurus site loads with persistent chat widget (circle icon) on every page
2. User clicks chat widget icon to open chat interface
3. Chat widget requests client_secret from backend via `/api/chatkit/session`
4. Backend creates OpenAI ChatKit session and returns client_secret
5. Chat widget initializes ChatKit with client_secret
6. User sends message through chat interface
7. ChatKit sends message to backend via `/chat`
8. Backend processes with OpenAI Agents SDK
9. Response sent back to ChatKit and displayed to user in modal
