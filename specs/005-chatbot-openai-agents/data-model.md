# Data Model: Chatbot with OpenAI Agents

## Entities

### Message
- **id**: string (UUID) - Unique identifier for the message
- **content**: string - The text content of the message
- **sender_type**: Enum('user', 'agent') - Type of sender
- **thread_id**: string - ID of the conversation thread this message belongs to
- **timestamp**: datetime - When the message was sent

### Thread
- **id**: string (UUID) - Unique identifier for the conversation thread
- **created_at**: datetime - When the conversation was started
- **updated_at**: datetime - When the conversation was last updated
- **title**: string - Optional title for the conversation

## Validation Rules
- Messages must have content (non-empty string)
- Messages must have a valid sender_type
- Messages must belong to a valid thread
- Content moderation must be applied before processing user messages

## Database Schema (Neon PostgreSQL)
```sql
CREATE TABLE threads (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    title TEXT
);

CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    content TEXT NOT NULL,
    sender_type VARCHAR(10) NOT NULL CHECK (sender_type IN ('user', 'agent')),
    thread_id UUID NOT NULL REFERENCES threads(id) ON DELETE CASCADE,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_messages_thread_id ON messages(thread_id);
CREATE INDEX idx_messages_timestamp ON messages(timestamp);
```

## Notes
- Session state is managed by OpenAI Agents SDK internally
- Persistent storage implemented with Neon PostgreSQL database
- Conversation context maintained by OpenAI Agents framework with thread-based persistence
- Messages and threads are stored in Neon database for persistence across sessions
