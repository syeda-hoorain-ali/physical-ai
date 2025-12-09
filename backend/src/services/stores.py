import asyncpg
from typing import Optional, List, Dict, Any
import uuid
import logging
from chatkit.store import Store, StoreItemType
from chatkit.types import (
    ThreadMetadata, ThreadItem,
    Attachment, Page, InferenceOptions,
    UserMessageItem, AssistantMessageItem,
    AssistantMessageContent, UserMessageTextContent
)
from src.config import Config


# Set up logging
logger = logging.getLogger(__name__)

class NeonStores(Store[dict]):
    """
    Store implementations using Neon PostgreSQL database for message and attachment persistence.
    """

    def __init__(self):
        from src.config import settings
        self.connection_string = settings.neon_database_url
        if not self.connection_string:
            raise ValueError("NEON_DATABASE_URL environment variable is required")
        self.pool = None


    async def connect(self):
        """
        Establish connection to the Neon database.
        """
        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.connection_string,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            logger.info("Connected to Neon database successfully")

            # Initialize tables if they don't exist
            await self._initialize_tables()
        except Exception as e:
            logger.error(f"Failed to connect to Neon database: {e}")
            raise


    async def _initialize_tables(self):
        """
        Create required tables if they don't exist.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        async with self.pool.acquire() as connection:
            # Create threads table
            await connection.execute("""
                CREATE TABLE IF NOT EXISTS threads (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                    title TEXT
                )
            """)

            # Create messages table
            await connection.execute("""
                CREATE TABLE IF NOT EXISTS messages (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    content TEXT NOT NULL,
                    sender_type VARCHAR(10) NOT NULL CHECK (sender_type IN ('user', 'agent')),
                    thread_id UUID NOT NULL REFERENCES threads(id) ON DELETE CASCADE,
                    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW()
                )
            """)

            # Create indexes
            await connection.execute("""
                CREATE INDEX IF NOT EXISTS idx_messages_thread_id ON messages(thread_id)
            """)

            await connection.execute("""
                CREATE INDEX IF NOT EXISTS idx_messages_timestamp ON messages(timestamp)
            """)

            logger.info("Database tables initialized")


    # Implement the abstract methods from Store class
    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        """
        Load a thread by its ID.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        async with self.pool.acquire() as connection:
            row = await connection.fetchrow("""
                SELECT id, created_at, updated_at, title
                FROM threads
                WHERE id = $1
            """, thread_id)

            if not row:
                raise ValueError(f"Thread {thread_id} not found")

            # Create and return a ThreadMetadata object
            return ThreadMetadata(
                id=str(row['id']),
                created_at=row['created_at'],
                metadata={"updated_at": row['updated_at']},
                title=row['title'] or f"Thread {thread_id[:8]}"
            )


    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        """
        Save a thread.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        async with self.pool.acquire() as connection:
            await connection.execute("""
                INSERT INTO threads (id, title, created_at, updated_at)
                VALUES ($1, $2, $3, $4)
                ON CONFLICT (id) DO UPDATE SET
                    title = EXCLUDED.title,
                    updated_at = EXCLUDED.updated_at
            """, thread.id, thread.title, thread.created_at, thread.metadata.get("updated_at", thread.created_at) if hasattr(thread, 'metadata') and thread.metadata else thread.created_at)


    async def load_thread_items(self, thread_id: str, after: str | None, limit: int, order: str, context: dict) -> Page[ThreadItem]:
        """
        Load items for a specific thread.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        # Build the query based on parameters
        query = """
            SELECT id, content, sender_type, timestamp
            FROM messages
            WHERE thread_id = $1
        """
        params = [thread_id]
        param_index = 2

        if after:
            query += f" AND id > ${param_index}"
            params.append(after)
            param_index += 1

        # Apply ordering
        order_clause = "ASC" if order.lower() == "asc" else "DESC"
        query += f" ORDER BY timestamp {order_clause} LIMIT ${param_index}"
        params.append(limit)

        async with self.pool.acquire() as connection:
            rows = await connection.fetch(query, *params)

            # Convert to ThreadItem objects
            items = []
            for row in rows:
                # Create a ThreadItem (this is a simplified version)
                # In a real implementation, you'd need to create proper ThreadItem objects

                # Create a message item based on sender type
                if row['sender_type'] == 'user':
                    item = UserMessageItem(
                        id=str(row['id']),  # Ensure ID is a string
                        created_at=row['timestamp'],
                        thread_id=thread_id,
                        content=[UserMessageTextContent(type="input_text", text=row['content'])],
                        inference_options=InferenceOptions()
                    )
                else:  # sender_type is 'agent' or 'assistant'
                    item = AssistantMessageItem(
                        id=str(row['id']),  # Ensure ID is a string
                        created_at=row['timestamp'],
                        thread_id=thread_id,
                        content=[AssistantMessageContent(type="output_text", text=row['content'])]
                    )
                items.append(item)

            # Return a Page object
            from chatkit.types import Page
            return Page(data=items, has_more=len(items) == limit)


    async def save_attachment(self, attachment: Attachment, context: dict) -> None:
        """
        Save an attachment.
        """
        # For now, we'll just log this since we're not implementing file storage
        logger.info(f"Saving attachment: {attachment.id}")


    async def load_attachment(self, attachment_id: str, context: dict) -> Attachment:
        """
        Load an attachment by ID.
        """
        # For now, we'll return a placeholder since we're not implementing file storage
        logger.info(f"Loading attachment: {attachment_id}")
        raise ValueError(f"Attachment {attachment_id} not found")


    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        """
        Delete an attachment by ID.
        """
        logger.info(f"Deleting attachment: {attachment_id}")


    async def load_threads(self, limit: int, after: str | None, order: str, context: dict) -> Page[ThreadMetadata]:
        """
        Load multiple threads.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        query = "SELECT id, created_at, updated_at, title FROM threads"
        params = []
        param_index = 1

        if after:
            query += f" WHERE id > ${param_index}"
            params.append(after)
            param_index += 1

        # Apply ordering
        order_clause = "ASC" if order.lower() == "asc" else "DESC"
        query += f" ORDER BY created_at {order_clause} LIMIT ${param_index}"
        params.append(int(limit))

        async with self.pool.acquire() as connection:
            rows = await connection.fetch(query, *params)

            threads = []
            for row in rows:
                thread = ThreadMetadata(
                    id=str(row['id']),
                    created_at=row['created_at'],
                    metadata={"updated_at": row['updated_at']},
                    title=row['title'] or f"Thread {str(row['id'])[:8]}"
                )
                threads.append(thread)

            from chatkit.types import Page
            return Page(data=threads, has_more=len(rows) == limit)


    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """
        Add an item to a thread.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        # Extract content from the item based on its type
        content = ""

        # Handle different item types that have content
        if (item.type == "user_message" or item.type == "assistant_message") and item.content:
            for part in item.content:
                content += part.text

        # Determine sender type based on the item type
        if hasattr(item, 'role'):
            sender_type = "user" if getattr(item, 'role') == "user" else "agent"
        elif isinstance(item, UserMessageItem):
            sender_type = "user"
        elif isinstance(item, AssistantMessageItem):
            sender_type = "agent"
        else:
            # Default to agent for other item types
            sender_type = "agent"

        async with self.pool.acquire() as connection:
            await connection.execute("""
                INSERT INTO messages (id, thread_id, content, sender_type, timestamp)
                VALUES ($1, $2, $3, $4, $5)
            """, item.id, thread_id, content, sender_type, item.created_at or "NOW()")

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """
        Save an item to a thread (alternative to add_thread_item).
        """
        # This can be the same as add_thread_item for our implementation
        await self.add_thread_item(thread_id, item, context)

    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        """
        Load a specific item from a thread.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        async with self.pool.acquire() as connection:
            row = await connection.fetchrow("""
                SELECT id, content, sender_type, timestamp
                FROM messages
                WHERE thread_id = $1 AND id = $2
            """, thread_id, item_id)

            if not row:
                raise ValueError(f"Item {item_id} not found in thread {thread_id}")


            if row['sender_type'] == 'user':
                return UserMessageItem(
                    id=str(row['id']),  # Ensure ID is a string
                    created_at=row['timestamp'],
                    thread_id=thread_id,
                    content=[UserMessageTextContent(type="input_text", text=row['content'])],
                    inference_options=InferenceOptions()
                )
            else:  # sender_type is 'agent' or 'assistant'
                return AssistantMessageItem(
                    id=str(row['id']),  # Ensure ID is a string
                    created_at=row['timestamp'],
                    thread_id=thread_id,
                    content=[AssistantMessageContent(type="output_text", text=row['content'])]
                )

    def generate_thread_id(self, context: dict) -> str:
        """
        Generate a new thread ID.
        """
        import uuid
        return str(uuid.uuid4())

    def generate_item_id(self, item_type: StoreItemType, thread: ThreadMetadata, context: dict) -> str:
        """
        Generate a new item ID.
        """
        import uuid
        return str(uuid.uuid4())

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        """
        Delete a thread by its ID.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        async with self.pool.acquire() as connection:
            await connection.execute("""
                DELETE FROM threads WHERE id = $1
            """, thread_id)

        logger.info(f"Thread deleted: {thread_id}")

    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        """
        Delete a specific item from a thread.
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        async with self.pool.acquire() as connection:
            await connection.execute("""
                DELETE FROM messages WHERE thread_id = $1 AND id = $2
            """, thread_id, item_id)

        logger.info(f"Item {item_id} deleted from thread {thread_id}")


    # Helper methods for compatibility with original implementation
    async def save_message(self, thread_id: str, content: str, sender_type: str) -> str:
        """
        Save a message to the database.

        Args:
            thread_id: The ID of the thread the message belongs to
            content: The content of the message
            sender_type: The type of sender ('user' or 'agent')

        Returns:
            The ID of the saved message
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")


        message_id = str(uuid.uuid4())

        async with self.pool.acquire() as connection:
            await connection.execute("""
                INSERT INTO messages (id, thread_id, content, sender_type)
                VALUES ($1, $2, $3, $4)
            """, message_id, thread_id, content, sender_type)

        logger.info(f"Message saved: {message_id} in thread {thread_id}")
        return message_id


    async def get_thread_messages(self, thread_id: str) -> List[Dict[str, Any]]:
        """
        Get all messages for a specific thread.

        Args:
            thread_id: The ID of the thread to retrieve messages for

        Returns:
            List of messages in the thread
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")


        async with self.pool.acquire() as connection:
            rows = await connection.fetch("""
                SELECT id, content, sender_type, timestamp
                FROM messages
                WHERE thread_id = $1
                ORDER BY timestamp ASC
            """, thread_id)

            messages = []
            for row in rows:
                messages.append({
                    'id': str(row['id']),
                    'content': row['content'],
                    'sender_type': row['sender_type'],
                    'timestamp': row['timestamp']
                })

        logger.info(f"Retrieved {len(messages)} messages for thread {thread_id}")
        return messages


    async def create_thread(self, title: Optional[str] = None) -> str:
        """
        Create a new conversation thread.

        Args:
            title: Optional title for the thread

        Returns:
            The ID of the created thread
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")

        thread_id = str(uuid.uuid4())

        async with self.pool.acquire() as connection:
            await connection.execute("""
                INSERT INTO threads (id, title)
                VALUES ($1, $2)
            """, thread_id, title)

        logger.info(f"Thread created: {thread_id}")
        return thread_id


    async def get_thread(self, thread_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific thread by ID.

        Args:
            thread_id: The ID of the thread to retrieve

        Returns:
            Thread information or None if not found
        """
        if not self.pool:
            await self.connect()

        # Verify the pool is not None after connecting
        if not self.pool:
            raise RuntimeError("Failed to establish database connection pool")


        async with self.pool.acquire() as connection:
            row = await connection.fetchrow("""
                SELECT id, created_at, updated_at, title
                FROM threads
                WHERE id = $1
            """, thread_id)

            if row:
                return {
                    'id': str(row['id']),
                    'created_at': row['created_at'],
                    'updated_at': row['updated_at'],
                    'title': row['title']
                }

        return None


    async def close(self):
        """
        Close the database connection pool.
        """
        if self.pool:
            await self.pool.close()
            logger.info("Database connection pool closed")

