from collections.abc import AsyncIterator
from chatkit.server import ChatKitServer
from agents import Agent, Runner
from .stores import NeonStores  # Relative import since both are in services
import logging
from pydantic import BaseModel
from typing import Optional
from chatkit.types import AssistantMessageItem, ThreadMetadata, UserMessageItem
from chatkit.agents import AgentContext, stream_agent_response, ThreadItemConverter


# Set up logging
logger = logging.getLogger(__name__)

class ChatRequest(BaseModel):
    """
    Request model for chat interactions.
    """
    message: str
    thread_id: Optional[str] = None

class ChatResponse(BaseModel):
    """
    Response model for chat interactions.
    """
    response: str
    thread_id: str
    timestamp: str

class CustomChatKitServer(ChatKitServer):
    """
    Custom ChatKit server implementation extending the base ChatKitServer class.
    """

    def __init__(self, agent: Agent, stores: NeonStores):
        """
        Initialize the custom ChatKit server with the provided agent and stores.

        Args:
            agent: The OpenAI Agent to use for processing messages
            stores: The stores implementation for message persistence
        """
        super().__init__(store=stores)
        self.agent = agent
        self.converter = ThreadItemConverter()
        logger.info("CustomChatKitServer initialized")


    async def respond(self, thread: ThreadMetadata, input_user_message: UserMessageItem | None, context: dict) -> AsyncIterator:
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Load all thread items and convert using ChatKit's converter
        page = await self.store.load_thread_items(thread.id, None, 100, "asc", context)
        all_items = list(page.data)

        # Add current input to the list if provided
        if input_user_message:
            all_items.append(input_user_message)

        print(f"[Server] Processing {len(all_items)} items for agent")

        # Convert thread items to agent input format using ChatKit's converter
        agent_input = await self.converter.to_agent_input(all_items) if all_items else []

        print(f"[Server] Converted to {len(agent_input)} agent input items")

        result = Runner.run_streamed(
            self.agent,
            agent_input,
            context=agent_context,
        )

        # Track ID mappings to ensure unique IDs (LiteLLM/Gemini may reuse IDs)
        id_mapping: dict[str, str] = {}

        async for event in stream_agent_response(agent_context, result):
            # Fix potential ID collisions from LiteLLM/Gemini
            if event.type == "thread.item.added":
                if isinstance(event.item, AssistantMessageItem):
                    old_id = event.item.id
                    # Generate unique ID if we haven't seen this response ID before
                    if old_id not in id_mapping:
                        new_id = self.store.generate_item_id("message", thread, context)
                        id_mapping[old_id] = new_id
                        print(f"[Server] Mapping ID {old_id} -> {new_id}")
                    event.item.id = id_mapping[old_id]
            elif event.type == "thread.item.done":
                if isinstance(event.item, AssistantMessageItem):
                    old_id = event.item.id
                    if old_id in id_mapping:
                        event.item.id = id_mapping[old_id]
            elif event.type == "thread.item.updated":
                if event.item_id in id_mapping:
                    event.item_id = id_mapping[event.item_id]

            yield event
