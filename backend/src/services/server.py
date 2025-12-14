from collections.abc import AsyncIterator
from chatkit.server import ChatKitServer
from agents import Agent, Runner
from openai.types.responses import ResponseInputTextParam
from .stores import NeonStores  # Relative import since both are in services
from .qdrant_service import QdrantService
import logging
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
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
    document_sources: List[Dict[str, Any]] = []
    retrieval_info: Dict[str, Any] = {}

class CustomChatKitServer(ChatKitServer):
    """
    Custom ChatKit server implementation extending the base ChatKitServer class.
    """

    def __init__(self, agent: Agent, stores: NeonStores, qdrant_service: QdrantService):
        """
        Initialize the custom ChatKit server with the provided agent, stores, and optional Qdrant service.

        Args:
            agent: The OpenAI Agent to use for processing messages
            stores: The stores implementation for message persistence
            qdrant_service: Optional Qdrant service for document retrieval (enables RAG functionality)
        """
        super().__init__(store=stores)
        self.agent = agent
        self.qdrant_service = qdrant_service
        self.converter = ThreadItemConverter()
        logger.info("CustomChatKitServer initialized")


    async def respond(self, thread: ThreadMetadata, input_user_message: UserMessageItem | None, context: dict) -> AsyncIterator:
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        logger.debug("Context: %s", context)

        # Extract page context from request headers if available
        page_context = ""
        request = context.get("request") if context else None
        if request:
            page_path = request.headers.get("x-page-path", "")
            page_heading = request.headers.get("x-page-heading", "")
            selected_text = request.headers.get("x-selected-text", "")
            context_source = request.headers.get("x-context-source", "")

            if page_path or page_heading:
                page_context = f"User is currently on page: '{page_path or 'Unknown path'}'\nPage heading: '{page_heading or 'No heading'}'"

            if selected_text:
                page_context += f"\nContext from page selection: '{selected_text}'"

            print(f"[Server] Page context extracted: {page_context}")

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

        # Retrieve relevant document chunks if Qdrant service is available
        document_context = ""
        document_sources = []
        retrieval_info = {
            "chunks_retrieved": 0,
            "query_processed": False,
            "fallback_mode": False
        }

        if self.qdrant_service:
            try:
                # Check if Qdrant service is healthy before attempting search
                is_healthy = await self.qdrant_service.health_check()
                if not is_healthy:
                    logger.warning("Qdrant service is not healthy, using fallback mode")
                    retrieval_info["fallback_mode"] = True
                else:
                    # Extract the user's query from the input message for document search
                    user_query = ""
                    if input_user_message and hasattr(input_user_message, 'content') and input_user_message.content:
                        # Extract text content from message items
                        for content_item in input_user_message.content:
                            user_query = content_item.text
                            break

                    if user_query and len(user_query.strip()) > 3 and user_query.lower().strip() not in ["continue", "yes", ".", "ok", "?", "hello", "hi", "why", "how", "what"]:
                        print(f"[Server] Searching for relevant documents for query: {user_query[:100]}...")
                        relevant_chunks = await self.qdrant_service.search_relevant_chunks(user_query)

                        if relevant_chunks:
                            # Format document chunks as context for the AI agent
                            document_context_parts = ["Here are some relevant document excerpts that may help answer the user's query:"]

                            for i, chunk in enumerate(relevant_chunks):
                                source_info = f"Source: {chunk.get('file_name', chunk.get('source_file', 'Unknown'))}"
                                content = chunk.get('content', '')
                                similarity = chunk.get('similarity_score', 0)

                                document_context_parts.append(f"\n[Document Excerpt {i+1} - {source_info} (Relevance: {similarity:.2f})]")
                                document_context_parts.append(f"{content}")

                                # Add to document sources for response metadata
                                document_sources.append({
                                    "source_file": chunk.get('source_file', ''),
                                    "file_name": chunk.get('file_name', ''),
                                    "similarity_score": similarity,
                                    "content_preview": content[:100] + "..." if len(content) > 100 else content
                                })

                            document_context = "\n".join(document_context_parts)
                            retrieval_info["chunks_retrieved"] = len(relevant_chunks)
                            retrieval_info["query_processed"] = True
                            print(f"[Server] Retrieved {len(relevant_chunks)} relevant document chunks")
                        else:
                            print("[Server] No relevant documents found for the query")
                            # Add a notification that no relevant documents were found
                            document_context = "Note: I searched the documentation for information related to your query, but no relevant documents were found. I'll answer based on my general knowledge."
                            retrieval_info["fallback_mode"] = True  # Set fallback mode when no relevant docs found
                    else:
                        print("[Server] No user query found to search for relevant documents")
                        retrieval_info["fallback_mode"] = True
            except Exception as e:
                logger.error(f"Error retrieving document context: {str(e)}", exc_info=True)
                retrieval_info["fallback_mode"] = True  # Set fallback mode on error
                # Continue without document context to maintain fallback behavior
        else:
            # No Qdrant service provided, use fallback mode
            retrieval_info["fallback_mode"] = True

        if document_context:
            # Find the last user message and append document context
            for i in range(len(agent_input) - 1, -1, -1):
                if agent_input[i].get("role") == "user":
                    content = agent_input[i].get("content", [])
                    if isinstance(content, list):
                        doc_context: ResponseInputTextParam = {
                            "type": "input_text",
                            "text": f"\nDOCUMENT CONTEXT:\n{document_context}",
                        }
                        content.append(doc_context)
                    break


        if page_context:
            # Find the last user message and append context
            for i in range(len(agent_input) - 1, -1, -1):
                if agent_input[i].get("role") == "user":
                    content = agent_input[i].get("content", [])
                    if isinstance(content, list):
                        page_content: ResponseInputTextParam = {
                            "type": "input_text",
                            "text": f"PAGE CONTEXT:\n{page_context}",
                        }
                        content.append(page_content)
                    break

        print("Agnet input: ", agent_input)
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

        print("\n\n\n", result.new_items, "\n\n\n")
