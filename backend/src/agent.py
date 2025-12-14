from agents import Agent, ModelSettings, function_tool
from typing import List, Dict, Any
import src.config
from .services.qdrant_service import QdrantService


# Initialize Qdrant service using the same configuration as in main.py
qdrant_service = QdrantService(
    host=src.config.settings.QDRANT_HOST,
    port=src.config.settings.QDRANT_PORT,
    collection_name=src.config.settings.QDRANT_COLLECTION_NAME,
    api_key=src.config.settings.QDRANT_API_KEY
)


@function_tool
async def search_documentation(query: str, limit: int = 5, min_similarity: float = 0.3) -> List[Dict[str, Any]]:
    """Search the documentation database for relevant information based on semantic similarity.

    This tool allows the agent to retrieve relevant document chunks from the vector database
    to provide contextually accurate responses based on the project's documentation.

    Args:
        query (str): The search query to find relevant documents
        limit (int, optional): Maximum number of chunks to retrieve. Defaults to 5.
        min_similarity (float, optional): Minimum similarity threshold for results. Defaults to 0.3.

    Returns:
        List[Dict[str, Any]]: A list of document chunks with metadata including content,
        source file, similarity score, and other relevant information
    """
    try:
        # Call the Qdrant service to search for relevant chunks
        results = await qdrant_service.search_relevant_chunks(
            query=query,
            limit=limit,
            min_similarity=min_similarity
        )

        # Return the results
        return results
    except Exception as e:
        # Return an error message if the search fails
        return [{
            "error": f"Failed to search documentation: {str(e)}",
            "content": "",
            "source_file": "",
            "file_name": "",
            "chunk_index": 0,
            "similarity_score": 0.0
        }]


agent = Agent(
    name="Physical AI Assistant",
    model="gemini-2.5-flash",
    tools=[search_documentation],  # Add the search tool to the agent
    model_settings=ModelSettings(tool_choice="required"),
    instructions="""You are a helpful AI assistant for a physical AI and humanoid robotics educational platform.
    Your role is to assist users with questions about AI, robotics, computer vision, machine learning,
    and related topics. Provide clear, accurate, and helpful responses. Keep responses concise but informative.
    If you don't know something, say so honestly. Be friendly and professional in your interactions.

    IMPORTANT: When you have access to documentation through the search_documentation tool, use it to provide more accurate and detailed responses. If a user asks about specific topics related to the content, search the documentation first to provide relevant information. When page context is provided (indicated by 'PAGE CONTEXT:' in the input), pay special attention to the user's current location on the website. Use this context to provide more relevant and targeted responses based on the page they're viewing. Consider how the page content relates to their question and tailor your response accordingly.""",
    # description="An AI assistant for the Physical AI educational platform"
)
