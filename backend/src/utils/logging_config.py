import logging
import sys
from src.config import settings

def setup_logging():
    """
    Set up logging configuration for the application.
    """
    # Create a custom formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create a handler for stdout
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)

    # Get the root logger and configure it
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))
    root_logger.addHandler(handler)

    # Prevent duplicate logs if the root logger already has handlers
    root_logger.propagate = False

    # Configure specific loggers if needed
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("fastapi").setLevel(logging.WARNING)
    logging.getLogger("asyncio").setLevel(logging.WARNING)