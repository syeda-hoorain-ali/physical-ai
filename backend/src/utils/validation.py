import re
from typing import Optional
from pydantic import BaseModel, validator, ValidationError
import logging

logger = logging.getLogger(__name__)

class MessageValidationResult:
    """
    Result of message validation containing validity status and potential errors.
    """
    def __init__(self, is_valid: bool, errors: list = []):
        self.is_valid = is_valid
        self.errors = errors or []

def validate_message_content(content: str) -> MessageValidationResult:
    """
    Validate the content of a message.

    Args:
        content: The message content to validate

    Returns:
        MessageValidationResult indicating validity and any errors
    """
    errors = []

    if not content or not content.strip():
        errors.append("Message content cannot be empty")
    elif len(content.strip()) < 1:
        errors.append("Message content is too short")
    elif len(content) > 10000:  # Limit message length to 10k characters
        errors.append("Message content is too long (max 10000 characters)")

    # Check for potentially harmful content (basic security check)
    harmful_patterns = [
        r'<script[^>]*>.*?</script>',  # Script tags
        r'javascript:',               # JavaScript URLs
        r'vbscript:',                # VBScript URLs
        r'on\w+\s*=',                # Event handlers
    ]

    for pattern in harmful_patterns:
        if re.search(pattern, content, re.IGNORECASE):
            errors.append("Message contains potentially harmful content")
            break

    return MessageValidationResult(len(errors) == 0, errors)

def validate_thread_id(thread_id: str) -> MessageValidationResult:
    """
    Validate the thread ID.

    Args:
        thread_id: The thread ID to validate

    Returns:
        MessageValidationResult indicating validity and any errors
    """
    errors = []

    if not thread_id:
        errors.append("Thread ID cannot be empty")
    elif not isinstance(thread_id, str):
        errors.append("Thread ID must be a string")
    elif len(thread_id) < 1:
        errors.append("Thread ID is too short")
    elif len(thread_id) > 100:  # Reasonable limit for thread ID
        errors.append("Thread ID is too long")

    # Check if it looks like a valid UUID (basic check)
    uuid_pattern = r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$'
    if not re.match(uuid_pattern, thread_id, re.IGNORECASE):
        # It's not a UUID, which is fine as long as it meets other criteria
        pass

    return MessageValidationResult(len(errors) == 0, errors)

def validate_sender_type(sender_type: str) -> MessageValidationResult:
    """
    Validate the sender type.

    Args:
        sender_type: The sender type to validate

    Returns:
        MessageValidationResult indicating validity and any errors
    """
    errors = []

    if not sender_type:
        errors.append("Sender type cannot be empty")
    elif sender_type not in ['user', 'agent']:
        errors.append("Sender type must be either 'user' or 'agent'")

    return MessageValidationResult(len(errors) == 0, errors)

def validate_message(content: str, thread_id: str, sender_type: str) -> MessageValidationResult:
    """
    Validate a complete message with content, thread ID, and sender type.

    Args:
        content: The message content
        thread_id: The thread ID
        sender_type: The sender type

    Returns:
        MessageValidationResult indicating validity and any errors
    """
    errors = []

    # Validate each component
    content_result = validate_message_content(content)
    thread_result = validate_thread_id(thread_id)
    sender_result = validate_sender_type(sender_type)

    # Combine all errors
    errors.extend(content_result.errors)
    errors.extend(thread_result.errors)
    errors.extend(sender_result.errors)

    return MessageValidationResult(len(errors) == 0, errors)

def moderate_content(content: str) -> MessageValidationResult:
    """
    Moderate content to check for inappropriate or harmful content.

    Args:
        content: The content to moderate

    Returns:
        MessageValidationResult indicating if content is appropriate
    """
    errors = []

    # Check for potentially harmful content
    harmful_patterns = [
        r'drop\s+table',  # SQL injection attempts
        r'exec\s*\(',     # Command execution
        r'eval\s*\(',     # Code evaluation
        r'import\s+os',   # System command access
        r'import\s+subprocess',  # System command access
    ]

    for pattern in harmful_patterns:
        if re.search(pattern, content, re.IGNORECASE):
            errors.append("Content contains potentially harmful commands")
            break

    # Check for common spam patterns
    spam_patterns = [
        r'(viagra|cialis|porn|casino|gambling)',  # Adult content
        r'(make\s+money|get\s+rich|click\s+here)',  # Spam phrases
        r'(congratulations.*winner|free\s+\w+)',  # Scam patterns
    ]

    for pattern in spam_patterns:
        if re.search(pattern, content, re.IGNORECASE):
            errors.append("Content contains potential spam or inappropriate material")
            break

    return MessageValidationResult(len(errors) == 0, errors)


def sanitize_message_content(content: str) -> str:
    """
    Sanitize message content to remove potentially harmful elements.

    Args:
        content: The content to sanitize

    Returns:
        Sanitized content string
    """
    if not content:
        return content

    # Remove script tags (case insensitive)
    content = re.sub(r'<script[^>]*>.*?</script>', '', content, flags=re.IGNORECASE)

    # Remove javascript: and vbscript: protocols
    content = re.sub(r'javascript:', '', content, flags=re.IGNORECASE)
    content = re.sub(r'vbscript:', '', content, flags=re.IGNORECASE)

    # Remove event handlers (basic approach)
    content = re.sub(r'on\w+\s*=\s*["\'][^"\']*["\']', '', content, flags=re.IGNORECASE)

    return content.strip()