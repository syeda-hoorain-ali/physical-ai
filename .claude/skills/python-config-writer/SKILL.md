# Python Configuration Writer

This skill helps create proper Python project configuration files using Pydantic Settings, following industry best practices.

## Usage Instructions

When a user needs to create configuration files for a Python project, use this skill to generate:

1. A configuration class using Pydantic Settings
2. Proper environment variable loading
3. Type hints for configuration values
4. Default values where appropriate
5. Configuration initialization methods

## Best Practices to Follow

- Use `BaseSettings` from `pydantic_settings`
- Configure `SettingsConfigDict` with `env_file=".env"` to load from .env files
- Include proper type hints for all configuration fields
- Set sensible defaults for optional settings
- Use `case_sensitive=False` for more flexible environment variable matching
- Include `extra="ignore"` to prevent errors from extra environment variables
- Create a singleton settings instance
- Include proper imports for the required dependencies

## Template Structure

```python
from pydantic_settings import BaseSettings, SettingsConfigDict

class Config(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=".env",
        case_sensitive=False,
        extra="ignore",
    )

    # Define required and optional configuration fields here
    # required_field: str
    # optional_field: str = "default_value"

settings = Config()
```

## Common Configuration Categories

- API keys and credentials
- Database connection strings
- Server settings (host, port)
- Logging configuration
- Third-party service settings
- Feature flags and rate limits

## Output Requirements

1. Generate complete, working configuration files
2. Include proper error handling where appropriate
3. Add helpful comments explaining configuration sections
4. Follow the exact pattern shown in the template
5. Make sure the configuration is easily extensible
