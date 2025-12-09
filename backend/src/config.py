from openai import AsyncOpenAI
from pydantic_settings import BaseSettings, SettingsConfigDict
from agents import set_default_openai_client, set_default_openai_api, set_tracing_disabled

class Config(BaseSettings):
    model_config = SettingsConfigDict(
        env_file = ".env",
        case_sensitive = False,
        extra="ignore",
    )

    # Gemini API
    gemini_api_key: str
    gemini_base_url: str = "https://generativelanguage.googleapis.com/v1beta/openai/"

    # Neon database settings
    neon_database_url: str

    # Server settings
    HOST: str = "0.0.0.0"
    PORT: int = 8000

    # Logging settings
    LOG_LEVEL: str = "INFO"

    # Rate limiting settings (if implemented later)
    RATE_LIMIT_REQUESTS: int = 100
    RATE_LIMIT_WINDOW: int = 3600  # in seconds


    def initialize_gemini_model(self):
        gemini_client = AsyncOpenAI(api_key=self.gemini_api_key, base_url=self.gemini_base_url)
        set_default_openai_client(gemini_client)
        set_default_openai_api("chat_completions")
        set_tracing_disabled(disabled=True)


settings = Config()
settings.initialize_gemini_model()
