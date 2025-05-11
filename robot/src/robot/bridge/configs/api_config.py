from pydantic_settings import BaseSettings


class APIConfig(BaseSettings):
    ORIGINS: list = ["http://localhost"]
    HOST: str = "127.0.0.1"
    PORT: int = 8383
