from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    FASTAPI_HOST: str = "127.0.0.1"
    FASTAPI_PORT: int = 8088
    ROS_NODE_NAME: str = "ros2_fastapi_bridge_node"
    
    # Keycloak configuration
    KEYCLOAK_URL: str = "http://localhost:8080/auth"
    KEYCLOAK_REALM: str = "ros-bridge"
    KEYCLOAK_CLIENT_ID: str = "ros-frontend"
    KEYCLOAK_CLIENT_SECRET: str = "8C9oBrd6yOBHzei3vrHey59aq1ECqKog"
    
    # Single role for all access
    ACCESS_ROLE: str = "ros-full-access"
    
    # ROS topics configuration
    TOPICS: dict = {
        "incoming": {
            "temperature": "/robot/temperature",
            "status": "/robot/status"
        },
        "outgoing": {
            "control": "/robot/control",
            "command": "/robot/command"
        }
    }

    class Config:
        env_file = ".env"

config = Settings()