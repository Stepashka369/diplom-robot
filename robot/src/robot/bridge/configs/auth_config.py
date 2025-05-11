from fastapi.security import HTTPBearer
from keycloak import KeycloakOpenID
from pydantic_settings import BaseSettings


class AuthConfig(BaseSettings):
    KEYCLOAK_SERVER_URL: str = "http://127.0.0.1:8080"
    KEYCLOAK_REALM: str = "robot"
    KEYCLOAK_CLIENT_ID: str = "robot-backend"
    KEYCLOAK_CLIENT_SECRET: str = "jwVIllnkq0PrqSnp5n4P3sxVnmJ4pDv3"


auth_config = AuthConfig()
bearer_scheme = HTTPBearer(scheme_name="Robot Token")
keycloak_openid = KeycloakOpenID(
    server_url=auth_config.KEYCLOAK_SERVER_URL,
    realm_name=auth_config.KEYCLOAK_REALM,
    client_id=auth_config.KEYCLOAK_CLIENT_ID,
    client_secret_key=auth_config.KEYCLOAK_CLIENT_SECRET,
)
