from fastapi import Depends, HTTPException, status, WebSocket
from fastapi.security import OAuth2AuthorizationCodeBearer
from keycloak import KeycloakOpenID
from keycloak.exceptions import KeycloakError

from .config import config

# Keycloak client setup
keycloak_openid = KeycloakOpenID(
    server_url=config.KEYCLOAK_URL,
    client_id=config.KEYCLOAK_CLIENT_ID,
    realm_name=config.KEYCLOAK_REALM,
    client_secret_key=config.KEYCLOAK_CLIENT_SECRET,
    verify=True
)

# OAuth2 scheme for FastAPI
oauth2_scheme = OAuth2AuthorizationCodeBearer(
    authorizationUrl=f"{config.KEYCLOAK_URL}/realms/{config.KEYCLOAK_REALM}/protocol/openid-connect/auth",
    tokenUrl=f"{config.KEYCLOAK_URL}/realms/{config.KEYCLOAK_REALM}/protocol/openid-connect/token"
)

async def verify_token(token: str):
    try:
        return keycloak_openid.decode_token(
            token,
            keycloak_openid.certs(),
            options={"verify_signature": True, "verify_aud": False}
        )
    except KeycloakError:
        return None

async def get_current_user(token: str = Depends(oauth2_scheme)):
    user = await verify_token(token)
    if not user or config.ACCESS_ROLE not in user.get("realm_access", {}).get("roles", []):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied"
        )
    return user

async def websocket_auth(websocket: WebSocket):
    token = await websocket.receive_text()
    user = await verify_token(token)
    if not user or config.ACCESS_ROLE not in user.get("realm_access", {}).get("roles", []):
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return None
    return user