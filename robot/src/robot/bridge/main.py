import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from bridge.api import routers
from bridge.configs.api_config import APIConfig

def configure_fastapi(app: FastAPI):
    api_config = APIConfig()

    for router in routers:
        app.include_router(router)

    app.add_middleware(
        CORSMiddleware,
        # allow_origins=api_config.ORIGINS,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )


def main(args=None):
    app = FastAPI(title="Robot API")
    configure_fastapi(app)
    uvicorn.run(app, host='localhost', port=8383)


if __name__ == "__main__":
    main()
