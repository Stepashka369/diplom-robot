# import uvicorn
# import rclpy
# from threading import Thread
# from fastapi.middleware.cors import CORSMiddleware

# from .broker import DataBroker
# from .ros_handler import ROS2Bridge
# from .web_handler import app
# from .config import config

# def configure_cors():
#     app.add_middleware(
#         CORSMiddleware,
#         allow_origins=["*"],
#         allow_credentials=True,
#         allow_methods=["*"],
#         allow_headers=["*"],
#     )

# def run_ros_node(broker: DataBroker):
#     rclpy.init()
#     ros_bridge = ROS2Bridge(broker)
    
#     try:
#         rclpy.spin(ros_bridge)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ros_bridge.destroy_node()
#         rclpy.shutdown()

# def run_fastapi(broker: DataBroker):
#     configure_cors()
    
#     app.state.broker = broker
#     app.state.ros_bridge = ROS2Bridge(broker)
    
#     uvicorn.run(app, host=config.FASTAPI_HOST, port=config.FASTAPI_PORT)

# def main(args=None):
#     broker = DataBroker()
    
#     ros_thread = Thread(target=run_ros_node, args=(broker,))
#     ros_thread.start()
    
#     run_fastapi(broker)
    
#     ros_thread.join()

# if __name__ == "__main__":
#     main()

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
        allow_origins=api_config.ORIGINS,
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
