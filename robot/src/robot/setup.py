import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'nodes'), glob('nodes/*')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'websockets>=10.0',
        'fastapi>=0.68.0',
        'uvicorn>=0.15.0',
        'python-keycloak>=2.10.0',
        'python-dotenv>=0.19.0',
        'pydantic>=2.0.0',
        'pydantic-settings>=2.0.0',
        'typing-extensions>=4.0.0',
        'annotated-types>=0.4.0'
    ],
    zip_safe=True,
    maintainer='stepa',
    maintainer_email='kardash-99@list.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '_movement = nodes.base_movement_node:main',
            '_rotate = nodes.head_movement_node:main',
            '_bridge = bridge.main:main',
            '_agregator = nodes.ir_sensor_agregator:main'
        ],
    },
)
