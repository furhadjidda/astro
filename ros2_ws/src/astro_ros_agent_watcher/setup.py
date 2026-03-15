from setuptools import find_packages
from setuptools import setup

package_name = "ros_agent_watcher"

setup(
    name=package_name,
    version="2.1.5",
    packages=find_packages(exclude=[]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    description=(
        "Micro ROS agent watcher for monitoring the status of micro ROS "
        "agents in a ROS 2 environment. This package provides tools to track "
        "and manage micro ROS agents, ensuring seamless integration and "
        "communication within the ROS 2 ecosystem."
    ),
    license="Apache License, Version 2.0",
    entry_points={
        "console_scripts": [
            "ros_agent_watcher = ros_agent_watcher.script.ros_agent_watcher:main"
        ],
    },
)
