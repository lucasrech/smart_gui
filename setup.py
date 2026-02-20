"""Packaging metadata for the `smart_gui` ROS 2 Python package."""

from setuptools import find_packages, setup

package_name = "smart_gui"

setup(
    # Package identity.
    name=package_name,
    version="0.1.0",
    # Install authored Python modules (excluding tests).
    packages=find_packages(exclude=["test"]),
    # Install ROS package metadata and launch files into share directory.
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/smart_gui_api.launch.py"]),
    ],
    # Runtime Python dependencies for backend API features.
    install_requires=[
        "setuptools",
        "fastapi",
        "uvicorn[standard]",
        "wsproto",
        "numpy",
        "pillow",
    ],
    zip_safe=True,
    maintainer="rech",
    maintainer_email="rech@todo.todo",
    description="ROS 2 package for Smart GUI backend API.",
    license="Apache-2.0",
    tests_require=["pytest"],
    # CLI entrypoints exposed via `ros2 run smart_gui <executable>`.
    entry_points={
        "console_scripts": [
            "smart_gui_api = smart_gui.ros2_inspector_api:main",
            "random_int8_topics = smart_gui.random_int8_topics_node:main",
        ],
    },
)
