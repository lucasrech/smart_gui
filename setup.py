"""Catkin packaging metadata for the `smart_gui` ROS Noetic package."""

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["smart_gui"],
    package_dir={"": "."},
)

setup(**setup_args)
