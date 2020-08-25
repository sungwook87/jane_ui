#!/usr/bin/env python
""" ROS client setup."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

__author__ = "JANE"

d = generate_distutils_setup(packages=["storm32"], package_dir={"": "jane_gimbal/src"})

setup(**d)
