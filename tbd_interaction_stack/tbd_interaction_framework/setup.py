#!/usr/bin/env python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tbd_interaction_framework'],
    package_dir={'': 'python_src'}
)

setup(**d)
