#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os


d = generate_distutils_setup()
setup(**d)