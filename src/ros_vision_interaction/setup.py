#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os

package_dirs = {
    'controllers': os.path.join('src', 'ros_vision_interaction', 'controllers'),
    'interaction_engine': os.path.join('src', 'interaction_engine', 'interaction_engine'),
}

d = generate_distutils_setup(
    packages=package_dirs.keys(),
    package_dir=package_dirs,
)
setup(**d)