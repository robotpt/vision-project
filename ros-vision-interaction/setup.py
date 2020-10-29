#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os

package_dirs = {
    'interaction_engine': os.path.join('interaction-engine', 'interaction_engine'),
}

d = generate_distutils_setup(
    packages=package_dirs.keys(),
    package_dir=package_dirs,
)
setup(**d)