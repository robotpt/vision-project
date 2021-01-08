#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os

package_dirs = {
    'controllers': os.path.join('src', 'vision-interaction', 'vision_interaction', 'controllers'),
    'data_structures': os.path.join('src', 'vision-interaction', 'vision_interaction', 'data_structures'),
    'interactions': os.path.join('src', 'vision-interaction', 'vision_interaction', 'interactions'),
    'interfaces': os.path.join('src', 'vision-interaction', 'vision_interaction', 'interfaces'),
    'interaction_engine': os.path.join('src', 'interaction_engine', 'interaction_engine'),
}

d = generate_distutils_setup(
    packages=package_dirs.keys(),
    package_dir=package_dirs,
)
setup(**d)