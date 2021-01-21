#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os

package_dirs = {
    'controllers': os.path.join('src', 'vision-interaction', 'vision_interaction', 'controllers'),
    'engine_statedb': os.path.join('src', 'vision-interaction', 'vision_interaction', 'engine_statedb'),
    'interaction_builder': os.path.join('src', 'vision-interaction', 'vision_interaction', 'interaction_builder'),
    'interfaces': os.path.join('src', 'vision-interaction', 'vision_interaction', 'interfaces'),
    'vision_project_tools': os.path.join('src', 'vision-interaction', 'vision_interaction', 'vision_project_tools'),
    'interaction_engine': os.path.join('src', 'interaction_engine', 'interaction_engine'),
}

d = generate_distutils_setup(
    packages=package_dirs.keys(),
    package_dir=package_dirs,
)
setup(**d)
