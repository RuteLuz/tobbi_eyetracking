#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['tobii_eyetracking', 'tobii_eyetracking_ros'],
 package_dir={'tobii_eyetracking': 'common/src/tobii_eyetracking', 'tobii_eyetracking_ros': 'ros/src/tobii_eyetracking_ros'}
)

setup(**d)
