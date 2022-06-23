#!/usr/bin/env python
 
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
 
d = generate_distutils_setup(
    # #  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    # #  packages should list all folders containing an __init__.py
    # #  that you would like to include as imports in other rospy projects
    # packages=['hello_world'],
    package_dir={'': 'src'}
)
 
setup(**d)