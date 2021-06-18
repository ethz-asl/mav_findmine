## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['fm_mission_planner'],
    package_dir={'':'python'},
    scripts=['nodes/target_viz']
    )

setup(**setup_args)
