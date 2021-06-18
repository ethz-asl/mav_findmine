## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['fm_pix4d'],
    package_dir={'': 'python'},
    scripts=['scripts/export_dsm',
             'scripts/fm_export_pix4d',
             'scripts/fm_tile_tf',
             'scripts/fm_gcp_correct'],
)

setup(**setup_args)
