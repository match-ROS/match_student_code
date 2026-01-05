# Source - https://stackoverflow.com/a
# Posted by J.V.
# Retrieved 2025-11-27, License - CC BY-SA 4.0

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
packages=['wall_localization_demo'],
package_dir={'': 'src'},
)
setup(**setup_args)
