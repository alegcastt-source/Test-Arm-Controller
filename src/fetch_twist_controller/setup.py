from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['fetch_twist_controller'], package_dir={'': 'src'}))
