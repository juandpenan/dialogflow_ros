from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os
import glob

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['dialogflow_ros'],
    package_dir={'': 'src'},
    data_files = [
        (os.path.join('share', package_name), glob('resource/*'))

    ]
)
        

setup(**setup_args)