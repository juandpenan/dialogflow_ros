#!/usr/bin/env python

from setuptools import find_packages
from setuptools import setup

package_name = 'dialogflow_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jonatan Gines',
    author_email='jonatan.gines@urjc.es',
    maintainer='Jonatan Gines',
    maintainer_email='jonatan.gines@urjc.es',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'The dialogflow_ros package'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dialogflow_client = dialogflow_ros.dialogflow_client:main'
        ],
    },
)
