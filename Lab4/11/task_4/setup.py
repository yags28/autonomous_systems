from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'task_4'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # folder exists but can be empty; ament_python expects it
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # install an empty maps dir so it exists after install
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Week 5 Task 4: generate mapping data (SLAM + RViz) under a namespace',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_navigator = task_4.auto_navigator:main',
        ],
    },
)