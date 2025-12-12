from setuptools import setup

package_name = 'task_6'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/red_ball_tracker_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ydawanka',
    maintainer_email='ydawanka@purdue.edu',
    description='Red Ball Tracker node for following a red ball using camera data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_ball_tracker = task_6.red_ball_tracker:main',
        ],
    },
)

