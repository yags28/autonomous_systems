from setuptools import setup

package_name = 'task_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/lab3_video.avi']),
        ('share/task_5/launch', ['launch/object_detector_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Image publisher node for lab3 video.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = task_5.image_publisher:main',
            'object_detector = task_5.object_detector:main',
        ],
    },
)
