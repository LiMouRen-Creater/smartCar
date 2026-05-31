import os
from setuptools import find_packages, setup

package_name = 'yellow_track_opencv'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'resource'), ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', 'yellow_track_opencv.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='基于OpenCV的黄色车道线检测节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yellow_track_opencv = yellow_track_opencv.yellow_track_opencv:main',
        ],
    },
)