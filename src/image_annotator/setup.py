import os
from setuptools import find_packages, setup

package_name = 'image_annotator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', 'image_annotator.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu2204',
    maintainer_email='limourendeyouxiang@qq.com',
    description='Annotate detections',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_annotator = image_annotator.image_annotator:main',
        ],
    },
)