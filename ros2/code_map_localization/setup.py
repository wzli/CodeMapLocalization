import os
from glob import glob
from setuptools import setup

package_name = 'code_map_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'codemap'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='wzli',
    maintainer_email='me@wenzheng.li',
    description='ROS2 wrapper camera based code map localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial = code_map_localization.serial:main',
            'webcam = code_map_localization.webcam:main'
        ],
    },
)
