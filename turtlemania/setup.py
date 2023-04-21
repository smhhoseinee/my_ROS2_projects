import os
from glob import glob
from setuptools import setup

package_name = 'turtlemania'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smh',
    maintainer_email='smhhoseinee@gmail.com',
    description='3 turtles chasing each other',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlemania_tf2_broadcaster = turtlemania.turtlemania_tf2_broadcaster:main',
            'turtlemania_tf2_listener = turtlemania.turtlemania_tf2_listener:main',
        ],
    },
)
