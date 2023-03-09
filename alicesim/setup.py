from setuptools import setup

package_name = 'alicesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smh',
    maintainer_email='smhhoseinee@gmail.com',
    description='Alice subscriber in python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alicenode = alicesim.alicenode:main'
        ],
    },
)
