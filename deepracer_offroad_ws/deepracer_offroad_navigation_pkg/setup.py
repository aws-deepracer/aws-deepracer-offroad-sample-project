from setuptools import setup
import os

package_name = 'deepracer_offroad_navigation_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/deepracer_offroad_navigation_pkg_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AWS DeepRacer',
    maintainer_email='aws-deepracer@amazon.com',
    description='This package contains logic for deepracer_offroad_navigation which decides \
                 the action / controller message to send to servo',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepracer_offroad_navigation_node = deepracer_offroad_navigation_pkg.deepracer_offroad_navigation_node:main'
        ],
    },
)
