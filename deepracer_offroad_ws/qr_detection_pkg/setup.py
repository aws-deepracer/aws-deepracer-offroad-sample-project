from setuptools import setup
import os

package_name = 'qr_detection_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/qr_detection_pkg_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AWS DeepRacer',
    maintainer_email='aws-deepracer@amazon.com',
    description='This package contains logic for qr detection from input camera images \
                 publish the normalized delta of the qr waypoint number from the target position.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detection_node = qr_detection_pkg.qr_detection_node:main'
        ],
    },
)
