from setuptools import setup
import os
from glob import glob

package_name = 'arena_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch/testing/simulators'), glob('launch/testing/simulators/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim Seeger',
    maintainer_email='tim.seeger@campus.tu-berlin.de',
    description='Arena bringup package with delete entity script',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_delete_entity = arena_bringup.launch.testing.simulators.gazebo_delete_entity:main',
        ],
    },
)
