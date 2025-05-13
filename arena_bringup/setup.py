import os
from glob import glob

from setuptools import setup

package_name = 'arena_bringup'


def recursive_walk(base_dir):
    return [
        (
            os.path.join('share', package_name, base),
            [os.path.join(base, file)]
        )
        for base, _, files in os.walk(base_dir)
        for file in files
    ]


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *recursive_walk('launch'),
        *recursive_walk('configs'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='voshch',
    maintainer_email='dev@voshch.dev',
    description='Arena bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # 'console_scripts': [
        #     'gazebo_delete_entity = arena_bringup.launch.testing.simulators.gazebo_delete_entity:main',
        # ],
        'launch_ros.node_action': [
            'NodeLogLevelExtension = arena_bringup.extensions.NodeLogLevelExtension:NodeLogLevelExtension',
        ],
    },
)
