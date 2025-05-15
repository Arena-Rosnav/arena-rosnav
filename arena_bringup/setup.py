import os
from glob import glob

from setuptools import setup

package_name = 'arena_bringup'


def recursive_walk(base_dir, *, destination=None, relative_to=None):
    if destination is None:
        destination = os.path.join('share', package_name)
    if relative_to is None:
        relative_to = ''

    def process(base, files):
        adjusted_base = os.path.relpath(base, relative_to)
        return (
            os.path.normpath(os.path.join(destination, adjusted_base)),
            [
                os.path.join(base, file)
                for file in files
            ]
        )

    return [
        process(base, files)
        for base, _, files in os.walk(base_dir)
    ]


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *recursive_walk('scripts', destination=os.path.join('lib', package_name), relative_to='scripts'),
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
