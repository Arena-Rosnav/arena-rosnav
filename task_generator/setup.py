from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'task_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        where='.',
        include=[f'{package_name}*']
    ),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Name',
    maintainer_email='your@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'task_generator_node = task_generator.task_generator_node:main',
            'generate_map = task_generator.utils.map_generator:main',
            # 'server = task_generator.server:main',
            # 'filewatcher = task_generator.filewatcher:main'
        ]
    }
)
