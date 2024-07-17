from setuptools import setup
import os
from glob import glob 

package_name = 'task_generator'
setup(
    name='task_generator',
    version='0.0.0',
    packages=[package_name],
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_generator_node = task_generator.task_generator_node:main',
            'server = task_generator.server:main',
            'filewatcher = task_generator.filewatcher:main'
        ]    
    }
)