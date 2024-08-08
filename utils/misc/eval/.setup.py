from setuptools import setup
import os
from glob import glob

package_name = 'eval'

setup(
    name=package_name,
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
    maintainer='reyk',
    maintainer_email='reyk@todo.todo',
    description='The eval package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'script_name = eval.script:main',
        ],
    },
)
