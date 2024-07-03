from setuptools import setup

package_name = 'testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'scripts.action_publisher',
        'scripts.drl_agent_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ahmed',
    author_email='ahmed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_publisher = scripts.action_publisher:main',
            'drl_agent_node = scripts.drl_agent_node:main',
        ],
    },
)
