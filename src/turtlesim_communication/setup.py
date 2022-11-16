from setuptools import setup

package_name = 'turtlesim_communication'

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
    maintainer='melody',
    maintainer_email='kishansawant96@gmail.com',
    description='This package is used to communicate with turtlesim. It will receive position of the turtle and moves it as per input',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = turtlesim_communication.control_node:main'
        ],
    },
)
