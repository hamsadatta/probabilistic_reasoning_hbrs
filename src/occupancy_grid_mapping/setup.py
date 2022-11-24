from setuptools import setup

package_name = 'occupancy_grid_mapping'
submodules = 'occupancy_grid_mapping/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pr_hbrs',
    maintainer_email='pr_hbrs@gmail.com',
    description='Mapping environment: occupancy grid mapping using turtle bot in simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gmapping_node = occupancy_grid_mapping.gmapping_node:main',
            'create_from_rosbag = occupancy_grid_mapping.create_from_rosbag:main'
        ],
    },
)
