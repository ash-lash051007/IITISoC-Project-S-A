from setuptools import find_packages, setup

package_name = 'waste_database'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/waste_database.launch.py',
            'launch/robot_state_bridges.launch.py',
            'launch/task_managers.launch.py',
            'launch/aruco_database_bridges.launch.py',
        ]),
        ('share/' + package_name + '/dashboard', ['dashboard/waste_dashboard.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ash',
    maintainer_email='me250003002@iiti.ac.in',
    description='Centralized shared waste database node for multi-robot task allocation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waste_database_node = waste_database.waste_database_node:main',
            'robot_state_bridge_node = waste_database.robot_state_bridge_node:main',
            'task_manager_node = waste_database.task_manager_node:main',
            'aruco_database_bridge_node = waste_database.aruco_database_bridge_node:main',
        ],
    },
)
