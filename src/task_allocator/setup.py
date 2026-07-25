from setuptools import setup

package_name = 'task_allocator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'scipy', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Dynamic task allocation (Hungarian algorithm) for multi-robot waste collection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'allocator_node = task_allocator.allocator_node:main',
        ],
    },
)
