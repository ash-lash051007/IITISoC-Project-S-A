from setuptools import setup
import os
from glob import glob

package_name = 'iitisoc'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

    # Launch files
    data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))

    # Config files (all types)
    data_files.append((os.path.join('share', package_name, 'config'), glob('config/*')))

    # World files (only .world files)
    data_files.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')))

    # Map files
    data_files.append((os.path.join('share', package_name, 'maps'), glob('maps/*')))
    
    # URDF files
    data_files.append((os.path.join('share', package_name, 'urdf'), glob('urdf/*')))

    # Model files (recursive)
    for dirpath, dirnames, filenames in os.walk('models'):
        if filenames:
            file_paths = [os.path.join(dirpath, f) for f in filenames]
            install_dir = os.path.join('share', package_name, dirpath)
            data_files.append((install_dir, file_paths))

    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'aruco_detector = iitisoc.aruco_detector:main',
        ],
    },
)
