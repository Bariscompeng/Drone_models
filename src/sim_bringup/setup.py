from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'sim_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        # ament index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # launch & config dosyaları
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='baris',
    maintainer_email='baris@example.com',
    description='Gazebo (Ignition Fortress) world + bridge bringup',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'tf_parent_fix = sim_bringup.tf_parent_fix:main',
        	'imu_frame_fix = sim_bringup.imu_frame_fix:main',
            
        ],
    },
)
