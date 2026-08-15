
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'simple_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caaren',
    maintainer_email='153974602+camirian@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulation_controller = simple_manipulation.manipulation_controller:main',
            'simple_trajectory_server = simple_manipulation.simple_trajectory_server:main',
            'perception_node = simple_manipulation.perception_node:main',
            'perform_pick = simple_manipulation.perform_pick:main',
        ],
    },
)
