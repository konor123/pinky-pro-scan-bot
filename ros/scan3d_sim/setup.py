from glob import glob

from setuptools import find_packages, setup

package_name = 'scan3d_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/layouts', glob('layouts/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinky',
    maintainer_email='pinky@example.com',
    description='Gazebo scan test for the Pinky Pro 3D scan bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_agent=scan3d_sim.sim_agent:main',
            'wheel_moves=scan3d_sim.wheel_moves:main',
        ],
    },
)
