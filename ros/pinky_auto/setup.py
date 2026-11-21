from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_auto'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinky',
    maintainer_email='pinky@example.com',
    description='Pinky Pro autonomous driving',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver=pinky_auto.motor_driver:main',
            'patrol=pinky_auto.patrol:main',
            'follow_ultra=pinky_auto.follow_ultra:main',
        ],
    },
)
