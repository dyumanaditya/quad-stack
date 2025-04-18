import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'quadstack_keyboard_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='dyuman2001@gmail.com',
    description='Teleop package for controlling robot with keyboard',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = quadstack_keyboard_teleop.teleop_node:main',
        ],
    },
)

