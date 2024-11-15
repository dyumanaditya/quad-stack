import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mab_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='dyuman2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_saver = mab_localization.map_saver:main',
            'imu_covariance_node = mab_localization.imu_covariance:main',
            'odom_2d = mab_localization.odom_2d:main',
        ],
    },
)
