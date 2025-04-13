from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadstack_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
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
            'publish_robot_state = quadstack_utils.publish_robot_state:main',
            'velocity_relay = quadstack_utils.velocity_relay:main',
            'point_cloud_to_occupancy = quadstack_utils.point_cloud_to_occupancy:main',
            'camera_frame_stabilizer = quadstack_utils.camera_frame_stabilizer:main',
            'image_rotate = quadstack_utils.image_rotate:main',
            'project_laser_frame = quadstack_utils.project_laser_frame:main',
            'fake_imu = quadstack_utils.fake_imu:main',
            'odom_gt = quadstack_utils.odom_gt:main',
            'odom_plotter = quadstack_utils.odom_plotter:main',
            'publish_velocity_path = quadstack_utils.publish_velocity_path:main',
            'publish_tf_on_topic = quadstack_utils.publish_tf_on_topic:main',
            'compute_leg_odom_covariance = quadstack_utils.compute_leg_odom_covariance:main',
            'plot_header_time = quadstack_utils.plot_header_time:main',
        ],
    },
)
