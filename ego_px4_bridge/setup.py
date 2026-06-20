from setuptools import find_packages, setup

package_name = 'ego_px4_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JIMT111',
    maintainer_email='30071363+JIMT111@users.noreply.github.com',
    description='ROS 2 bridges connecting EGO Planner with PX4 SITL and Gazebo depth data.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_to_ego_odom = ego_px4_bridge.px4_to_ego_odom:main',
            'ego_to_px4_setpoint = ego_px4_bridge.ego_to_px4_setpoint:main',
            'px4_offboard_arm = ego_px4_bridge.px4_offboard_arm:main',
            'gazebo_cloud_to_ego_cloud = ego_px4_bridge.gazebo_cloud_to_ego_cloud:main',
        ],
    },
)
