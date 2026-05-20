from setuptools import find_packages, setup

package_name = 'imr_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='robot@example.com',
    description='Intelligent Mobile Robot — ROS2 hardware-independent package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Step 1
            'mock_encoder_node = imr_ros2.mock_encoder_node:main',
            # Step 2
            'mock_motor_controller_node = imr_ros2.mock_motor_controller_node:main',
            # Step 3
            'encoder_odometry_node = imr_ros2.encoder_odometry_node:main',
            # Step 4 / 5
            'waypoint_follower_node = imr_ros2.waypoint_follower_node:main',
            # Step 6
            'mock_ultrasonic_node = imr_ros2.mock_ultrasonic_node:main',
            # Steps 7-8 will be added here incrementally
        ],
    },
)
