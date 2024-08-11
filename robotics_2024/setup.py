from setuptools import find_packages, setup

package_name = 'turtlebot3_control_ros2'

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
    maintainer='ricardo',
    maintainer_email='ricardogrando13@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_ctrl = turtlebot3_control_ros2.turtlebot_ctrl:main',
            'turtle_ctrl = turtlebot3_control_ros2.turtle_ctrl:main'
        ],
    },
)
