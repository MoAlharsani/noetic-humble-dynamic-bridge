from setuptools import find_packages, setup

package_name = 'ros_connection_bridge'

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
    maintainer='alharpc',
    maintainer_email='alharpc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_ros_bridge = ros_connection_bridge.mqtt_ros_bridge:main',
            'ros_mqtt_bridge = ros_connection_bridge.ros_mqtt_bridge:main',
        ],
    },
)
