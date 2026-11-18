from setuptools import setup

package_name = 'drill_arm_ros_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'drill_arm_bridge = drill_arm_ros_bridge.bridge_node:main',
            'drill_arm_cli    = drill_arm_ros_bridge.cli_node:main',
        ],
    },
)
