from setuptools import find_packages, setup

package_name = 'ofb_ctrl'

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
    maintainer='acdl8',
    maintainer_email='acdl8@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_offboard_node = ofb_ctrl.waypoint_offboard_node:main',
            'offboard_control_node = ofb_ctrl.offboard_control:main',
        ],
    },
)
