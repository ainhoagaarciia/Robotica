from glob import glob

from setuptools import find_packages, setup

package_name = 'rg2_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacio',
    maintainer_email='ignacio@todo.todo',
    description='Driver ROS 2 de la pinza OnRobot RG2 montada en un robot UR',
    license='TODO',
    entry_points={
        'console_scripts': [
            'rg2_control_node = rg2_driver.driver_action_server:main',
        ],
    },
)
