from glob import glob

from setuptools import find_packages, setup

package_name = 'launcher_robots_lab_robotica'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/calibration', glob('calibration/*.yaml')),
        ('share/' + package_name + '/meshes/visual', glob('meshes/visual/*')),
        ('share/' + package_name + '/meshes/collision', glob('meshes/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacio',
    maintainer_email='ignacio@todo.todo',
    description='Lanzadores y configuración de los robots UR3e + RG2 del laboratorio',
    license='TODO',
    entry_points={
        'console_scripts': [],
    },
)
