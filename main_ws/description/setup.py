import glob
from setuptools import find_packages, setup

package_name = 'description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.xml')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*.rviz')),
        ('share/' + package_name + '/docky/urdf', glob.glob('docky/urdf/*.urdf')),
        ('share/' + package_name + '/docky/urdf', glob.glob('docky/urdf/*.xacro')),
        ('share/' + package_name + '/docky/meshes/stl', glob.glob('docky/meshes/stl/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
