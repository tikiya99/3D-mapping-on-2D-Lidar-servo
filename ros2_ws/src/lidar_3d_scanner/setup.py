from setuptools import find_packages, setup

package_name = 'lidar_3d_scanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scanner.launch.py']),
        ('share/' + package_name + '/config', ['config/scanner.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thasinduwickrama',
    maintainer_email='thasinduwickrama12@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanner_node = lidar_3d_scanner.scanner_node:main'
        ],
    },
)
