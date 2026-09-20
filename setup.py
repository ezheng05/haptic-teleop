from setuptools import find_packages, setup

package_name = 'depth_estimation_pkg'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/depth_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ellen',
    maintainer_email='ellenz@bu.edu',
    description='Depth estimation using ZoeDepth',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = depth_estimation_pkg.camera_node:main',
            'depth_node = depth_estimation_pkg.node:main',
            'cbf_node = depth_estimation_pkg.cbf_node:main',
            'haptic_teleop = depth_estimation_pkg.haptic_teleop:main',
            'overlay = depth_estimation_pkg.overlay:main',
            'force_viz = depth_estimation_pkg.force_viz:main',
            'validate = depth_estimation_pkg.validate:main'
        ],
    },
)