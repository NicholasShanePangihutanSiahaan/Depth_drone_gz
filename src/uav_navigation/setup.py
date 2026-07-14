from setuptools import setup

package_name = 'uav_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shane',
    maintainer_email='nicholasshane08@gmail.com',
    description='Navigation package for autonomous plantation UAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'global_planner=uav_navigation.global_planner:main',

            'local_planner=uav_navigation.local_planner:main',

            'waypoint_manager=uav_navigation.waypoint_manager:main',

            'mission_controller=uav_navigation.mission_controller:main',

            'mission_analyzer=uav_navigation.mission_analyzer:main',

            'tree_inspection_manager = uav_navigation.tree_inspection_manager:main',

            'trajectory_generator=uav_navigation.trajectory_generator:main',

        ],
    },
)