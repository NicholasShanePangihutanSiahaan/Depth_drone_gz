from setuptools import find_packages, setup

package_name = 'uav_perception'

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
    maintainer='shane',
    maintainer_email='nicholasshane08@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tree_detector = uav_perception.tree_detector:main',
            'depth_processor = uav_perception.depth_processor:main',
            'tree_localizer = uav_perception.tree_localizer:main',
            'obstacle_detector = uav_perception.obstacle_detector:main'
        ],
    },
)
