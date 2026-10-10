from setuptools import find_packages, setup

package_name = 'obj_det_bridge'

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
    maintainer='palmbee1',
    maintainer_email='palmbee1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = obj_det_bridge.bridge_node:main',
            'object_listener = obj_det_bridge.object_listener:main',
            'visual_subscriber = obj_det_bridge.visual_subscriber:main',
            'detection_publisher = obj_det_bridge.detection_publisher:main'
        ],
    },
)
