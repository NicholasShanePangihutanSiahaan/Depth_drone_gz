from glob import glob
from setuptools import setup, find_packages

setup(name='mid360_slam_test', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/mid360_slam_test']),
        ('share/mid360_slam_test', ['package.xml', 'README.md', 'VALIDATION.md']),
        ('share/mid360_slam_test/config', glob('config/*')),
        ('share/mid360_slam_test/launch', glob('launch/*.py'))],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'], zip_safe=False,
    maintainer='Polinasi team', maintainer_email='maintainer@example.com',
    description='Passive manual experimental SLAM bench tests', license='Apache-2.0',
    entry_points={'console_scripts': [
        f'{name} = mid360_slam_test.nodes:{name}_main' for name in
        ('cloud_preprocessor', 'lidar_imu_estimator', 'test_evaluator', 'test_monitor', 'slam_recorder')
    ]+['test_controller = mid360_slam_test.controller:main',
       'follow_view = mid360_slam_test.follow_view:main']})
