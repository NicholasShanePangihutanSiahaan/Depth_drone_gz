from glob import glob
from setuptools import find_packages, setup

setup(
    name='mid360_test', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mid360_test']),
        ('share/mid360_test', ['package.xml', 'README.md']),
        ('share/mid360_test/launch', glob('launch/*.py')),
        ('share/mid360_test/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'], zip_safe=False,
    maintainer='Polinasi team', maintainer_email='maintainer@example.com',
    description='Passive MID-360 bench tests', license='Apache-2.0',
    entry_points={'console_scripts': [
        f'{name} = mid360_test.nodes:{name}_main' for name in
        ('cloud_adapter', 'directional_distance', 'imu_integration',
         'health_monitor', 'test_logger', 'raw_recorder')
    ]},
)
