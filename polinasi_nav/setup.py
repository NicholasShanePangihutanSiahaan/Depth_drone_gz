from glob import glob
from setuptools import setup

setup(name='polinasi_nav', version='0.1.0', packages=['polinasi_nav'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/polinasi_nav']),
                  ('share/polinasi_nav', ['package.xml']),
                  ('share/polinasi_nav/config', glob('config/*')),
                  ('share/polinasi_nav/launch', glob('launch/*.launch.py')),
                  ('share/polinasi_nav/worlds', glob('worlds/*.sdf')),
                  ('share/polinasi_nav/models/iris_lidar', glob('models/iris_lidar/*')),
                  ('share/polinasi_nav/models/palm_farm', glob('models/palm_farm/*.*')),
                  ('share/polinasi_nav/models/palm_farm/meshes', glob('models/palm_farm/meshes/*.dae'))],
      install_requires=['setuptools', 'numpy', 'scipy'], tests_require=['pytest'], zip_safe=False,
      maintainer='Polinasi team', maintainer_email='nicholasshane08@gmail.com',
      description='Simulation-only map-based inspection navigation', license='Apache-2.0',
      entry_points={'console_scripts': [
          'navigation = polinasi_nav.ros_nodes:navigation_main',
          'mapping_navigation = polinasi_nav.ros_nodes:mapping_main',
          'mapping_io = polinasi_nav.mapping_io:main',
          'identification_navigation = polinasi_nav.identification_node:main',
          'identification_recorder = polinasi_nav.identification_recorder:main',
          'mavros_configurator = polinasi_nav.mavros_configurator:main',
          'localisation = polinasi_nav.ros_nodes:localisation_main',
          'externalnav = polinasi_nav.ros_nodes:externalnav_main',
          'range_bridge = polinasi_nav.ros_nodes:range_main',
          'sensor_gate = polinasi_nav.ros_nodes:gate_main',
          'benchmark = polinasi_nav.benchmark:main']})
