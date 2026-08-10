from glob import glob
import os

from setuptools import find_packages, setup

package_name = "beehive_drone"

setup(
    name=package_name,
    version="2.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="beehive_drone_team",
    maintainer_email="maintainer@example.com",
    description="Gazebo/SITL validation package for the revised one-tree mission.",
    license="Apache-2.0",
    entry_points={
        'console_scripts': [
            # DAFTARKAN SEMUA NODE DI SINI (nama_eksekusi = nama_folder.nama_file:main)
            'mission_state_machine = beehive_drone.mission_state_machine:main',
            'dynamic_orbit_controller = beehive_drone.dynamic_orbit_controller:main',
            'velocity_controller = beehive_drone.velocity_controller:main',
            'vortex_avoidance_controller = beehive_drone.vortex_avoidance_controller:main',
            'tree_mapper = beehive_drone.tree_mapper:main',
            'flight_manager = beehive_drone.flight_manager:main',
            'mission_analyzer = beehive_drone.mission_analyzer:main',
        ],
    },
)
