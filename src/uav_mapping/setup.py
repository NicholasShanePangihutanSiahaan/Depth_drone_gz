from setuptools import setup

package_name = "uav_mapping"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shane",
    maintainer_email="nicholasshane08@gmail.com",
    description="Mapping package",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tree_mapper=uav_mapping.tree_mapper:main",
            "occupancy_mapper=uav_mapping.occupancy_mapper:main",
            "rviz_marker=uav_mapping.rviz_marker:main",
            "map_saver=uav_mapping.map_saver:main",
        ],
    },
)