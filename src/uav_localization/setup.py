from setuptools import setup, find_packages

package_name = "uav_localization"

setup(
    name=package_name,
    version="0.0.1",

    packages=find_packages(exclude=["test"]),

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

    description="Localization package for plantation UAV",
    license="MIT",

    entry_points={
        "console_scripts": [
            "localization_bridge=uav_localization.localization_bridge:main",
            "tree_mapper=uav_localization.tree_mapper:main",
            "row_detector=uav_localization.row_detector:main",
            "occupancy_mapper=uav_localization.occupancy_mapper:main",
            "tf_broadcaster=uav_localization.tf_broadcaster:main",
        ],
    },
)