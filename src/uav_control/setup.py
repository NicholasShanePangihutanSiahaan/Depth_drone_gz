from setuptools import setup, find_packages

package_name = "uav_control"

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

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="shane",
    maintainer_email="nicholasshane08@gmail.com",

    description="UAV Flight Controller",
    license="MIT",

    entry_points={
        "console_scripts":[
            "path_follower=uav_control.path_follower:main",
            "velocity_controller=uav_control.velocity_controller:main",
            "yaw_controller=uav_control.yaw_controller:main",
            "obstacle_avoidance=uav_control.obstacle_avoidance:main",
            "mission_executor=uav_control.mission_executor:main",
            "fake_uav = uav_control.fake_uav:main", 
            "landing_controller=uav_control.landing_controller:main",
        ],
    },
)