from setuptools import setup

package_name = 'uav_localization'

setup(
    name=package_name,
    version='0.0.1',

    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='shane',
    maintainer_email='nicholasshane08@gmail.com',

    description='Localization package for plantation UAV',
    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'ekf_localization=uav_localization.ekf_localization:main',
            'tree_mapper=uav_localization.tree_mapper:main',
            'row_detector=uav_localization.row_detector:main',
            'occupancy_mapper=uav_localization.occupancy_mapper:main',
            'tf_broadcaster=uav_localization.tf_broadcaster:main',
        ],
    },
)