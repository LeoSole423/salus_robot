from glob import glob

from setuptools import find_packages, setup

package_name = "salus_localization"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SALUS maintainers",
    maintainer_email="leonel.sole423@gmail.com",
    description="Local Ackermann odometry and simulated IMU normalization for SALUS.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "ackermann_odometry = salus_localization.ackermann_odometry:main",
            "sim_imu_from_odom = salus_localization.sim_imu_from_odom:main",
            "imu_normalizer = salus_localization.imu_normalizer:main",
            "sim_gps_normalizer = salus_localization.sim_gps_normalizer:main",
            "gps_course_heading = salus_localization.gps_course_heading:main",
            "map_gps_absolute_measurement = salus_localization.map_gps_absolute_measurement:main",
        ],
    },
)
