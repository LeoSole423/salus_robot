from glob import glob

from setuptools import find_packages, setup

package_name = "salus_control"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial>=3.5"],
    zip_safe=True,
    maintainer="SALUS maintainers",
    maintainer_email="leonel.sole423@gmail.com",
    description="Vehicle control, simulated actuation and battery logic for SALUS.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "controller_server_node = salus_control.controller_server_node:main",
            "legacy_vehicle_command_node = salus_control.legacy_vehicle_command_node:main",
            "vehicle_command_comparison_node = salus_control.vehicle_command_comparison_node:main",
            "canonical_command_dry_run_node = salus_control.canonical_command_dry_run_node:main",
        ],
    },
)
