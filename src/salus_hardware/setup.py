from glob import glob
from setuptools import find_packages, setup

package_name = "salus_hardware"
setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/launch", glob("launch/*.launch.py"))],
      install_requires=["setuptools"], extras_require={"test": ["pytest"]}, zip_safe=True,
      maintainer="SALUS maintainers", maintainer_email="leonel.sole423@gmail.com",
      description="Hardware adapters for SALUS.", license="MIT", entry_points={"console_scripts": [
          "camera_node = salus_hardware.camera_node:main",
          "capability_profile = salus_hardware.capability_profile_node:main",
          "legacy_drive_measurement_node = salus_hardware.legacy_drive_measurement_node:main",
          "legacy_rtk_observer = salus_hardware.legacy_rtk_observer:main",
          "pixhawk_sensor_adapter = salus_hardware.pixhawk_sensor_adapter:main",
          "pixhawk_rtk_adapter = salus_hardware.pixhawk_rtk_adapter:main",
          "rtcm_dry_run_sink = salus_hardware.rtcm_dry_run_sink:main",
          "vehicle_kinematic_converter = salus_hardware.kinematic_conversion_node:main",
      ]})
