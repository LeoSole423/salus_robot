from glob import glob
from setuptools import find_packages, setup

package_name = "salus_perception"
setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
                  (
                      "share/" + package_name + "/config",
                      glob("config/*.yaml") + glob("config/*.rviz"),
                  )],
      install_requires=["setuptools"], extras_require={"test": ["pytest"]}, zip_safe=True,
      maintainer="SALUS maintainers", maintainer_email="leonel.sole423@gmail.com",
      description="Perception algorithms for SALUS.", license="MIT", entry_points={"console_scripts": [
          "cloud_normalizer = salus_perception.cloud_normalizer:main",
          "scan_ground_filter = salus_perception.scan_ground_filter:main",
          "scan_noise_filter = salus_perception.scan_noise_filter:main",
          "scan_preview = salus_perception.scan_preview:main",
      ]})
