from glob import glob
from setuptools import find_packages, setup

package_name = "salus_navigation"
setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
                  ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.xml") + glob("config/*.geojson") + glob("config/*.pgm"))],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="SALUS maintainers", maintainer_email="leonel.sole423@gmail.com",
      description="Navigation and missions for SALUS.", license="MIT", extras_require={"test": ["pytest"]}, entry_points={"console_scripts": [
          "nav_command_server = salus_navigation.nav_command_server:main",
          "nav_observer = salus_navigation.nav_observer:main",
          "nav2_startup_coordinator = salus_navigation.nav2_startup_coordinator:main",
          "navigation_profile_coordinator = salus_navigation.navigation_profile_coordinator:main",
          "nav_snapshot_server = salus_navigation.nav_snapshot_server:main",
          "path_health = salus_navigation.path_health:main",
          "patrol_mission_coordinator = salus_navigation.patrol_mission_coordinator:main",
          "route_executor = salus_navigation.route_executor_node:main",
          "zones_manager = salus_navigation.zones_manager:main",
      ]})
