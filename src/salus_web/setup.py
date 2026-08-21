from glob import glob
from setuptools import find_packages, setup

package_name = "salus_web"
setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/launch", glob("launch/*.launch.py"))],
      install_requires=["setuptools", "PyYAML", "websockets"], zip_safe=True,
      maintainer="SALUS maintainers", maintainer_email="leonel.sole423@gmail.com",
      description="Operator WebSocket bridge for SALUS.", license="MIT",
      extras_require={"test": ["pytest"]}, entry_points={"console_scripts": [
          "web_bridge = salus_web.web_bridge_node:main",
      ]})
