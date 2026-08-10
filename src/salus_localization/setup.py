from setuptools import find_packages, setup

package_name = "salus_localization"
setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="SALUS maintainers", maintainer_email="leonel.sole423@gmail.com",
      description="Localization algorithms for SALUS.", license="MIT", entry_points={"console_scripts": []})
