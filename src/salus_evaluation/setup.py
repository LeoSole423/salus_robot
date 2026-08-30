from glob import glob
from setuptools import find_packages, setup

package_name = "salus_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config/scenarios", glob("config/scenarios/*.yaml")),
        ("share/" + package_name + "/config/matrices", glob("config/matrices/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="SALUS maintainers",
    maintainer_email="leonel.sole423@gmail.com",
    description="Reproducible navigation evaluation domain for SALUS.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": [
        "navigation_evaluation = salus_evaluation.evaluation_runner:main",
        "navigation_matrix_summary = salus_evaluation.matrix_runner:main",
        "navigation_matrix_execute = salus_evaluation.matrix_executor:main",
    ]},
)
