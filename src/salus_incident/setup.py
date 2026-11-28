from setuptools import setup

setup(
    name="salus_incident", version="0.1.0", packages=["salus_incident"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/salus_incident"]),
        ("share/salus_incident", ["package.xml"]),
        ("share/salus_incident/config", ["config/incident.rviz"]),
    ],
    maintainer="SALUS maintainers", maintainer_email="leonel.sole423@gmail.com",
    description="Bounded navigation incident recording and offline replay.",
    license="MIT", extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": [
        "incident_recorder = salus_incident.recorder:main",
        "incident_replay = salus_incident.replay:main",
    ]},
)
