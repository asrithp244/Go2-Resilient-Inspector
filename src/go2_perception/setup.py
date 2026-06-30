from setuptools import setup

package_name = "go2_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Asrith Pandreka",
    maintainer_email="apandrek@asu.edu",
    description="Visual anomaly detection and CAN telemetry fusion for inspection stations.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_node = go2_perception.perception_node:main",
        ],
    },
)
