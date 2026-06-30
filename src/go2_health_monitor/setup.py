from setuptools import setup

package_name = "go2_health_monitor"

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
    description="Independent topic health watchdog for the Go2 inspection system.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "topic_health_monitor = go2_health_monitor.topic_health_monitor:main",
        ],
    },
)
