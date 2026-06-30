from setuptools import setup

package_name = "go2_sim"

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
    description="Simulated CAN/Modbus telemetry emitter for inspection stations.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sim_machine_emulator = go2_sim.sim_machine_emulator:main",
        ],
    },
)
