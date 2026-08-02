from setuptools import find_packages, setup

package_name = "parol6_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gupyfish",
    maintainer_email="gupyfish@gmail.com",
    description=(
        "Local TCP bridge between the parol6_hardware_interface ros2_control "
        "plugin and a parol6-server instance."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "parol6_bridge = parol6_bridge.bridge_node:main",
        ],
    },
)
