from setuptools import setup

package_name = "jetbot_sys_info"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@todo.todo",
    description="Extracts and publishes several information from the jetbot hardware. Publishes teh following topics: info_sys_disk, info_sys_mem, info_sys_CPU, info_sys_power.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros2_jetbot_sys_info = jetbot_sys_info.jetbot_sys_info:main"
        ]
    },
)
