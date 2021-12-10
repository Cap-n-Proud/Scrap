from setuptools import setup

package_name = "jetbot_oled_display"

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
    maintainer="Cap-n-Proud",
    maintainer_email="robot@todo.todo",
    description="Manages the jetbot oled display as a ROS2 node",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ros2_jetbot_oled_display = jetbot_oled_display.oled:main"]
    },
)
