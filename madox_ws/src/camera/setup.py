from setuptools import setup

package_name = "camera"

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
    description="Manages the camera including overlay information and streaming to web browser via gstreamer.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ros2_camera = camera.server:main"]
    },
)
