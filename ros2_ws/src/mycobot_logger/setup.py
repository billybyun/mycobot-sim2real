from setuptools import setup

package_name = "mycobot_logger"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Billy Byun",
    maintainer_email="villy.byun@gmail.com",
    description="ROS2 logger stub for myCobot sim2real.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "logger_node = mycobot_logger.logger_node:main",
        ],
    },
)

