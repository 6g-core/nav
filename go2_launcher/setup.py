from glob import glob
import os

from setuptools import setup


package_name = "go2_launcher"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="haorunlai@huawei.com",
    description="Launch files for the Go2 runtime nodes",
    license="MIT",
)
