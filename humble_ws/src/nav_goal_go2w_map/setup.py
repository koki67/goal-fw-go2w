from glob import glob

from setuptools import find_packages, setup

package_name = "nav_goal_go2w_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Koki Tanaka",
    maintainer_email="67k.tanaka@gmail.com",
    description="Pre-built map preparation and serving for Go2W goal navigation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "prepare_map = nav_goal_go2w_map.prepare_map_cli:main",
            "finish_map = nav_goal_go2w_map.finish_map_cli:main",
            "map_cloud_publisher = nav_goal_go2w_map.map_cloud_publisher_node:main",
        ],
    },
)
