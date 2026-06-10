from glob import glob

from setuptools import find_packages, setup

package_name = "nav_goal_go2w_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Koki Tanaka",
    maintainer_email="67k.tanaka@gmail.com",
    description="Nav2 stack + operator goal executor for the Go2W.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goal_pose_executor = nav_goal_go2w_planner.goal_pose_executor:main",
            "mppi_trajectory_lines = nav_goal_go2w_planner.mppi_trajectory_lines:main",
            "map_viz_layers = nav_goal_go2w_planner.map_viz_layers:main",
        ],
    },
)
