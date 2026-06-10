from glob import glob

from setuptools import find_packages, setup


package_name = "nav_goal_go2w_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/worlds", glob("worlds/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Koki Tanaka",
    maintainer_email="67k.tanaka@gmail.com",
    description="Desktop 2D simulator for the Go2W goal-navigation stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = nav_goal_go2w_sim.sim_node:main",
            "keyboard_pause_node = nav_goal_go2w_sim.keyboard_pause_node:main",
            "gen_sim_map = nav_goal_go2w_sim.gen_sim_map_cli:main",
        ],
    },
)
