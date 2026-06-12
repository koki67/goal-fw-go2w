from glob import glob
from setuptools import find_packages, setup

package_name = "nav_goal_go2w_web"
setup(
    name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/www", ["www/index.html"]),
        (f"share/{package_name}/www/css", glob("www/css/*")),
        (f"share/{package_name}/www/js", glob("www/js/*")),
        (f"share/{package_name}/www/vendor", glob("www/vendor/*")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Koki Tanaka", maintainer_email="67k.tanaka@gmail.com",
    description="Browser operator interface for Go2W goal navigation and map preparation.",
    license="MIT", tests_require=["pytest"],
    entry_points={"console_scripts": ["prep_web_node = nav_goal_go2w_web.prep_web_node:main"]},
)
