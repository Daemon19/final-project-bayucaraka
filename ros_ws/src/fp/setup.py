from setuptools import find_packages, setup

package_name = "fp"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="atma",
    maintainer_email="komanglangendria04@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "aruco_detection = fp.aruco_detection:main",
            "camera = fp.camera:main",
            "aruco_detector = fp.aruco_detector:main",
        ],
    },
)
