from setuptools import find_packages, setup

package_name = "clock_pose_issuer"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="divyansh",
    maintainer_email="divyanshagarwal81@gmail.com",
    description="Publishes a 6D pose based on the minute hand of the system clock.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "clock_publisher = clock_pose_issuer.clock_publisher:main",
        ],
    },
)
