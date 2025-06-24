from setuptools import setup

package_name = "gui_pose_issuer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="divyansh",
    maintainer_email="divyanshagarwal81@gmail.com",
    description="GUI to click a pose on a clock face and publish it.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gui_publisher = gui_pose_issuer.gui_publisher:main",
        ],
    },
)
