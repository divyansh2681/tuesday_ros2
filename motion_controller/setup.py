from setuptools import setup

package_name = "motion_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/full_stack.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="divyansh",
    maintainer_email="divyanshagarwal81@gmail.com",
    description="Motion controller that switches between GUI and clock pose.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "controller = motion_controller.controller:main",
        ],
    },
)
