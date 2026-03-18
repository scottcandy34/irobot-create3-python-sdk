import datetime

from setuptools import setup, find_packages

readme = open("readme.md").read()

fp = open("create3/__version__.py", "r").read()
VERSION = eval(fp.strip().split()[-1])

requirements = [
    "rclpy>=7.1.9",
    "std_msgs>=5.3.6",
    "nav_msgs>=5.3.6",
    "sensor_msgs>=5.3.6",
    "geometry_msgs>=5.3.6",
    "builtin_interfaces>=2.0.3",
    "irobot_create_msgs>=3.0.0",
]

VERSION += "+" + datetime.datetime.now().strftime("%Y%m%d%H%M")[2:]

setup(
    name="create3",
    version=VERSION,
    author="Scottcandy34",
    url="https://github.com/scottcandy34/irobot-create3-python-sdk/",
    package_dir={"":"create3"},
    description="SDK for iRobot Create3 robot control over ROS2",
    long_description=readme,
    long_description_content_type="text/markdown",
    licence="MIT",
    packages=find_packages("create3"),
    zip_safe=True,
    install_requires=requirements,
    classifiers=[
        "Programming Language :: Python :: 3",
    ],
)