import datetime

from setuptools import setup, find_packages

readme = open("readme.md").read()

fp = open("create3/__version__.py", "r").read()
VERSION = eval(fp.strip().split()[-1])

requirements = [
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
    license="MIT",
    packages=find_packages("create3"),
    zip_safe=True,
    install_requires=requirements,
    classifiers=[
        "Programming Language :: Python :: 3",
    ],
)