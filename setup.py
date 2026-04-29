import datetime

from setuptools import setup, find_packages

readme = open("readme.md").read()

fp = open("create3/__version__.py", "r").read()
VERSION = eval(fp.strip().split()[-1])

requirements = [
    # Core ROS 2
    "rclpy>=3.1.0",           # ROS 2 Python client
    "rosidl_default_generators",  # if you ever add custom messages
    
    # Math / geometry / perception (heavily used in utils/common + companion/perception)
    "numpy>=1.21.0",
    "scipy>=1.7.0",           # likely used in PID, algorithms, coords, collisions
    
    # Visualization & GUI (display/ folder + examples)
    "matplotlib>=3.5.0",      # controller.py, pid_tuner.py, point_cloud.py
    "pygame>=2.1.0",          # joystick + controller_example
    
    # Optional but very useful
    # "opencv-python-headless>=4.5.0",  # if you ever add camera fusion
]

VERSION += "+" + datetime.datetime.now().strftime("%Y%m%d%H%M")[2:]

setup(
    name="create3",
    version=VERSION,
    author="Scottcandy34",
    url="https://github.com/scottcandy34/irobot-create3-python-sdk/",
    packages=find_packages(where=".", include=["create3*"]),
    package_dir={"create3": "create3"},
    description="SDK for iRobot Create3 robot control over ROS2",
    long_description=readme,
    long_description_content_type="text/markdown",
    license="MIT",
    zip_safe=True,
    install_requires=requirements,
    python_requires=">=3.8",
    classifiers=[
        "Programming Language :: Python :: 3",
    ],
)