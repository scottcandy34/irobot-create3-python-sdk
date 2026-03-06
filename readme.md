# ROS2 Examples for iRobot Create3 - Jazzy
This repository provides Python examples and a library for integrating the iRobot Create3 robot with ROS2 (Jazzy distribution). It includes multithreading support, topic subscriptions/publishers, actions, services, debugging tools, and utility functions for controlling the robot, handling sensors, and interfacing with peripherals like Raspberry Pi or a remote PC.

The code is structured as a modular library that can be imported into your ROS2 nodes. It handles common tasks like navigation, LED control, audio playback, sensor data processing, and more.
**Note:** This library is inspired by and bases its command structures on the official iRobot Education Python SDK for Bluetooth (https://github.com/iRobotEducation/irobot-edu-python-sdk). However, it is adapted for ROS2 over Wi-Fi or wired connections.
## Features
- **Multithreading Support:** Uses ROS2 executors and callback groups for efficient, non-blocking operations.
- **Subscriptions & Publishers:** Handles topics for odometry, IR sensors, hazards, IMU, battery, docking, and more.
- **Actions:** Supports navigation to positions, driving arcs/distances, rotations, docking/undocking, LED animations, and audio sequences. With and without sending goals.
- **Services:** Includes pose reset and other robot services.
- **Tools & Utilities:** Conversion tools for positions, colors, lidar processing, joystick mapping, and more.
- **Debugging:** A built-in debugger monitors topics, services, actions, and uptime for reliability.
- **Peripheral Integration:** Examples for Raspberry Pi (e.g., lidar, ultrasonics, servos) and remote PC (e.g., PS controller input).
- **Music & Notes:** Helper constants for playing notes and sequences on the robot.

## Tested Environments
- Ubuntu 24.04 LTS with ROS2 Jazzy (primary testing platform).
- Should be compatible with ROS2 Iron or Humble, but untested—please report issues if you try it.

## Prerequisites
- iRobot Create3 robot with ROS2 firmware installed.
- Ubuntu (or compatible Linux) with ROS2 Jazzy installed.
- **Optional:** Raspberry Pi attached to the Create3 for additional sensors (e.g., lidar).
- **Optional:** PS controller for remote PC input.

## Setup for iRobot Create3
Before using this library, set up your iRobot Create3 robot with ROS2. Follow the official iRobot Create3 documentation for hardware setup, firmware installation, and ROS2 configuration:
- Official Docs: https://irobot.github.io/create3_docs/
	- Start with the "Getting Started" guide for unboxing, Wi-Fi setup, and ROS2 installation.
	- Ensure the robot is running the latest firmware compatible with ROS2 Jazzy.
	- Connect the robot to your network (Wi-Fi or Ethernet) and verify ROS2 topics are available (e.g., via ros2 topic list).

If using a Raspberry Pi on the Create3:
- Install ROS2 on the Pi and configure it as a ROS2 node.
- Attach sensors like lidar or ultrasonics as needed.

## Installation
1. **Clone the Repository:**

	```sh
	git clone https://github.com/scottcandy34/irobot-create3-python-sdk.git
	cd irobot-create3-python-sdk
	```

2. **Install Dependencies:**
   - Follow the official documentation on iRobot Create3. https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/

Follow the official documentation on iRobot Create3. https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/

## Usage
This library is designed to replace your ROS2 packages. Here's a basic example to get started:
1. **Import the Library:**
  	Create a Python script (e.g., my_robot_node.py):
	```python
	from create3.nodes import RobotNode
	from create3.music import Note

	# Initialize the Robot Node
	robot = RobotNode()

	# Example: Drive forward 1 meter
	robot.drive_distance(1.0)  # Distance in meters

	# Example: Rotate 90 degrees
	robot.rotate_angle(90)  # Angle in degrees

	# Example: Set LED colors
	robot.set_lights_on_rgb(255, 0, 0)  # Red

	# Example: Play a note
	robot.play_note(Note.C4, 0.5)  # Play C4 for 0.5 seconds

	# Shutdown cleanly
	robot.shutdown()
	```

1. **Run the Node:**
	```sh
	python3 my_robot_node.py 
	```

2. **Advanced Usage:**
   - **Raspberry Pi Node:** Use RpiNode for lidar/ultrasonic handling.
   - **PC Node:** Use PcNode for controller input and rumble feedback.
   - Access sensor data via methods like robot.get_position().x or robot.get_battery_level().
   - **Use tools:** robot.tools.robot.getIrAngle(0) or rpi.tools.lidar.find_line().
   - **Debugging:** The built-in debugger logs topic availability, uptime warnings, and errors.

   For full API details, refer to the source files (e.g., [actions.py](create3/interfaces/actions.py), [subscriptions.py](create3/interfaces/subscriptions.py), [tools.py](create3/tools.py)).

## Contributing
Feel free to open issues or pull requests for improvements, bug fixes, or additional features.

## License
This project is licensed under the [MIT License](LICENSE.md)

## Acknowledgments
- Based on commands from the iRobot Education Python SDK: https://github.com/iRobotEducation/irobot-edu-python-sdk

