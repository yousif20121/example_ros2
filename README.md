# ROS2 Python Examples

This repository contains a collection of example ROS2 nodes implemented in Python. The examples cover fundamental ROS2 concepts, including publishers, subscribers, services, actions, parameters, and more. It’s ideal for those who want to learn or improve their understanding of ROS2 with Python.

## Table of Contents
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Features
- **Basic ROS2 Node Setup**: Shows how to initialize and run a simple ROS2 node in Python.
- **Publishers & Subscribers**: Learn how to publish and subscribe to topics, send and receive messages, and handle different message types.
- **Services & Actions**: Examples of setting up services and actions for synchronous and asynchronous communication.
- **Parameters & Configurations**: Demonstrates the use of ROS2 parameters, including setting, retrieving, and updating parameter values.
- **Timers & Rate Control**: Manages node execution rates and timers for scheduled callbacks.

## Requirements
- **ROS2**: Compatible with ROS2 Humble.
- **Python**: Python 3.8 or later.

## Installation
1. Ensure ROS2 is installed on your system. Follow the [ROS2 installation guide](https://docs.ros.org/en/ros2).
2. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws
   git clone https://github.com/yourusername/example_ros2.git
   ```
3. Build the workspace:
   ```bash
   colcon build
   ```
4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage
Each example can be run independently. After sourcing the workspace, use `ros2 run` to start the desired example. For example:
```bash
ros2 run example_ros2 <node_name>
```
Each example includes comments and documentation to help you understand the code and its functionality.

## Contributing
Contributions are welcome! Feel free to submit issues, feature requests, or pull requests.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
