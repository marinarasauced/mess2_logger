# mess2_logger

ROS 2 package for logging messages directly to `.csv` files; not recommended for binary message types with large arrays (i.e., images, point clouds) or significant volumes of data as these may result in poor performance.

## Overview

This package contains a general purpose data logger for ROS 2, designed to be accessible to users with limited experience in robotics software who may prefer human-readable data over ROS-specific serialization (i.e., rosbag2).

> [!WARNING]
> This package is not intended for high-bandwidth binary data (i.e., images, point clouds) as these message types may cause poor performance. Nodes in this package are written in Pythonfor ease of use and to enable dynamic topic subscription without the use of msg introspection, while backend tasks (i.e., msg parsing, log file i/o) utilize C++ functions via `pybind11` bindings.

## License

This package is released under an [MIT License](https://github.com/marinarasauced/mess2_logger/blob/main/LICENSE).

Authors: [Marina Nelson](https://github.com/marinarasauced)
Contributors: [Vik095](https://github.com/Vik095) ([ros msg parser in py](https://github.com/Vik095/Ros_msgs_parser))
Affiliation: *[ACE Lab](https://rvcowlagi-research.owlstown.net/)
Maintainer: Marina Nelson, marinarasauced@outlook.com

**The original version of the package was developed for the ACE Lab at WPI. This current version is streamlined to improve code legibility and improve performance.*

## Installation

### Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy
- ROS 2 Workspace

The following assumes that you have already completed the above prerequisites.

### Steps

Clone the repository into the `src` directory of your ROS 2 workspace:

```zsh
cd ~/your_ws/src
git clone https://github.com/marinarasauced/mess2_logger
```

Update ROS 2 dependencies and compile the package:

```zsh
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mess2_logger
```

Source the setup script:

```zsh
source ~/your_wsinstall/setup.zsh
```

## Usage

### Nodes

The `mess2_logger` currently includes Python nodes that dynamically subscribe to ROS 2 topics and logs the corresponding messages to `.csv` files. A single executable starts nodes of two types: an initializer and logger instances. The initializer node is responsible for declaring parameters for the executable and ensuring that the write path for the logs exists. The logger node instances are responsible for logging their respective topics to the `.csv` files at the write path.

To start the nodes, execute the following command in a terminal:

```zsh
ros2 run mess2_logger main.py
```