# mess2_logger

ROS 2 package for logging messages directly to `.csv` files; not recommended for binary message types with large arrays (i.e., images, point clouds) or significant volumes of data as these may result in poor performance.

## Overview

This package contains a general purpose data logger for ROS 2, designed to be accessible to users with limited experience in robotics software who may prefer human-readable data over ROS-specific serialization (i.e., rosbag2).

> [!WARNING]
> This package is not intended for high-bandwidth binary data (i.e., images, point clouds) as these message types may cause poor performance. Nodes in this package are written in Pythonfor ease of use and to enable dynamic topic subscription without the use of msg introspection, while backend tasks (i.e., msg parsing, log file i/o) utilize C++ functions via `pybind11` bindings.

## License

This package is released under an [MIT License](https://github.com/marinarasauced/mess2_logger/blob/main/LICENSE).

Authors: [Marina Nelson](https://github.com/marinarasauced) <br/>
Contributors: [Vik095](https://github.com/Vik095) ([ros msg parser in py](https://github.com/Vik095/Ros_msgs_parser)) <br/>
Affiliation: [ACE Lab](https://rvcowlagi-research.owlstown.net/)* <br/>
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

Nodes follow a standard naming pattern. For a logger node instance for a topic `'/ns/topic123'`, the name of the node is `'mess2_logger_ns_topic123'`. Logger node instances are spawned after the `'mess2_logger_initializer'` node, which is destroyed after all logger node instances have started.

### Launch Files

Three default launch options are available. All launch.py files include the following launch arguments: `namespace`, `log_dir_path`, `topic_names`, and `period`. All launch.py files contain logic in the `launch_setup` function to account for different cases where topics and namespaces are entered differently; i.e., `namespace='mess2'` with `topic_names='/topic1, /namespace2/topic2, topic3, namespace3/topic4'`.

The first launch option `logger.launch.py` is considered the default launch.py file and utilizes the default launch arguments.

```zsh
ros2 launch mess2_logger logger.launch.py namespace:='mess2' log_dir_path:='~/mess2/logs' topic_names:='topic1, topic2' period:=5.0
```

The second launch option `experiment.launch.py` auto generates a subdirectory in `log_dir_path` of the form `XXXX_YYYY-MM-DD_HH:MM:SS` where `XXXX` is the trial number (i.e., 0001, 0002, etc.) and the remainder is descriptive of the time of the start of the trial.

```zsh
ros2 launch mess2_logger experiment.launch.py namespace:='mess2' log_dir_path:='~/mess2/logs' topic_names:='topic1, topic2' period:=5.0
```

The third launch option `experiments.launch.py` adds a launch argument `trial_dir` such that the log_dir_path parameter is the joined path of the `log_dir_path` and `trial_dir` launch arguments.

```zsh
ros2 launch mess2_logger experiments.launch.py namespace:='mess2' log_dir_path:='~/mess2/logs' trial_dir:='0000' topic_names:='topic1, topic2' period:=5.0
```

### Log Files

Log files follow a standard format. The ROS 2 message parser function returns a list of complete fields for a given message type. These fields are the first row, and all subsequent rows are the corresponding values of each field. An example for a topic `/test_topic` with type `std_msgs/msg/Header` is shown below assuming the messages published via `ros2 topic pub /test_topic std_msgs/msg/Header`:

test_topic.csv
| msg.stamp.sec | msg.stamp.nanosec | msg.frame_id |
| --- | --- | --- |
| 0 | 0 |  |
| 0 | 0 |  |
| 0 | 0 |  |

## Parameters

The nodes in the `mess2_logger` package contain several ROS 2 parameters:

| Parameter | Description | Default Value |
| `namespace` | The namespace of the topics being logged. | `''` |
| `log_dir_path` | The relative path from the user's home directory to the location where the topics are to be logged. | `'~/mess2/logs'` |
| `trial_dir` | An optional subdirectory for keeping track of log files for multi-trial experiments. | `'0000'` |
| `topic_names` | The names of all topics to be logged in list form | `'test1, test2'` |
| `period` | The period of a timer function that checks if topics are advertised in seconds | `5.0` |


## pybind11 Interface

The `mess2_logger` package integrates C++ backend using `pybind11`. The following are examples of how to use the exposed functions and classes.

### `parse_msg(prefix, msg_type)`

- **Description:** Parses a ROS 2 message type and converts it list of all complete fields.
- **Arguments:** 
    - prefix: str = the name of the variable being evaluated in the subscription callback.
    - msg_type: str = the type of the message in the form `pkg_name/msg/Type`.
- **Returns:**
    - list[str] = All complete fields of the message type.

Example:

```py
from mess2_logger.msg_parser import parse_msg
parsed_msg: list[str] = parse_msg(prefix="msg", msg_type="std_msgs/msg/Header")
```

### `Logger(log_file_path: str, overwrite: bool = True)`

- **Description:** Handles logging to `.csv` file for logger node instance, including opening file, clearing file, appending rows to file, and closing file.
- **Arguments:** 
    - log_file_path: str = the absolute path to the log file
    - overwrite: bool = decides whether to overwrite preexisting data if a log file already exists at `log_file_path`

#### `log(data: list[str])`

- **Description:** Logs a row of data to the open `.csv` file.
- **Arguments:** 
    - data: list[str] = the data formatted in the subscription callback function using the parsed message.
- **Methods:**

Example:

```py
from mess2_logger.msg_logger import Logger
logger: Logger = Logger(log_file_path="/home/marinarasauced/mess2/logs/0000/test_topic.csv", overwrite=True)

logger.log(["msg.stamp.sec", "msg.stamp.nanosec", "msg.frame_id"])
logger.log(["0", "0", ""])
```
