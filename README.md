# mess2_logger_cpp
ROS2 package for logging image messages on topics directly to .jpg files. 

## Overview
This package contains a ROS2 logger node that dynamically creates subscription instances for topics of the message type sensor_msgs::msg::Image.

## License
This package is released under an [Apache 2.0 license](https://github.com/marinarasauced/mess2_logger_cpp/blob/main/LICENSE).

**Authors:** [Marina Nelson](https://github.com/marinarasauced) <br/>
**Affiliation:** [ACE Lab](https://rvcowlagi-research.owlstown.net/) <br/>
**Maintainer:** Marina Nelson, marinarasauced@outlook.com

This package has been tested in ROS2 Humble and Jazzy on Ubuntu 22.04 and 24.04, respectively.

## Installation
### Prerequisites
- Ubuntu 22.04 or Ubuntu 24.04
- ROS2 Humble or Jazzy
- ROS2 Workspace

These instructions assume that you meet the above prerequisites. Do not proceed if you have not already installed ROS2 Humble or Jazzy on Ubuntu 22.04 or 24.04, respectively, or if you have not created a ROS2 workspace.

### Building
Clone the repository into your ROS2 workspace's `src` directory:

```zsh
cd ~/your_ws/src
git clone https://github.com/marinarasauced/mess2_logger_cpp.git
```

Update your ROS2 dependencies:

```zsh
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

Compile the package using `colcon`:

```
cd ~/your_ws
colcon build --symlink-install --packages-select mess2_logger_cpp
```

Source the setup script:

```
source ~/your_ws/install/setup.zsh
```

## Usage
Start the `log_topics_to_jpgs` node with:

> [!CAUTION]  
> Before running the node, please read the below documentation. The directory in which the log .csv files are written to is CLEARED on run.

```zsh
ros2 launch mess2_logger_cpp template.launch.py
```

When you start the node,

## Config Files

- **`config/template.yaml`**

    Defines the default values for the `dir_logs`, `dirs_sub`, `names_actors`, and `topics` parameters for use with `template.launch.py`.

## Launch Files

- **`template.launch.py`**

    Provides a template for building launch files for the `log_topics_to_jpgs` node using config files.

## Nodes

### log_topics_to_jpgs

Create subscriptions for a set of image topics and record messages on said topics as .jpg files.

Parameters are used to define the path at which the images are saved and the topics to which the node subscribes. On run, a save directory is created at `~/dir_logs/dirs_sub[i]/names_actors[i]/images/` for each element of `dirs_sub` and `names_actors`. If this directory contains files, those files are cleared before new images are saved.

#### Subscriptions

Subscriptions are created dynamically using the `topics` parameter. Each topic should be of the type `sensor_msgs::msg::Image`.

#### Parameters

- **`dir_logs`** (string, default: "Projets/testing")

	The relative path from the home directory to the logs directory.

- **`dirs_sub`** (list\[string\], default: \["flir"\])

	The subdirectories in which the log .csv files are written; i.e., actor/actor1/. The relative path from home to `dir_logs` and subsequently `dirs_sub` is CLEARED when the node starts.

- **`names_actors`** (list\[string\], default: \["flir1", "flir2"\])

    he names associated with each image topic. Each name corresponds to a subdirectory under `dir_logs` where the images for that topic are saved.

- **`topics`** (list\[string\], default: \["/mess2/flir1/image_raw", "/mess2/flir2/image_raw"\])

	The topics to log.
