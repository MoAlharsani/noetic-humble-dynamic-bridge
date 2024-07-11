# ROS-Connection-Bridge

This repository contains ROS1 and ROS2 packages that enable communication between hosts running different ROS distributions (e.g., ROS1 Noetic, ROS2 Foxy, or ROS2 Humble) within the same network or over the internet. The communication is facilitated using either public or local MQTT brokers. This package allows seamless data exchange between different ROS distributions or even the same ROS versions across different machines. 

## Installation

### Prerequisites

- Ensure you have `pip` installed:

    ```bash
    sudo apt install python3-pip
    ```

- Install `paho-mqtt` library:

    ```bash
    pip install paho-mqtt
    ```

### ROS1 (Noetic)

1. Create a workspace and the package:

    ```bash
    mkdir -p ~/ros_connection_bridge_catkin_ws/src
    cd ~/ros_connection_bridge_catkin_ws/src
    catkin_create_pkg ros_connection_bridge rospy std_msgs geometry_msgs
    ```

2. Replace `package.xml` with the provided one:

    ```bash
    cd ros_connection_bridge
    rm -rf package.xml
    curl -O https://raw.githubusercontent.com/MoAlharsani/ros-connection-bridge/main/ros1-noetic-pkg/package.xml
    ```

3. Add the Python scripts:

    ```bash
    cd src
    curl -O https://raw.githubusercontent.com/MoAlharsani/ros-connection-bridge/main/ros1-noetic-pkg/ros_mqtt_bridge.py
    curl -O https://raw.githubusercontent.com/MoAlharsani/ros-connection-bridge/main/ros1-noetic-pkg/mqtt_ros_bridge.py
    chmod +x ros_mqtt_bridge.py mqtt_ros_bridge.py
    ```

4. Build the workspace:

    ```bash
    cd ~/ros_connection_bridge_catkin_ws
    catkin_make
    source devel/setup.bash
    ```

5. To run the nodes:

    ```bash
    rosrun ros_connection_bridge mqtt_ros_bridge.py <msg_type> <ros_topic> <mqtt_topic> [host] [port]
    rosrun ros_connection_bridge ros_mqtt_bridge.py <msg_type> <ros_topic> <mqtt_topic> [host] [port]
    ```

### ROS2 (Humble)

1. Create a workspace and the package:

    ```bash
    mkdir -p ~/ros_connection_bridge_ros2_ws/src
    cd ~/ros_connection_bridge_ros2_ws/src
    ros2 pkg create ros_connection_bridge --build-type ament_python --dependencies rclpy
    ```

2. Replace `setup.py` with the provided one:

    ```bash
    cd ros_connection_bridge
    rm -rf setup.py
    curl -O https://raw.githubusercontent.com/MoAlharsani/ros-connection-bridge/main/ros2-humble-pkg/setup.py
    ```

3. Add the Python scripts:

    ```bash
    cd ros_connection_bridge
    curl -O https://raw.githubusercontent.com/MoAlharsani/ros-connection-bridge/main/ros2-humble-pkg/mqtt_ros_bridge.py
    curl -O https://raw.githubusercontent.com/MoAlharsani/ros-connection-bridge/main/ros2-humble-pkg/ros_mqtt_bridge.py
    chmod +x mqtt_ros_bridge.py ros_mqtt_bridge.py
    ```

4. Build the workspace:

    ```bash
    cd ~/ros_connection_bridge_ros2_ws
    colcon build
    source install/setup.bash
    ```

5. To run the nodes:

    ```bash
    ros2 run ros_connection_bridge mqtt_ros_bridge <msg_type> <ros_topic> <mqtt_topic> [host] [port]
    ros2 run ros_connection_bridge ros_mqtt_bridge <msg_type> <ros_topic> <mqtt_topic> [host] [port]
    ```


## Usage

### Example: Communicating between ROS2 and ROS1 nodes

#### Scenario

We have two devices:
- Device A running ROS2 (Humble)
- Device B running ROS1 (Noetic)

We want to send messages from a ROS2 node on Device A to a ROS1 node on Device B using an MQTT broker.

#### Steps

1. **On Device A (ROS2 Humble)**:

    - Run the ROS2 node to publish messages to the MQTT broker:

        ```bash
        humble@robot:~$ ros2 run ros_connection_bridge ros_mqtt_bridge String /chatter /chatter broker.hivemq.com 1883
        ```

    - Run a sample talker node to publish messages to the `/chatter` topic:

        ```bash
        humble@robot:~$ ros2 run demo_nodes_cpp talker
        ```

2. **On Device B (ROS1 Noetic)**:

    - Run the ROS1 node to subscribe to the MQTT broker and publish messages to the `/chatter` topic:

        ```bash
        noetic@pc:~$ rosrun ros_connection_bridge mqtt_ros_bridge.py String /chatter /chatter broker.hivemq.com 1883
        ```

    - Run a sample listener node to listen to messages from the `/chatter` topic:

        ```bash
        noetic@pc:~$ rosrun rospy_tutorials listener
        ```

#### Explanation

- The `ros_mqtt_bridge` node on Device A subscribes to the `/chatter` topic in ROS2 and publishes the messages to the MQTT broker (`broker.hivemq.com` on port `1883`).
- The `mqtt_ros_bridge` node on Device B subscribes to the MQTT broker and republishes the messages to the `/chatter` topic in ROS1.
- The listener node on Device B will then receive the messages published by the talker node on Device A, demonstrating successful communication between ROS2 and ROS1 nodes via an MQTT broker.

## Supported Message Types

The package currently supports the following message types:

- `Twist`
- `String`

You can easily modify the code to support other message types by adding the appropriate message handling code.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request with your changes.

## License

This project is licensed under the MIT License.
