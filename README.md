Communication of sensors through ROS2

This ROS 2 node communicates with a sensor over TCP port 2000, following a specific communication protocol for sending and receiving messages. The sensor acts as the TCP server, while the ROS 2 node acts as the client to send commands and receive status updates.
Features

    Automatically sends the start command to the sensor upon launch with a configurable interval.
    Decodes and publishes status messages from the sensor to ROS 2 topics.
    Start/Stop control to send custom interval start command and stop the sensor communication.

Topics Published

The node publishes the following topics:
/sensor/supply_voltage

    Type: std_msgs/msg/UInt16
    Description: The supply voltage in milli-volts.

/sensor/env_temp

    Type: std_msgs/msg/Int16
    Description: The temperature in deci-Celsius.

/sensor/yaw

    Type: std_msgs/msg/Int16
    Description: The Yaw angle in deci-degrees.

/sensor/pitch

    Type: std_msgs/msg/Int16
    Description: The Pitch angle in deci-degrees.

/sensor/roll

    Type: std_msgs/msg/Int16
    Description: The Roll angle in deci-degrees.

Parameters

    interval (integer, default: 1000): The interval (in milliseconds) at which the start command will be sent to the sensor.


**To launch the node with a custom interval, use:**

ros2 launch sensor_package sensor_launch.py interval:=1000

This will start sending the start command to the sensor with a 1000 millisecond interval. You can adjust the interval value as needed.
Start and Stop the Sensor

To start or stop the sensor communication, you can modify the interval parameter or stop the node manually.
Command Messages
Start Command

    Command ID: 03
    Payload Size: 2 bytes
    Message Format:

    # 03 02 <interval_in_hex> <CR><LF>

Stop Command

    Command ID: 09
    Payload Size: 0 bytes
    Message Format:

    # 09 00 <CR><LF>

Response Messages
Status Message

    Command ID: 11
    Payload Size: 10 bytes
    Message Format:

    $ 11 10 <SUPPLY_VOLTAGE> <ENV_TEMP> <YAW> <PITCH> <ROLL> <CR><LF>

Code Overview

The Python-based ROS 2 node uses the socket library to communicate with the sensor over TCP and rclpy to interact with ROS 2. The node sends commands to the sensor, receives status messages, decodes them, and publishes the values to the appropriate ROS 2 topics.

This should provide clear instructions on how to set up and use the ROS 2 node for your sensor communication project! Let me know if you need anything else.
