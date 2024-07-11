#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import sys

class RosMqttBridge(Node):
    
    def __init__(self, msg_type, ros_topic, mqtt_topic, host="broker.emqx.io", port=1883):
        super().__init__('ros_mqtt_bridge')
        self.get_logger().info('ros_mqtt_bridge node has been started') 

        # MQTT Client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.connect(host, port, 60)
        self.mqtt_client.loop_start()  # Start the MQTT client in a non-blocking background thread
        self.get_logger().info(f'Connected to {host}:{port}') 

        self.mqtt_topic = mqtt_topic
        self.msg_type = msg_type

        # The topic that this node will subscribe to in ROS communication
        self.ros_sub = self.create_subscription(self.msg_type, ros_topic, self.ros_callback, 10)
        self.get_logger().info(f'This node will subscribe to {ros_topic} and publish to MQTT topic {self.mqtt_topic}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker with result code " + str(rc))

    def ros_callback(self, msg):
        if self.msg_type == Twist:
            self.publish_twist_message(msg)
        elif self.msg_type == String:  # Handle std_msgs/String messages
            self.publish_string_message(msg)
        else:
            self.get_logger().error(f"Unsupported message type: {self.msg_type}")

    def publish_twist_message(self, msg):
        message = {
            "linear": {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z
            },
            "angular": {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z
            }
        }
        message_str = json.dumps(message)
        self.get_logger().info("Publishing cmd_vel to MQTT: %s" % message_str)
        self.mqtt_client.publish(self.mqtt_topic, message_str)

    def publish_string_message(self, msg):
        message_str = msg.data  # Assuming String message has a `data` field
        self.get_logger().info(f"Publishing String message to MQTT: {message_str}")
        self.mqtt_client.publish(self.mqtt_topic, message_str)

def main(args=None):
    if len(sys.argv) < 5 or len(sys.argv) > 7:
        print("Usage: python ros2_mqtt_bridge.py <msg_type> <ros_topic> <mqtt_topic> [host] [port]")
        sys.exit(1)
    
    msg_type_str = sys.argv[1]
    ros_topic = sys.argv[2]
    mqtt_topic = sys.argv[3]
    host = sys.argv[4] if len(sys.argv) > 4 else "broker.emqx.io"
    port = int(sys.argv[5]) if len(sys.argv) > 5 else 1883
    
    rclpy.init(args=args)
    
    # Determine the message type
    if msg_type_str == 'Twist':
        msg_type = Twist
    elif msg_type_str == 'String':  # Add support for String message type
        msg_type = String
    else:
        msg_type = String
        print(f"Unsupported message type: The message will be treated as String")
        return
    
    node = RosMqttBridge(msg_type, ros_topic, mqtt_topic, host, port)
    rclpy.spin(node)
    
    # Clean up
    node.mqtt_client.loop_stop()  # Stop the MQTT client
    rclpy.shutdown()

if __name__ == '__main__':
    main()
