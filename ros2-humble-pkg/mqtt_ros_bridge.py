#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import sys

class MqttRosBridge(Node):
    
    def __init__(self, msg_type, topic_name, mqtt_topic, host="broker.emqx.io", port=1883):
        super().__init__('mqtt_ros_bridge')
        self.get_logger().info('mqtt_ros_bridge node has been started') 

        # MQTT Client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(host, port, 60)
        
        self.mqtt_client.loop_start()  # Start the MQTT client in a non-blocking background thread
        self.get_logger().info(f'Connected to {host}:{port}') 

        self.mqtt_topic = mqtt_topic
        self.msg_type = msg_type
        self.topic_name = topic_name 
        self.ros_pub = self.create_publisher(self.msg_type, self.topic_name, 10)
        self.get_logger().info(f'This node will publish to {self.topic_name} with message type of {self.msg_type}') 

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker with result code " + str(rc))
        self.mqtt_client.subscribe(self.mqtt_topic)  # Subscribe to the MQTT topic where messages are published

    def on_mqtt_message(self, client, userdata, msg):
        # MQTT message received
        self.get_logger().info(msg.payload.decode())

        # Process the message based on the message type
        if self.msg_type == Twist:
            self.publish_twist_message(msg.payload)
        elif self.msg_type == String:  # Handle std_msgs/String messages
            self.publish_string_message(msg.payload)
        else:
            self.get_logger().error(f"Unsupported message type: {self.msg_type}")

    def publish_twist_message(self, payload):
        try:
            message = json.loads(payload.decode())
            twist_msg = Twist()
            twist_msg.linear.x = message['linear']['x']
            twist_msg.linear.y = message['linear']['y']
            twist_msg.linear.z = message['linear']['z']
            twist_msg.angular.x = message['angular']['x']
            twist_msg.angular.y = message['angular']['y']
            twist_msg.angular.z = message['angular']['z']

            self.ros_pub.publish(twist_msg)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding MQTT message: {e}")
        except KeyError as e:
            self.get_logger().error(f"Missing key in MQTT message: {e}")

    def publish_string_message(self, payload):
        try:
            message_str = payload.decode()
            string_msg = String(data=message_str)
            self.ros_pub.publish(string_msg)
        except Exception as e:
            self.get_logger().error(f"Error handling String message: {e}")

def main(args=None):
    if len(sys.argv) < 4 or len(sys.argv) > 6:
        print("Usage: python mqtt_ros_bridge.py <msg_type> <topic_name> <mqtt_topic> [host] [port]")
        sys.exit(1)
    
    msg_type_str = sys.argv[1]
    topic_name = sys.argv[2]
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
    
    node = MqttRosBridge(msg_type, topic_name, mqtt_topic, host, port)
    rclpy.spin(node)
    
    # Clean up
    node.mqtt_client.loop_stop()  # Stop the MQTT client
    rclpy.shutdown()

if __name__ == '__main__':
    main()
