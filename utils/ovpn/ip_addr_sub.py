#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from msg_context.loader import ClientInfo

def main():
    rclpy.init()
    node = rclpy.create_node('client_info_subscriber')

    def callback(msg):
        node.get_logger().info(f"received: {msg.platform_id} @ {msg.ip_address}")

    node.create_subscription(ClientInfo, 'clients', callback, qos_profile_sensor_data)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
