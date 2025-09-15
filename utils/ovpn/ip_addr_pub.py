#!/usr/bin/env python3
import rclpy
from rclpy.qos import qos_profile_sensor_data
from msg_context.loader import ClientInfo
import subprocess
import time
import json

def read_platform_id_from_json():
    file_path = '/root/auspex_params/platform_properties/platform_properties.json'
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data.get('platform_id', 'unknown')
    except Exception as e:
        print(str(e))
        return 'unknown'

def get_tun0_ip():
    try:
        result = subprocess.run(
            ['ip', '-4', 'addr', 'show', 'tun0'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        if result.returncode != 0:
            return "0.0.0.0"
        for line in result.stdout.split('\n'):
            line = line.strip()
            if line.startswith('inet '):
                return line.split()[1].split('/')[0]
        return "0.0.0.0"
    except Exception as e:
        print(str(e))
        return "0.0.0.0"

def main():
    rclpy.init()
    node = rclpy.create_node('client_info_pub')
    pub = node.create_publisher(ClientInfo, 'clients', qos_profile_sensor_data)

    try:
        while rclpy.ok():
            msg = ClientInfo()
            msg.platform_id = read_platform_id_from_json()
            msg.ip_address = get_tun0_ip()
            #platform_id = read_platform_id_from_json()
            #ip_address = get_tun0_ip()

            if msg.platform_id == 'unknown' or msg.ip_address == '0.0.0.0':
                node.get_logger().info(f'error: {msg.platform_id} @ {msg.ip_address}')
            else:
                pub.publish(msg)
                node.get_logger().info(f'published: {msg.platform_id} @ {msg.ip_address}')

            time.sleep(10)
    except Exception as e:
        node.get_logger().error(f'exception: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
