#!/usr/bin/env python3

import asyncio
import websockets
import json
import base64
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, BatteryState
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2


class WsSender(Node):
    def __init__(self):
        super().__init__('ws_sender')
        self.bridge = CvBridge()
        self.latest_scan = None
        self.latest_image = None
        self.latest_battery = None

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.get_logger().info('WsSender iniciado')

    def scan_callback(self, msg):
        self.latest_scan = {
            'type': 'scan',
            'ranges': [round(r, 2) for r in msg.ranges[::10]]
        }

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_img = cv2.resize(cv_img, (320, 240))
            _, jpeg = cv2.imencode(
                '.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            self.latest_image = {
                'type': 'image',
                'data': base64.b64encode(jpeg.tobytes()).decode('ascii')
            }
        except Exception as e:
            self.get_logger().error(f'Error en imagen: {e}')

    def battery_callback(self, msg):
        self.latest_battery = {
            'type': 'battery',
            'percentage': round(msg.percentage * 100, 1),
            'voltage': round(msg.voltage, 2),
            'current': round(msg.current, 2),
            'charge': round(msg.charge, 2),
            'capacity': round(msg.capacity, 2),
            'power_supply_status': msg.power_supply_status
        }

    def publish_velocity(self, linear_x, angular_z):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = float(linear_x)
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist_stamped)


ws_sender = None
connected_clients = set()


async def handler(websocket):
    connected_clients.add(websocket)
    print(f"Cliente conectado: {websocket.remote_address}")
    try:
        async def send_data():
            while True:
                out_data = {}
                if ws_sender.latest_image:
                    out_data['image'] = ws_sender.latest_image
                if ws_sender.latest_scan:
                    out_data['scan'] = ws_sender.latest_scan
                if ws_sender.latest_battery:
                    out_data['battery'] = ws_sender.latest_battery
                if out_data:
                    await websocket.send(json.dumps(out_data))
                await asyncio.sleep(0.033)

        async def receive_commands():
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get('type') == 'cmd_vel':
                        linear = data.get('linear', 0.0)
                        angular = data.get('angular', 0.0)
                        ws_sender.publish_velocity(linear, angular)
                except json.JSONDecodeError as e:
                    print(f"Error JSON: {e}")
                except Exception as e:
                    print(f"Error procesando comando: {e}")

        await asyncio.gather(send_data(), receive_commands())

    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        print(f"Error en handler: {e}")
    finally:
        connected_clients.remove(websocket)
        print(f"Cliente desconectado")


async def main():
    global ws_sender
    rclpy.init()
    ws_sender = WsSender()

    async def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(ws_sender, timeout_sec=0.01)
            await asyncio.sleep(0.01)

    async def start_server():
        async with websockets.serve(handler, "0.0.0.0", 8765):
            print("WebSocket servidor escuchando en puerto 8765")
            await asyncio.Future()

    await asyncio.gather(spin_ros(), start_server())


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Detenido")
