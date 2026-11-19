import socket, struct, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_udp_bridge')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info('Pont /cmd_vel -> Isaac Sim prêt')

    def cb(self, msg):
        self.sock.sendto(struct.pack('fff', msg.linear.x, msg.linear.y, msg.angular.z), ('127.0.0.1', 9870))

rclpy.init()
rclpy.spin(CmdVelBridge())
