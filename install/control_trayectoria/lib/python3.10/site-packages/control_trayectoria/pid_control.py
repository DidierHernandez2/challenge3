import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ForwardPublisher(Node):
    def __init__(self):
        super().__init__('forward_publisher')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publica comando cada 0.1s -> 10Hz
        self.create_timer(0.1, self.publish_forward)
        self.get_logger().info('ForwardPublisher iniciado: publicando velocidad constante hacia adelante.')

    def publish_forward(self):
        msg = Twist()
        msg.linear.x = 0.2  # velocidad constante hacia adelante
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = ForwardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
