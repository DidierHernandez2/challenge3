import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AngularPublisher(Node):
    def __init__(self):
        super().__init__('angular_publisher')
        # Publicador para cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publica comando cada 0.1s (10Hz)
        self.create_timer(0.1, self.publish_angular)
        self.get_logger().info('AngularPublisher iniciado: publicando velocidad angular constante.')

    def publish_angular(self):
        msg = Twist()
        msg.linear.x = 0.0    # sin componente lineal
        msg.angular.z = 0.5   # velocidad angular constante (rad/s)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = AngularPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
