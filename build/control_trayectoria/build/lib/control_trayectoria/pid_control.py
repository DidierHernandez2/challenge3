import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_control_node')

        # QoS para datos de sensores
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Variables del robot
        self.r = 0.05  # radio de rueda [m]
        self.l = 0.18  # distancia entre ruedas [m]

        # Estado actual
        self.vl = 0.0
        self.vr = 0.0
        self.theta_servo = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Controlador PID
        self.kp_rho = 1.5
        self.kp_alpha = 4.0
        self.kp_beta = -1.5

        # Waypoints del cuadrado
        self.waypoints = [
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0)
        ]
        self.current_goal_index = 0

        # Subscribers
        self.create_subscription(Float32, '/VelocityEncL', self.vl_callback, sensor_qos)
        self.create_subscription(Float32, '/VelocityEncR', self.vr_callback, sensor_qos)
        self.create_subscription(Float32, '/ServoAngle', self.servo_callback, sensor_qos)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(0.1, self.control_loop)  # 10 Hz

    def vl_callback(self, msg):
        self.vl = msg.data

    def vr_callback(self, msg):
        self.vr = msg.data

    def servo_callback(self, msg):
        self.theta_servo = msg.data

    def control_loop(self):
        # Actualiza odometría
        dt = 0.1
        v = (self.r / 2.0) * (self.vr + self.vl)
        w = (self.r / self.l) * (self.vr - self.vl)

        self.theta += w * dt
        self.theta = self.normalize_angle(self.theta)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        if self.current_goal_index >= len(self.waypoints):
            self.stop_robot()
            return

        goal_x, goal_y = self.waypoints[self.current_goal_index]
        dx = goal_x - self.x
        dy = goal_y - self.y

        rho = math.hypot(dx, dy)
        alpha = self.normalize_angle(math.atan2(dy, dx) - self.theta)
        beta = self.normalize_angle(-self.theta - alpha)

        # Ley de control de pose polar
        v_cmd = self.kp_rho * rho
        w_cmd = self.kp_alpha * alpha + self.kp_beta * beta

        # Saturación
        v_cmd = max(min(v_cmd, 0.3), -0.3)
        w_cmd = max(min(w_cmd, 1.5), -1.5)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

        if rho < 0.1:
            self.current_goal_index += 1
            self.get_logger().info(f"Waypoint {self.current_goal_index} alcanzado.")

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Todos los waypoints alcanzados.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
