import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PIDSquareController(Node):
    def __init__(self):
        super().__init__('pid_square_controller')

        # Parámetros PID para control angular
        self.kp_ang = 2.0
        self.ki_ang = 0.0
        self.kd_ang = 0.1

        # Parámetros PID para control lineal
        self.kp_lin = 0.8
        self.ki_lin = 0.0
        self.kd_lin = 0.05

        # Estados internos del robot
        self.xr = 0.0
        self.yr = 0.0
        self.thetar = 0.0

        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

        self.integral_lin = 0.0
        self.prev_error_lin = 0.0

        # Waypoints para la trayectoria cuadrada
        self.waypoints = [
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0)
        ]
        self.current_index = 0

        # Publicador y suscriptor
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Temporizador
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.thetar = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def euler_from_quaternion(self, x, y, z, w):
        # Convierte un quaternion a ángulos de Euler
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 0.0, 0.0, yaw

    def control_loop(self):
        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            return

        goal_x, goal_y = self.waypoints[self.current_index]
        dx = goal_x - self.xr
        dy = goal_y - self.yr

        rho = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        alpha = self.normalize_angle(angle_to_goal - self.thetar)

        # Fase de alineación primero si no estamos bien orientados
        if abs(alpha) > 0.2:
            v = 0.0
            w = self.pid_angular(alpha)
        else:
            v = self.pid_linear(rho)
            w = self.pid_angular(alpha)

        cmd = Twist()
        cmd.linear.x = max(min(v, 0.3), -0.3)
        cmd.angular.z = max(min(w, 1.0), -1.0)
        self.cmd_pub.publish(cmd)

        if rho < 0.1:
            self.get_logger().info(f"Waypoint {self.current_index} alcanzado.")
            self.current_index += 1
            self.reset_integrals()

    def pid_linear(self, error):
        self.integral_lin += error
        derivative = error - self.prev_error_lin
        output = self.kp_lin * error + self.ki_lin * self.integral_lin + self.kd_lin * derivative
        self.prev_error_lin = error
        return output

    def pid_angular(self, error):
        self.integral_ang += error
        derivative = error - self.prev_error_ang
        output = self.kp_ang * error + self.ki_ang * self.integral_ang + self.kd_ang * derivative
        self.prev_error_ang = error
        return output

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def reset_integrals(self):
        self.integral_lin = 0.0
        self.prev_error_lin = 0.0
        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Trayectoria completada.")

def main(args=None):
    rclpy.init(args=args)
    node = PIDSquareController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
