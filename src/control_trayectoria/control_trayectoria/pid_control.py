import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import numpy as np

class PIDController(Node):

    def __init__(self):
        super().__init__('pid_control_node')

        # Parámetros físicos del robot
        self.L = 0.18  # distancia entre ruedas (m)
        self.R = 0.05  # radio de las ruedas (m)
        self.dt = 0.1  # periodo de muestreo (s)

        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocidades angulares de las ruedas
        self.wl = 0.0
        self.wr = 0.0

        # Subscripciones a encoders
        self.create_subscription(Float32, '/VelocityEncL', self.wl_callback, 10)
        self.create_subscription(Float32, '/VelocityEncR', self.wr_callback, 10)

        # Publicador de velocidades
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID para orientación
        self.kp_ang = 4.0
        self.ki_ang = 0.0
        self.kd_ang = 0.1
        self.integral_ang = 0.0
        self.prev_ang_error = 0.0

        # Waypoints
        self.waypoints = [[2, 0], [2, 2], [0, 2], [0, 0]]
        self.current_wp = 0

        # Timer de control
        self.create_timer(self.dt, self.control_loop)

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def update_odometry(self):
        # Velocidades tangenciales
        v_l = self.R * self.wl
        v_r = self.R * self.wr

        # Cinemática diferencial
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.L

        # Actualización de pose
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta += w * self.dt
        self.theta = self.normalize_angle(self.theta)

    def control_loop(self):
        self.update_odometry()

        if self.current_wp >= len(self.waypoints):
            self.stop_robot()
            return

        goal_x, goal_y = self.waypoints[self.current_wp]
        dx = goal_x - self.x
        dy = goal_y - self.y
        rho = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.theta)

        # PID Angular
        self.integral_ang += angle_error * self.dt
        derivative = (angle_error - self.prev_ang_error) / self.dt
        w = self.kp_ang * angle_error + self.ki_ang * self.integral_ang + self.kd_ang * derivative
        self.prev_ang_error = angle_error

        # Velocidad lineal (avanza solo si está alineado)
        v = 0.3 * rho if abs(angle_error) < 0.2 else 0.0

        # Publicar velocidades
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # Chequeo para avanzar al siguiente waypoint
        if rho < 0.1:
            self.current_wp += 1
            self.get_logger().info(f"Waypoint {self.current_wp} alcanzado.")

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Todos los waypoints alcanzados.")

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
