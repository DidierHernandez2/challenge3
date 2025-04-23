import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class Controller(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # PID para control angular
        self.pid_theta = PID(2.5, 0.0, 0.1)

        # Estado actual estimado
        self.xr, self.yr, self.thetar = 0.0, 0.0, 0.0

        # Variables físicas del robot
        self.r = 0.05
        self.L = 0.18

        # Variables de velocidades
        self.vl = 0.0
        self.vr = 0.0
        self.servo_angle = 0.0

        # Waypoints para seguir trayectoria cuadrada
        self.waypoints = [[2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]]
        self.current_index = 0

        # Subscripciones a sensores
        self.create_subscription(Float32, '/VelocityEncL', self.vl_callback, 10)
        self.create_subscription(Float32, '/VelocityEncR', self.vr_callback, 10)
        self.create_subscription(Float32, '/ServoAngle', self.servo_callback, 10)

        # Publicador a /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Temporizador de control
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def vl_callback(self, msg):
        self.vl = msg.data

    def vr_callback(self, msg):
        self.vr = msg.data

    def servo_callback(self, msg):
        self.servo_angle = msg.data

    def control_loop(self):
        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            return

        # Cinemática diferencial
        v = (self.vr + self.vl) * self.r / 2
        w = (self.vr - self.vl) * self.r / self.L

        # Estimación del estado (integración simple)
        self.xr += v * math.cos(self.thetar) * self.timer_period
        self.yr += v * math.sin(self.thetar) * self.timer_period
        self.thetar += w * self.timer_period
        self.thetar = self.normalize_angle(self.thetar)

        # Cálculo de errores
        goal = self.waypoints[self.current_index]
        dx = goal[0] - self.xr
        dy = goal[1] - self.yr

        rho = math.hypot(dx, dy)
        desired_theta = math.atan2(dy, dx)
        alpha = self.normalize_angle(desired_theta - self.thetar)

        # Control de movimiento
        linear_vel = 0.3 * rho
        angular_vel = self.pid_theta.compute(alpha, self.timer_period)

        # Publicación de velocidades
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

        # Avanzar al siguiente waypoint
        if rho < 0.1:
            self.get_logger().info(f"Waypoint {self.current_index} alcanzado.")
            self.current_index += 1

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Todos los waypoints alcanzados.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
