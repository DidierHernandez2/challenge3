import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')

        # Parámetros del PID
        self.kp = 1.5
        self.ki = 0.0
        self.kd = 0.1

        # Estado inicial
        self.xr, self.yr, self.thetar = 0.0, 0.0, 0.0
        self.integral_error = 0.0
        self.prev_error = 0.0

        # Lista de waypoints [x, y]
        self.waypoints = [
            [2.0, 0.0],  # Derecha
            [2.0, 2.0],  # Arriba
            [0.0, 2.0],  # Izquierda
            [0.0, 0.0]   # Abajo (cierra el cuadrado)
        ]
        self.current_index = 0

        # Publicador a /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer de control
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.control_loop)

    def control_loop(self):
        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            return

        goal = self.waypoints[self.current_index]
        dx = goal[0] - self.xr
        dy = goal[1] - self.yr

        rho = math.hypot(dx, dy)  # distancia al objetivo
        angle_to_goal = math.atan2(dy, dx)
        alpha = self.normalize_angle(angle_to_goal - self.thetar)

        # Controlador PID simple (solo proporcional en distancia, PID en ángulo)
        v = 0.5 * rho
        error = alpha
        self.integral_error += error
        derivative = error - self.prev_error
        w = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        self.prev_error = error

        # Comando de velocidad
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_vel_pub.publish(cmd)

        # Simulación de avance (opcional si no tienes odometría)
        self.xr += v * math.cos(self.thetar) * 0.1
        self.yr += v * math.sin(self.thetar) * 0.1
        self.thetar += w * 0.1

        if rho < 0.1:
            self.current_index += 1
            self.get_logger().info(f"Waypoint {self.current_index} alcanzado.")

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Todos los waypoints alcanzados.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()