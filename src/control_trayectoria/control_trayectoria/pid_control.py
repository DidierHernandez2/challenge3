import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')

        # Parámetros del PID (angular)
        self.kp = 1.5
        self.ki = 0.0
        self.kd = 0.1

        # Parámetros físicos del robot
        self.R = 0.05   # Radio de rueda (m)
        self.L = 0.18   # Distancia entre ruedas (m)

        # Estado del robot
        self.xr = 0.0
        self.yr = 0.0
        self.thetar = 0.0

        # Variables del PID
        self.integral_error = 0.0
        self.prev_error = 0.0

        # Tiempo anterior
        self.prev_time = self.get_clock().now()

        # Lista de waypoints [x, y]
        self.waypoints = [
            [2.0, 0.0],  # Derecha
            [2.0, 2.0],  # Arriba
            [0.0, 2.0],  # Izquierda
            [0.0, 0.0]   # Abajo (cierra el cuadrado)
        ]
        self.current_index = 0

        # Subscriptores a encoders
        self.wr = 0.0
        self.wl = 0.0
        self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, 10)
        self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, 10)

        # Publicador a /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(0.1, self.control_loop)

    def encR_callback(self, msg):
        self.wr = msg.data

    def encL_callback(self, msg):
        self.wl = msg.data

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        # Evitar división por cero
        if dt == 0:
            return

        # Calcular velocidades lineales de ruedas
        vL = self.R * self.wl
        vR = self.R * self.wr

        # Velocidades del robot
        v = (vR + vL) / 2
        w = (vR - vL) / self.L

        # Actualizar posición del robot
        self.xr += v * math.cos(self.thetar) * dt
        self.yr += v * math.sin(self.thetar) * dt
        self.thetar += w * dt
        self.thetar = self.normalize_angle(self.thetar)

        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            return

        # Objetivo actual
        goal = self.waypoints[self.current_index]
        dx = goal[0] - self.xr
        dy = goal[1] - self.yr

        rho = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        alpha = self.normalize_angle(angle_to_goal - self.thetar)

        # Control lineal proporcional
        linear_speed = 0.2 * rho

        # Control angular PID
        error = alpha
        self.integral_error += error * dt
        derivative = (error - self.prev_error) / dt
        angular_speed = 0.5 * (self.kp * error + self.ki * self.integral_error + self.kd * derivative)
        self.prev_error = error

        # Comando de velocidad
        cmd = Twist()
        cmd.linear.x = max(min(linear_speed, 0.5), -0.5)
        cmd.angular.z = max(min(angular_speed, 2.0), -2.0)
        self.cmd_vel_pub.publish(cmd)

        # Verificación de llegada
        if rho < 0.1:
            self.current_index += 1
            self.get_logger().info(f"Waypoint {self.current_index} alcanzado.")

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Todos los waypoints alcanzados.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
