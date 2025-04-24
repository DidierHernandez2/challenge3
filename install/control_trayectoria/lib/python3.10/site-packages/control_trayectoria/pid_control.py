import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from math import sin, cos, atan2, sqrt

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Parámetros físicos
        self.R = 0.05  # Radio de rueda en metros
        self.L = 0.18  # Distancia entre ruedas

        # Estados del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = self.get_clock().now()

        # Velocidades de ruedas
        self.wL = 0.0
        self.wR = 0.0

        # Waypoints para la trayectoria cuadrada
        self.waypoints = [(2, 0), (2, 2), (0, 2), (0, 0)]
        self.current_wp_index = 0

        # Controladores PID
        self.kp_lin = 1.0
        self.kp_ang = 4.0

        # Subscripciones
        self.create_subscription(Float32, '/VelocityEncL', self.left_callback, 10)
        self.create_subscription(Float32, '/VelocityEncR', self.right_callback, 10)

        # Publicador de cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

    def left_callback(self, msg):
        self.wL = msg.data

    def right_callback(self, msg):
        self.wR = msg.data

    def update_odometry(self, dt):
        vL = self.R * self.wL
        vR = self.R * self.wR

        v = (vR + vL) / 2
        w = (vR - vL) / self.L

        self.x += v * cos(self.theta) * dt
        self.y += v * sin(self.theta) * dt
        self.theta += w * dt

    def control_loop(self):
        # Calcular dt
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        # Actualizar odometría
        self.update_odometry(dt)

        # Obtener objetivo actual
        goal_x, goal_y = self.waypoints[self.current_wp_index]

        # Cálculo de errores
        dx = goal_x - self.x
        dy = goal_y - self.y
        rho = sqrt(dx**2 + dy**2)
        alpha = atan2(dy, dx) - self.theta

        # Normalizar ángulo
        alpha = atan2(sin(alpha), cos(alpha))

        # Condición de llegada
        if rho < 0.1:
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
            return

        # PID proporcional puro (puedes extender a PID completo si deseas)
        v = self.kp_lin * rho
        w = self.kp_ang * alpha

        # Saturación
        v = max(min(v, 0.5), -0.5)
        w = max(min(w, 2.0), -2.0)

        # Publicar comando
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
