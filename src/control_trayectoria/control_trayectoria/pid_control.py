import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def compute(self, setpoint, actual):
        current_time = time.time()
        dt = current_time - self.prev_time
        error = setpoint - actual
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        self.prev_time = current_time
        return output


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('pid_trajectory_controller')

        # Parámetros físicos del robot
        self.wheel_radius = 0.033  # metros
        self.base_width = 0.16     # distancia entre ruedas

        # Subscripciones a encoders
        self.sub_enc_l = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.encoder_left_callback,
            10
        )
        self.sub_enc_r = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.encoder_right_callback,
            10
        )

        # Publicador del comando de velocidad
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Inicializar PID para cada rueda
        self.pid_left = PIDController(1.0, 0.0, 0.1)
        self.pid_right = PIDController(1.0, 0.0, 0.1)

        self.encoder_l = 0.0
        self.encoder_r = 0.0

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.timer = self.create_timer(0.1, self.control_loop)

    def encoder_left_callback(self, msg):
        self.encoder_l = msg.data

    def encoder_right_callback(self, msg):
        self.encoder_r = msg.data

    def control_loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.start_time

        # Trayectoria cuadrada: 2m por lado
        # Aproximamos: 10s recto (0.2 m/s), 5s giro en el lugar (pi/2 rad)
        fase = (elapsed // 15) % 2  # 0 = avance, 1 = giro

        if fase == 0:
            v_d = 0.2  # m/s
            w_d = 0.0  # rad/s
        else:
            v_d = 0.0
            w_d = math.pi / 5  # giro 90° en 5 segundos

        # Calcular velocidades deseadas para cada rueda
        v_l_d = v_d - (w_d * self.base_width / 2)
        v_r_d = v_d + (w_d * self.base_width / 2)

        # Control por rueda
        v_l_real = self.encoder_l
        v_r_real = self.encoder_r

        vl_cmd = self.pid_left.compute(v_l_d, v_l_real)
        vr_cmd = self.pid_right.compute(v_r_d, v_r_real)

        # Convertir a Twist (inversa de la cinemática diferencial)
        linear = (vl_cmd + vr_cmd) / 2
        angular = (vr_cmd - vl_cmd) / self.base_width

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f"Fase: {fase} | v_d: {v_d:.2f}, w_d: {w_d:.2f} | v_l_real: {v_l_real:.2f}, v_r_real: {v_r_real:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
