import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# Normalize angle between -pi and pi
def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class PIDControl(Node):
    def __init__(self):
        super().__init__('pid_control')  # Nodo y archivo pid_control.py en paquete control_trayectoria

        # Parámetros cinemáticos del robot
        self.r = 0.05  # radio de rueda (m)
        self.L = 0.18  # distancia entre ruedas (m)

        # Estado interno de la pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Velocidades medidas por encoders
        self.v_r = 0.0
        self.v_l = 0.0

        # Ganancias proporcionales
        self.Kp_rho = 0.8   # ganancia para distancia
        self.Kp_alpha = 4.0 # ganancia para orientación

        # Integral de errores (para PI si lo extiendes)
        self.sum_rho = 0.0
        self.sum_alpha = 0.0

        # Waypoints de un cuadrado de lado 2 m
        self.waypoints = [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        self.current_wp = 0

        # Tolerancias y temporización
        self.dist_tol = 0.05  # metros
        self.dt = 0.05        # periodo de control (20 Hz)

        # Suscripciones a encoders
        self.sub_encR = self.create_subscription(
            Float32, 'VelocityEncR', self.encR_callback, qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(
            Float32, 'VelocityEncL', self.encL_callback, qos_profile_sensor_data)

        # Publicador de comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer de control
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('PIDControl iniciado: control P de pose con square trajectory.')

    def encR_callback(self, msg: Float32):
        # Convierte velocidad angular (rad/s) a velocidad lineal en rueda (m/s)
        self.v_r = msg.data * self.r

    def encL_callback(self, msg: Float32):
        self.v_l = msg.data * self.r

    def control_loop(self):
        # Velocidades reales del robot
        V_real = 0.5 * (self.v_r + self.v_l)
        Omega_real = (self.v_r - self.v_l) / self.L

        # Integración de la pose
        self.x += V_real * np.cos(self.th) * self.dt
        self.y += V_real * np.sin(self.th) * self.dt
        self.th = normalize_angle(self.th + Omega_real * self.dt)

        # Cálculo de errores hacia waypoint actual
        gx, gy = self.waypoints[self.current_wp]
        dx = gx - self.x
        dy = gy - self.y
        rho = np.hypot(dx, dy)
        alpha = normalize_angle(np.arctan2(dy, dx) - self.th)

        # Avance al siguiente waypoint si se cumple tolerancia
        if rho < self.dist_tol:
            self.get_logger().info(f"Waypoint {self.current_wp} alcanzado: ({self.x:.2f}, {self.y:.2f})")
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)
            rho = 0.0
            alpha = 0.0
            self.sum_rho = 0.0
            self.sum_alpha = 0.0

        # Velocidades de referencia (P) para pose
        # v_ref = Kp_rho * rho
        # w_ref = Kp_alpha * alpha
        v_cmd = float(np.clip(self.Kp_rho * rho, -0.5, 0.5))
        w_cmd = float(np.clip(self.Kp_alpha * alpha, -1.0, 1.0))

        # Publicar comando
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PIDControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()