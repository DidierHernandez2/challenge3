#!/usr/bin/env python3
# Archivo: pid_control.py (paquete: control_trayectoria)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# Normaliza un ángulo entre -pi y pi
def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class PIDControl(Node):
    def __init__(self):
        super().__init__('pid_control')  # Nodo en paquete control_trayectoria, archivo pid_control.py

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

        # Ganancias proporcionales para control P de pose
        self.Kp_rho = 0.8   # ganancia distancia
        self.Kp_alpha = 4.0 # ganancia orientación

        # Waypoints de un cuadrado de lado 2 m
        self.waypoints = [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        self.current_wp = 0

        # Tolerancia y periodo de control
        self.dist_tol = 0.05  # metros
        self.dt = 0.1         # periodo de control: 10 Hz (se reduce carga)

        # Suscripciones a encoders (lecturas asíncronas)
        self.sub_encR = self.create_subscription(
            Float32, 'VelocityEncR', self.encR_callback, qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(
            Float32, 'VelocityEncL', self.encL_callback, qos_profile_sensor_data)

        # Publicador de comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer de control a 10 Hz
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('PIDControl iniciado con control P de pose. Timer a 10 Hz para reducir carga.')

    def encR_callback(self, msg: Float32):
        # Convierte velocidad angular (rad/s) a velocidad lineal de rueda (m/s)
        self.v_r = msg.data * self.r

    def encL_callback(self, msg: Float32):
        self.v_l = msg.data * self.r

    def control_loop(self):
        # Calcula velocidades reales del robot
        V_real = 0.5 * (self.v_r + self.v_l)
        Omega_real = (self.v_r - self.v_l) / self.L

        # Integra la pose internamente
        self.x += V_real * np.cos(self.th) * self.dt
        self.y += V_real * np.sin(self.th) * self.dt
        self.th = normalize_angle(self.th + Omega_real * self.dt)

        # Obtiene el waypoint actual
        gx, gy = self.waypoints[self.current_wp]
        dx = gx - self.x
        dy = gy - self.y
        rho = np.hypot(dx, dy)
        alpha = normalize_angle(np.arctan2(dy, dx) - self.th)

        # Verifica si alcanzó el waypoint
        if rho < self.dist_tol:
            self.get_logger().info(f"Waypoint {self.current_wp} alcanzado en x={self.x:.2f}, y={self.y:.2f}")
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)
            rho = 0.0
            alpha = 0.0

        # Control P para velocidad de avance y giro
        v_cmd = float(np.clip(self.Kp_rho * rho, -0.5, 0.5))
        w_cmd = float(np.clip(self.Kp_alpha * alpha, -1.0, 1.0))

        # Publica en /cmd_vel
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
