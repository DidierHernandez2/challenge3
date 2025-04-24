#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class SquarePController(Node):
    def __init__(self):
        super().__init__('pid_control')  # paquete control_trayectoria, archivo pid_control.py

        # Robot params
        self.r = 0.05  # radio rueda (m)
        self.L = 0.18  # distancia entre ruedas (m)

        # Pose estimada
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Velocidades medidas por encoders
        self.v_r = 0.0
        self.v_l = 0.0

        # Ganancias P
        self.Kp_v = 1.0  # control lineal
        self.Kp_w = 1.0  # control angular

        # Estados del square
        self.segment = 0          # de 0 a 3
        self.phase = 'forward'    # 'forward' o 'turn'
        # Referencias de inicio
        self.X0 = 0.0
        self.Y0 = 0.0
        self.Th0 = 0.0

        # Parámetros de movimiento
        self.dist_target = 2.0      # metros
        self.angle_target = np.pi/2 # rad
        self.v_des = 0.2            # m/s base
        self.w_des = 0.5            # rad/s base

        # Subscripciones a encoders
        self.sub_encR = self.create_subscription(
            Float32, 'VelocityEncR', self.encR_callback, qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(
            Float32, 'VelocityEncL', self.encL_callback, qos_profile_sensor_data)

        # Publisher cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control loop a 20 Hz
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('SquarePController iniciado: 4 segmentos de 2m + 90°')

    def encR_callback(self, msg: Float32):
        self.v_r = msg.data * self.r

    def encL_callback(self, msg: Float32):
        self.v_l = msg.data * self.r

    def control_loop(self):
        # Calcular velocidades reales
        V_real     = 0.5 * (self.v_r + self.v_l)
        Omega_real = (self.v_r - self.v_l) / self.L

        # Integración de pose
        self.X  += V_real * self.dt * np.cos(self.Th)
        self.Y  += V_real * self.dt * np.sin(self.Th)
        self.Th = normalize_angle(self.Th + Omega_real * self.dt)

        # Distancia y ángulo desde el inicio de fase
        dx = self.X - self.X0
        dy = self.Y - self.Y0
        dist = np.hypot(dx, dy)
        dTh = normalize_angle(self.Th - self.Th0)

        # Selección de fase
        if self.phase == 'forward':
            # Si recorrimos la distancia target
            if dist >= self.dist_target:
                # iniciar giro
                self.phase = 'turn'
                self.Th0 = self.Th
                self.get_logger().info(f"Segmento {self.segment+1}: forward completo")
        else:  # phase == 'turn'
            if abs(dTh) >= self.angle_target:
                # fin giro
                self.phase = 'forward'
                self.segment = (self.segment + 1) % 4
                self.X0 = self.X
                self.Y0 = self.Y
                self.get_logger().info(f"Segmento {self.segment}: turn completo")

        # Definir v_des y w_des según fase
        if self.phase == 'forward':
            v_ref = self.v_des
            w_ref = 0.0
        else:
            v_ref = 0.0
            w_ref = self.w_des

        # P-control de velocidad
        e_v = v_ref - V_real
        e_w = w_ref - Omega_real
        v_cmd = float(np.clip(self.Kp_v * e_v, -0.5, 0.5))
        w_cmd = float(np.clip(self.Kp_w * e_w, -1.0, 1.0))

        # Publicar cmd_vel
        cmd = Twist()
        cmd.linear.x  = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

    # No necesita main aparte; al paquete lo invocamos con nodename pid_control

def main():
    rclpy.init()
    node = SquarePController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
