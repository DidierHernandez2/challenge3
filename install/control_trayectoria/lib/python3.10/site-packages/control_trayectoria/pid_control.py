#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# Helper to normalize angles between -pi and pi
def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class SquarePController(Node):
    def __init__(self):
        super().__init__('pid_control')  # paquete control_trayectoria, archivo pid_control.py

        # Parámetros del robot
        self.r = 0.05   # radio de rueda (m)
        self.L = 0.18   # separación entre ruedas (m)

        # Pose estimada
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Velocidades medidas por encoders
        self.v_r = 0.0
        self.v_l = 0.0

        # Ganancias P
        self.Kp_v = 1.0
        self.Kp_w = 1.0

        # Estado del square: segmento y fase
        self.segment = 0        # 0..3
        self.phase = 'forward'  # 'forward' o 'turn'
        # Referencias iniciales para cada fase
        self.X0 = 0.0
        self.Y0 = 0.0
        self.Th0 = 0.0

        # Parámetros de movimiento
        self.dist_target = 2.0      # metros a avanzar
        self.angle_target = np.pi/2 # radianes a girar
        # Velocidades deseadas (puedes ajustar más rápido)
        self.v_des = 0.4   # m/s
        self.w_des = 1.0   # rad/s

        # Suscripciones a encoders
        self.sub_encR = self.create_subscription(
            Float32, 'VelocityEncR', self.encR_callback, qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(
            Float32, 'VelocityEncL', self.encL_callback, qos_profile_sensor_data)

        # Publicador de cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer de control a 20 Hz
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('SquarePController reiniciado: cuadrado hardcodeado sin waypoints.')

    def encR_callback(self, msg: Float32):
        self.v_r = msg.data * self.r

    def encL_callback(self, msg: Float32):
        self.v_l = msg.data * self.r

    def control_loop(self):
        # Calcular velocidades reales
        V_real     = 0.5 * (self.v_r + self.v_l)
        Omega_real = (self.v_r - self.v_l) / self.L

        # Integrar pose
        self.X  += V_real * np.cos(self.Th) * self.dt
        self.Y  += V_real * np.sin(self.Th) * self.dt
        self.Th = normalize_angle(self.Th + Omega_real * self.dt)

        # Determinar avance o giro basados en fase
        # Calcular progresos
        dx = self.X - self.X0
        dy = self.Y - self.Y0
        dist = np.hypot(dx, dy)
        dTh = normalize_angle(self.Th - self.Th0)

        if self.phase == 'forward':
            if dist >= self.dist_target:
                # Inicia la fase de giro
                self.phase = 'turn'
                self.Th0 = self.Th
                self.get_logger().info(f"Segmento {self.segment+1}: avance completado")
        else:  # 'turn'
            if abs(dTh) >= self.angle_target:
                # Finaliza giro
                self.phase = 'forward'
                self.segment = (self.segment + 1) % 4
                self.X0 = self.X
                self.Y0 = self.Y
                self.get_logger().info(f"Segmento {self.segment}: giro completado")

        # Velocidad de referencia según fase
        if self.phase == 'forward':
            v_ref = self.v_des
            w_ref = 0.0
        else:
            v_ref = 0.0
            w_ref = self.w_des * np.sign(dTh)

        # Control P
        e_v = v_ref - V_real
        e_w = w_ref - Omega_real
        v_cmd = float(np.clip(self.Kp_v * e_v, -self.v_des, self.v_des))
        w_cmd = float(np.clip(self.Kp_w * e_w, -self.w_des, self.w_des))

        # Publicar cmd_vel
        cmd = Twist()
        cmd.linear.x  = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

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