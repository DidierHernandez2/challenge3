#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class WaypointPController(Node):
    def __init__(self):
        super().__init__('pid_control')  # paquete control_trayectoria, archivo pid_control.py

        # Parámetros del robot
        self.r = 0.05   # radio de rueda (m)
        self.L = 0.18   # distancia entre ruedas (m)

        # Estado de pose estimada
        self.X = 0.0; self.Y = 0.0; self.Th = 0.0

        # Velocidades medidas por encoders
        self.v_r = 0.0; self.v_l = 0.0

        # Ganancias P
        self.Kp_v = 1.0   # control lineal
        self.Kp_w = 1.0   # control angular

        # Waypoints del cuadrado (en metros)
        self.waypoints = [(2.0,0.0), (2.0,2.0), (0.0,2.0), (0.0,0.0)]
        self.current_wp = 0

        # Fase de movimiento: 'rotate' o 'forward'
        self.phase = 'rotate'
        # Umbrales
        self.dist_tol  = 0.05   # m
        self.angle_tol = 0.05   # rad

        # Velocidades deseadas (más rápidas)
        self.v_des = 0.4   # m/s
        self.w_des = 1.0   # rad/s

        # Subscripciones a encoders
        self.sub_encR = self.create_subscription(
            Float32, 'VelocityEncR', self.encR_callback, qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(
            Float32, 'VelocityEncL', self.encL_callback, qos_profile_sensor_data)

        # Publisher de comando corregido
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Loop de control a 20 Hz
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('WaypointPController iniciado: moviendo por waypoints.')

    def encR_callback(self, msg: Float32):
        self.v_r = msg.data * self.r
    def encL_callback(self, msg: Float32):
        self.v_l = msg.data * self.r

    def control_loop(self):
        # Calcular velocidades reales
        V_real     = 0.5 * (self.v_r + self.v_l)
        Omega_real = (self.v_r - self.v_l) / self.L

        # Integrar pose
        self.X  += V_real * self.dt * np.cos(self.Th)
        self.Y  += V_real * self.dt * np.sin(self.Th)
        self.Th = normalize_angle(self.Th + Omega_real * self.dt)

        # Objetivo actual
        x_goal, y_goal = self.waypoints[self.current_wp]
        dx = x_goal - self.X; dy = y_goal - self.Y
        dist_error = np.hypot(dx, dy)
        angle_goal = np.arctan2(dy, dx)
        angle_error = normalize_angle(angle_goal - self.Th)

        # Lógica de fases
        if self.phase == 'rotate':
            # girar primero
            if abs(angle_error) < self.angle_tol:
                self.phase = 'forward'
                self.get_logger().info(f"Waypoint {self.current_wp}: rotación completada")
        else:
            # avanzar
            if dist_error < self.dist_tol:
                self.get_logger().info(f"Waypoint {self.current_wp} alcanzado")
                # siguiente waypoint y volver a rotar
                self.current_wp = (self.current_wp + 1) % len(self.waypoints)
                self.phase = 'rotate'

        # Definir velocidades de referencia según fase
        if self.phase == 'rotate':
            v_ref = 0.0
            w_ref = self.w_des * np.sign(angle_error)
        else:
            v_ref = self.v_des
            w_ref = 0.0

        # Control P
        e_v = v_ref - V_real
        e_w = w_ref - Omega_real
        v_cmd = float(np.clip(self.Kp_v * e_v, -self.v_des, self.v_des))
        w_cmd = float(np.clip(self.Kp_w * e_w, -self.w_des, self.w_des))

        # Publicar comando
        cmd = Twist()
        cmd.linear.x  = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = WaypointPController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()