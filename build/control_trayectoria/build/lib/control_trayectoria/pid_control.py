#!/usr/bin/env python3
# Archivo: pid_control.py (paquete: control_trayectoria)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class PIDControl(Node):
    def __init__(self):
        # Nodo nombrado según el archivo: pid_control\        
        super().__init__('pid_control')

        # Parámetros del robot
        self.r = 0.05    # radio de rueda (m)
        self.L = 0.18    # separación entre ruedas (m)

        # Velocidades deseadas (lectura de /cmd_vel)
        self.v_des = 0.0
        self.w_des = 0.0
        # Velocidades reales medidas por encoders
        self.v_r = 0.0
        self.v_l = 0.0

        # Ganancias P
        self.Kp_v = 1.0  # control lineal
        self.Kp_w = 1.0  # control angular

        # Suscribir a la orden deseada de velocidad
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            10
        )
        # Suscripciones a encoders /VelocityEncR y /VelocityEncL
        self.sub_encR = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.encR_callback,
            qos_profile_sensor_data
        )
        self.sub_encL = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.encL_callback,
            qos_profile_sensor_data
        )

        # Publicador para enviar velocidad corregida a /cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Bucle de control a 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Nodo pid_control.py iniciado en paquete control_trayectoria.')

    def cmd_vel_cb(self, msg: Twist):
        # Velocidad deseada (linear.x, angular.z)
        self.v_des = msg.linear.x
        self.w_des = msg.angular.z

    def encR_callback(self, msg: Float32):
        # ω_r → v_r
        self.v_r = msg.data * self.r

    def encL_callback(self, msg: Float32):
        # ω_l → v_l
        self.v_l = msg.data * self.r

    def control_loop(self):
        # Vel real
        V_real     = 0.5 * (self.v_r + self.v_l)
        Omega_real = (self.v_r - self.v_l) / self.L

        # Errores
        e_v = self.v_des - V_real
        e_w = self.w_des - Omega_real

        # Control P y saturación
        v_cmd = float(np.clip(self.Kp_v * e_v, -0.5, 0.5))
        w_cmd = float(np.clip(self.Kp_w * e_w, -1.0, 1.0))

        # Publicar
        cmd = Twist()
        cmd.linear.x  = v_cmd
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