#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# Normalize angle between -pi and pi
def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class SquarePIDController(Node):
    def __init__(self):
        super().__init__('pid_control')  # paquete control_trayectoria, archivo pid_control.py

        # Robot parameters
        self.r = 0.05   # wheel radius (m)
        self.L = 0.18   # wheel separation (m)

        # Estimated pose
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Encoder-measured velocities
        self.v_r = 0.0
        self.v_l = 0.0

        # PID gains for linear velocity
        self.Kp_v = 1.0
        self.Ki_v = 0.0
        self.Kd_v = 0.1
        # PID state for linear
        self.sum_e_v = 0.0
        self.e_v_prev = 0.0

        # PID gains for angular velocity
        self.Kp_w = 1.0
        self.Ki_w = 0.0
        self.Kd_w = 0.1
        # PID state for angular
        self.sum_e_w = 0.0
        self.e_w_prev = 0.0

        # Square motion state
        self.segment = 0        # 0..3
        self.phase = 'forward'  # 'forward' or 'turn'
        self.X0 = 0.0          # reference pose for segment
        self.Y0 = 0.0
        self.Th0 = 0.0

        # Motion parameters
        self.dist_target = 2.0      # meters forward
        self.angle_target = np.pi/2 # radians turn
        # Desired speeds
        self.v_des = 0.4  # m/s
        self.w_des = 0.5  # rad/s (reduced)

        # Subscriptions to encoders
        self.sub_encR = self.create_subscription(
            Float32, 'VelocityEncR', self.encR_callback, qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(
            Float32, 'VelocityEncL', self.encL_callback, qos_profile_sensor_data)

        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control loop timer at 20 Hz
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('SquarePIDController iniciado: cuadrado con PID completo.')

    def encR_callback(self, msg: Float32):
        self.v_r = msg.data * self.r

    def encL_callback(self, msg: Float32):
        self.v_l = msg.data * self.r

    def control_loop(self):
        # Compute real velocities
        V_real     = 0.5 * (self.v_r + self.v_l)
        Omega_real = (self.v_r - self.v_l) / self.L

        # Integrate pose
        self.X  += V_real * np.cos(self.Th) * self.dt
        self.Y  += V_real * np.sin(self.Th) * self.dt
        self.Th = normalize_angle(self.Th + Omega_real * self.dt)

        # Progress metrics
        dx = self.X - self.X0
        dy = self.Y - self.Y0
        dist = np.hypot(dx, dy)
        dTh = normalize_angle(self.Th - self.Th0)

        # Phase switching
        if self.phase == 'forward':
            if dist >= self.dist_target:
                self.phase = 'turn'
                self.Th0 = self.Th
                self.get_logger().info(f"Segmento {self.segment+1}: avance completo")
        else:  # 'turn'
            if abs(dTh) >= self.angle_target:
                self.phase = 'forward'
                self.segment = (self.segment + 1) % 4
                self.X0 = self.X
                self.Y0 = self.Y
                self.get_logger().info(f"Segmento {self.segment}: giro completo")

        # Desired reference speeds
        if self.phase == 'forward':
            v_ref = self.v_des
            w_ref = 0.0
        else:
            v_ref = 0.0
            w_ref = self.w_des * np.sign(dTh)

        # PID for linear velocity
        e_v = v_ref - V_real
        self.sum_e_v += e_v * self.dt
        d_e_v = (e_v - self.e_v_prev) / self.dt
        v_cmd = (self.Kp_v * e_v + self.Ki_v * self.sum_e_v + self.Kd_v * d_e_v)
        self.e_v_prev = e_v

        # PID for angular velocity
        e_w = w_ref - Omega_real
        self.sum_e_w += e_w * self.dt
        d_e_w = (e_w - self.e_w_prev) / self.dt
        w_cmd = (self.Kp_w * e_w + self.Ki_w * self.sum_e_w + self.Kd_w * d_e_w)
        self.e_w_prev = e_w

        # Saturation
        v_cmd = float(np.clip(v_cmd, -self.v_des, self.v_des))
        w_cmd = float(np.clip(w_cmd, -self.w_des, self.w_des))

        # Publish cmd_vel
        cmd = Twist()
        cmd.linear.x  = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = SquarePIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
