import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
import traceback

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_control_node')

        # Parámetros del robot
        self.R = 0.05   # radio de rueda (m)
        self.L = 0.18   # distancia entre ruedas (m)

        # Estados de pose
        self.X = self.Y = self.Th = 0.0

        # Lecturas de encoder (ω en rad/s)
        self.wr_rad = None
        self.wl_rad = None

        # PID externo: posición → v_ref, w_ref
        self.Kp_dist, self.Ki_dist, self.Kd_dist = 1.5, 0.0, 0.1
        self.Kp_ang,  self.Ki_ang,  self.Kd_ang  = 4.0, 0.0, 0.1
        self.e_dist_prev = self.sum_e_dist = 0.0
        self.e_ang_prev  = self.sum_e_ang  = 0.0

        # PID interno: modula v_real→v_cmd, Ω_real→w_cmd
        self.Kp_v, self.Ki_v, self.Kd_v = 0.8, 0.0, 0.05
        self.Kp_w, self.Ki_w, self.Kd_w = 1.0, 0.0, 0.02
        self.e_v_prev = self.sum_e_v = 0.0
        self.e_w_prev = self.sum_e_w = 0.0

        # Trayectoria: cuadrado 2×2
        self.goals = [(2.0,0.0),(2.0,2.0),(0.0,2.0),(0.0,0.0)]
        self.goal_idx = 0

        # QoS: lecturas fiables, comandos best-effort (menos overhead)
        self.create_subscription(Float32, 'VelocityEncR',
                                 self.cb_enc_r, qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL',
                                 self.cb_enc_l, qos_profile_sensor_data)

        cmd_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)

        # Control loop a 10 Hz (mucho más ligero)
        self.loop_hz = 10.0
        self.dt = 1.0 / self.loop_hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("PID controller iniciado a 10 Hz con QoS BE.")

    def cb_enc_r(self, msg: Float32):
        self.wr_rad = msg.data

    def cb_enc_l(self, msg: Float32):
        self.wl_rad = msg.data

    def normalize(self, ang):
        return np.arctan2(np.sin(ang), np.cos(ang))

    def control_loop(self):
        try:
            # 1) Esperar primeras lecturas
            if self.wr_rad is None or self.wl_rad is None:
                return

            # 2) ω → velocidades lineales
            v_r = self.R * self.wr_rad
            v_l = self.R * self.wl_rad

            # 3) Vel real
            V_real     = 0.5 * (v_r + v_l)
            Omega_real = (v_r - v_l) / self.L

            # 4) PID externo para posición/orientación
            gx, gy = self.goals[self.goal_idx]
            dx, dy = gx - self.X, gy - self.Y
            dist = np.hypot(dx, dy)
            ang_to_goal = np.arctan2(dy, dx)
            e_ang = self.normalize(ang_to_goal - self.Th)

            # Distancia
            self.sum_e_dist += dist * self.dt
            d_dist = (dist - self.e_dist_prev) / self.dt
            v_ref = (self.Kp_dist*dist +
                     self.Ki_dist*self.sum_e_dist +
                     self.Kd_dist*d_dist)
            self.e_dist_prev = dist

            # Angular
            self.sum_e_ang += e_ang * self.dt
            d_ang = (e_ang - self.e_ang_prev) / self.dt
            w_ref = (self.Kp_ang*e_ang +
                     self.Ki_ang*self.sum_e_ang +
                     self.Kd_ang*d_ang)
            self.e_ang_prev = e_ang

            # Meta alcanzada
            if dist < 0.05:
                self.get_logger().info(f"Meta {self.goal_idx+1} alcanzada")
                self.goal_idx = (self.goal_idx + 1) % len(self.goals)
                self.sum_e_dist = self.sum_e_ang = 0.0

            # Saturar referencias
            v_ref = float(np.clip(v_ref, -0.3, 0.3))
            w_ref = float(np.clip(w_ref, -1.5, 1.5))

            # 5) PID interno para velocidad
            e_v = v_ref - V_real
            self.sum_e_v += e_v * self.dt
            d_v = (e_v - self.e_v_prev) / self.dt
            u_v = (self.Kp_v*e_v +
                   self.Ki_v*self.sum_e_v +
                   self.Kd_v*d_v)
            self.e_v_prev = e_v

            e_w = w_ref - Omega_real
            self.sum_e_w += e_w * self.dt
            d_w = (e_w - self.e_w_prev) / self.dt
            u_w = (self.Kp_w*e_w +
                   self.Ki_w*self.sum_e_w +
                   self.Kd_w*d_w)
            self.e_w_prev = e_w

            v_cmd = float(np.clip(u_v, -0.3, 0.3))
            w_cmd = float(np.clip(u_w, -1.5, 1.5))

            # 6) Actualizar odometría interna
            self.X  += V_real * np.cos(self.Th) * self.dt
            self.Y  += V_real * np.sin(self.Th) * self.dt
            self.Th += Omega_real * self.dt

            # 7) Publicar SOLO SI HAY CAMBIO SIGNIFICATIVO
            if abs(v_cmd) > 0.01 or abs(w_cmd) > 0.01:
                cmd = Twist()
                cmd.linear.x  = v_cmd
                cmd.angular.z = w_cmd
                self.cmd_pub.publish(cmd)

        except Exception:
            self.get_logger().error("Error en control_loop:\n" +
                                    traceback.format_exc())

def main():
    rclpy.init()
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
