import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import traceback

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_control_node')

        # Parámetros del robot
        self.R = 0.05   # radio de rueda (m)
        self.L = 0.18   # distancia entre ruedas (m)

        # Estados de pose
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Lecturas de encoder (ω en rad/s)
        self.wr_rad = None
        self.wl_rad = None

        # PID externo: genera v_ref y w_ref según la meta
        self.Kp_dist, self.Ki_dist, self.Kd_dist = 1.5, 0.0, 0.1
        self.Kp_ang,  self.Ki_ang,  self.Kd_ang  = 4.0, 0.0, 0.1
        self.e_dist_prev = 0.0
        self.sum_e_dist = 0.0
        self.e_ang_prev  = 0.0
        self.sum_e_ang   = 0.0

        # PID interno: regula velocidades
        self.Kp_v, self.Ki_v, self.Kd_v = 0.8, 0.0, 0.05
        self.Kp_w, self.Ki_w, self.Kd_w = 1.0, 0.0, 0.02
        self.e_v_prev = 0.0
        self.sum_e_v  = 0.0
        self.e_w_prev = 0.0
        self.sum_e_w  = 0.0

        # Trayectoria: cuadrado 2×2
        self.goals = [(2.0,0.0),(2.0,2.0),(0.0,2.0),(0.0,0.0)]
        self.goal_idx = 0

        # Suscripciones a encoders (rad/s)
        self.create_subscription(Float32, 'VelocityEncR',
                                 self.cb_enc_r, qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL',
                                 self.cb_enc_l, qos_profile_sensor_data)

        # Publicador de cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Bucle de control a 20 Hz
        self.loop_hz = 20.0
        self.dt = 1.0 / self.loop_hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("PID controller con cascada de velocidad iniciado.")

    def cb_enc_r(self, msg: Float32):
        self.wr_rad = msg.data

    def cb_enc_l(self, msg: Float32):
        self.wl_rad = msg.data

    def normalize(self, ang):
        return np.arctan2(np.sin(ang), np.cos(ang))

    def control_loop(self):
        try:
            # esperar a la primera lectura
            if self.wr_rad is None or self.wl_rad is None:
                return

            # --- 1) convertir ω→vel lineal de rueda ---
            v_r = self.R * self.wr_rad
            v_l = self.R * self.wl_rad

            # --- 2) calcular vel real del robot ---
            V_real     = 0.5 * (v_r + v_l)
            Omega_real = (v_r - v_l) / self.L

            # --- 3) PID externo: posición y orientación → vel deseadas ---
            gx, gy = self.goals[self.goal_idx]
            dx = gx - self.X; dy = gy - self.Y
            dist = np.hypot(dx, dy)
            ang_to_goal = np.arctan2(dy, dx)
            e_ang = self.normalize(ang_to_goal - self.Th)

            # PID distancia
            self.sum_e_dist += dist * self.dt
            d_dist = (dist - self.e_dist_prev) / self.dt
            v_ref = (self.Kp_dist*dist +
                     self.Ki_dist*self.sum_e_dist +
                     self.Kd_dist*d_dist)
            self.e_dist_prev = dist

            # PID orientación
            self.sum_e_ang += e_ang * self.dt
            d_ang = (e_ang - self.e_ang_prev) / self.dt
            w_ref = (self.Kp_ang*e_ang +
                     self.Ki_ang*self.sum_e_ang +
                     self.Kd_ang*d_ang)
            self.e_ang_prev = e_ang

            # meta alcanzada?
            if dist < 0.05:
                self.get_logger().info(f"Meta {self.goal_idx+1} alcanzada.")
                self.goal_idx = (self.goal_idx + 1) % len(self.goals)
                self.sum_e_dist = 0.0
                self.sum_e_ang  = 0.0

            # saturar referencias
            v_ref = float(np.clip(v_ref, -0.4, 0.4))
            w_ref = float(np.clip(w_ref, -2.0, 2.0))

            # --- 4) PID interno: vel deseada vs real → comando final ---
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

            # saturar comando
            v_cmd = float(np.clip(u_v, -0.4, 0.4))
            w_cmd = float(np.clip(u_w, -2.0, 2.0))

            # --- 5) actualizar odometría interna de pose ---
            self.X  += V_real * np.cos(self.Th) * self.dt
            self.Y  += V_real * np.sin(self.Th) * self.dt
            self.Th += Omega_real * self.dt

            # --- 6) publicar comando ---
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
