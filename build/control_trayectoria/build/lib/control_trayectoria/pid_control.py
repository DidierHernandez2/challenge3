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
        self.r = 0.05   # Radio de rueda (m)
        self.l = 0.18   # Distancia entre ruedas (m)

        # Estados del robot
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Encoder velocities (m/s), inicializamos a None hasta recibir la 1ª lectura
        self.v_r = None
        self.v_l = None

        # Waypoints de la trayectoria (cuadrado 2×2)
        self.goals = [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        self.goal_idx = 0

        # Errores PID
        self.e_dist_prev = 0.0
        self.sum_e_dist = 0.0
        self.e_ang_prev = 0.0
        self.sum_e_ang = 0.0

        # Ganancias PID
        self.Kp_dist, self.Ki_dist, self.Kd_dist = 1.5, 0.0, 0.1
        self.Kp_ang,  self.Ki_ang,  self.Kd_ang  = 4.0, 0.0, 0.1

        # QoS y subscripciones
        self.create_subscription(Float32, 'VelocityEncR', self.cb_enc_r, qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL', self.cb_enc_l, qos_profile_sensor_data)

        # Publicador de cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control loop a 20 Hz (más suave para Jetson)
        self.timer = self.create_timer(1.0/20.0, self.control_loop)

        self.get_logger().info("PID controller started.")

    def cb_enc_r(self, msg: Float32):
        self.v_r = self.r * msg.data

    def cb_enc_l(self, msg: Float32):
        self.v_l = self.r * msg.data

    def normalize_angle(self, a):
        return np.arctan2(np.sin(a), np.cos(a))

    def control_loop(self):
        try:
            # Esperar a la 1ª lectura de encoders
            if self.v_r is None or self.v_l is None:
                self.get_logger().warn("Esperando encoders…")
                return

            # Calcular dt real
            now = self.get_clock().now()
            if not hasattr(self, 'last_time'):
                self.last_time = now
                return
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now
            if dt <= 0.0:
                return

            # --- 1) Actualizar odometría ---
            V = 0.5 * (self.v_r + self.v_l)
            Omega = (1.0 / self.l) * (self.v_r - self.v_l)
            self.X  += V * np.cos(self.Th) * dt
            self.Y  += V * np.sin(self.Th) * dt
            self.Th += Omega * dt

            # --- 2) Cálculo de error frente al objetivo actual ---
            gx, gy = self.goals[self.goal_idx]
            dx = gx - self.X
            dy = gy - self.Y
            dist = np.hypot(dx, dy)
            ang  = np.arctan2(dy, dx)
            e_ang = self.normalize_angle(ang - self.Th)

            # --- 3) PID distancia ---
            self.sum_e_dist += dist * dt
            d_dist = (dist - self.e_dist_prev) / dt
            u_dist = self.Kp_dist*dist + self.Ki_dist*self.sum_e_dist + self.Kd_dist*d_dist
            self.e_dist_prev = dist

            # --- 4) PID orientación ---
            self.sum_e_ang += e_ang * dt
            d_ang = (e_ang - self.e_ang_prev) / dt
            u_ang = self.Kp_ang*e_ang + self.Ki_ang*self.sum_e_ang + self.Kd_ang*d_ang
            self.e_ang_prev = e_ang

            # --- 5) Comprobación de llegada ---
            if dist < 0.05:
                self.get_logger().info(f"Goal {self.goal_idx+1} reached: ({gx:.2f},{gy:.2f})")
                self.goal_idx = (self.goal_idx+1) % len(self.goals)
                # reset integrales
                self.sum_e_dist = 0.0
                self.sum_e_ang  = 0.0

            # --- 6) Saturación ---
            v = float(np.clip(u_dist, -0.4, 0.4))
            w = float(np.clip(u_ang , -2.0, 2.0))

            # --- 7) Publicar cmd_vel ---
            cmd = Twist()
            cmd.linear.x  = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            # --- 8) Log de depuración cada 1s ---
            if hasattr(self, '_log_timer'):
                if (now - self._log_timer).nanoseconds * 1e-9 > 1.0:
                    self._log_timer = now
                    self.get_logger().info(
                        f"Pose: x={self.X:.2f}, y={self.Y:.2f}, θ={self.Th:.2f} | "
                        f"e_dist={dist:.2f}, e_ang={e_ang:.2f} | "
                        f"cmd: v={v:.2f}, w={w:.2f}")
            else:
                self._log_timer = now

        except Exception:
            self.get_logger().error("Error en control_loop:\n" + traceback.format_exc())

def main():
    rclpy.init()
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, cerrando...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
