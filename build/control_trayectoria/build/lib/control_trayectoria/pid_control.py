import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_control')

        # === Parámetros del robot ===
        self.r = 0.05      # Radio de las ruedas (m)
        self.l = 0.18      # Distancia entre ruedas (m)
        self.rate_hz = 100
        self.dt = 1.0 / self.rate_hz

        # === Estado del robot ===
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0
        self.v_r = 0.0
        self.v_l = 0.0

        # === Objetivo actual ===
        self.goals = [(2, 0), (2, 2), (0, 2), (0, 0)]
        self.current_goal_index = 0

        # === Errores anteriores ===
        self.e_dist_prev = 0.0
        self.e_ang_prev = 0.0
        self.sum_e_dist = 0.0
        self.sum_e_ang = 0.0

        # === PID gains ===
        self.Kp_dist = 1.5
        self.Ki_dist = 0.0
        self.Kd_dist = 0.1

        self.Kp_ang = 4.0
        self.Ki_ang = 0.0
        self.Kd_ang = 0.1

        # === Subscripciones ===
        self.sub_r = self.create_subscription(Float32, 'VelocityEncR', self.enc_r_cb, 10)
        self.sub_l = self.create_subscription(Float32, 'VelocityEncL', self.enc_l_cb, 10)

        # === Publicador de velocidades ===
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # === Timer principal ===
        self.timer = self.create_timer(self.dt, self.control_loop)

    def enc_r_cb(self, msg):
        self.v_r = self.r * msg.data

    def enc_l_cb(self, msg):
        self.v_l = self.r * msg.data

    def update_odometry(self):
        V = 0.5 * (self.v_r + self.v_l)
        Omega = (1.0 / self.l) * (self.v_r - self.v_l)

        self.X += V * np.cos(self.Th) * self.dt
        self.Y += V * np.sin(self.Th) * self.dt
        self.Th += Omega * self.dt

    def control_loop(self):
        self.update_odometry()

        goal_x, goal_y = self.goals[self.current_goal_index]

        dx = goal_x - self.X
        dy = goal_y - self.Y
        distance = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.Th)

        # Control de distancia
        self.sum_e_dist += distance * self.dt
        d_dist = (distance - self.e_dist_prev) / self.dt
        v = self.Kp_dist * distance + self.Ki_dist * self.sum_e_dist + self.Kd_dist * d_dist

        # Control de orientación
        self.sum_e_ang += angle_error * self.dt
        d_ang = (angle_error - self.e_ang_prev) / self.dt
        w = self.Kp_ang * angle_error + self.Ki_ang * self.sum_e_ang + self.Kd_ang * d_ang

        self.e_dist_prev = distance
        self.e_ang_prev = angle_error

        # Comprobación de llegada
        if distance < 0.05:
            self.get_logger().info(f"Goal {self.current_goal_index+1} reached!")
            self.current_goal_index += 1
            if self.current_goal_index >= len(self.goals):
                self.get_logger().info("Trajectory complete.")
                v = 0.0
                w = 0.0
                self.current_goal_index = 0  # O quitar si solo quieres hacer una vuelta

        # Publicar velocidades
        cmd = Twist()
        cmd.linear.x = float(np.clip(v, -0.4, 0.4))       # Saturación de seguridad
        cmd.angular.z = float(np.clip(w, -2.0, 2.0))
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
