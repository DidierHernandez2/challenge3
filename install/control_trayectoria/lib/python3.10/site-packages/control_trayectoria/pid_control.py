import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PIDSquareController(Node):
    def __init__(self) -> None:
        super().__init__('pid_square_controller')

        # ——— PID gains ———
        self.kp_ang, self.ki_ang, self.kd_ang = 2.0, 0.0, 0.10
        self.kp_lin, self.ki_lin, self.kd_lin = 0.8, 0.0, 0.05

        # ——— Robot state ———
        self.xr = self.yr = self.thetar = 0.0

        # ——— PID memory ———
        self.integral_ang = self.integral_lin = 0.0
        self.prev_error_ang = self.prev_error_lin = 0.0

        # ——— Trajectory: square 2 m × 2 m ———
        self.waypoints = [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        self.current_index = 0

        # ——— ROS I/O ———
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Control period
        self.dt = 0.1
        self.create_timer(self.dt, self.control_loop)

    # ----------------- Callbacks -----------------
    def odom_callback(self, msg: Odometry) -> None:
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.thetar = self.quat_to_yaw(msg.pose.pose.orientation)

    # ----------------- Helpers -----------------
    @staticmethod
    def quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def normalize(angle: float) -> float:
        """Wrap to (-π, π] with trig magic."""
        return math.atan2(math.sin(angle), math.cos(angle))

    # ----------------- PID cores -----------------
    def pid_linear(self, err: float) -> float:
        self.integral_lin += err * self.dt
        der = (err - self.prev_error_lin) / self.dt
        self.prev_error_lin = err
        return self.kp_lin * err + self.ki_lin * self.integral_lin + self.kd_lin * der

    def pid_angular(self, err: float) -> float:
        self.integral_ang += err * self.dt
        der = (err - self.prev_error_ang) / self.dt
        self.prev_error_ang = err
        return self.kp_ang * err + self.ki_ang * self.integral_ang + self.kd_ang * der

    # ----------------- Main control loop -----------------
    def control_loop(self) -> None:
        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            return

        gx, gy = self.waypoints[self.current_index]
        dx, dy = gx - self.xr, gy - self.yr
        rho = math.hypot(dx, dy)

        desired_theta = math.atan2(dy, dx)
        alpha = self.normalize(desired_theta - self.thetar)

        # 1) Gira hasta estar alineado, 2) avanza
        if abs(alpha) > 0.20:
            v, w = 0.0, self.pid_angular(alpha)
        else:
            v, w = self.pid_linear(rho), self.pid_angular(alpha)

        # Saturación prudente
        cmd = Twist()
        cmd.linear.x = max(min(v, 0.30), -0.30)
        cmd.angular.z = max(min(w, 1.00), -1.00)
        self.cmd_pub.publish(cmd)

        # ¿Llegamos?
        if rho < 0.05:
            self.get_logger().info(f'Waypoint {self.current_index} alcanzado.')
            self.current_index += 1
            self.reset_pid()

    # ----------------- Utilities -----------------
    def reset_pid(self) -> None:
        self.integral_lin = self.integral_ang = 0.0
        self.prev_error_lin = self.prev_error_ang = 0.0

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())   # envía ceros
        self.get_logger().info('Trayectoria completada. ¡Hora del café!')


# ----------------- Main entry -----------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PIDSquareController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()