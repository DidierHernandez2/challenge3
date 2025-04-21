import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def compute(self, setpoint, actual):
        current_time = time.time()
        dt = current_time - self.prev_time
        error = setpoint - actual
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        self.prev_time = current_time
        return output


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('pid_trajectory_controller')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.publisher_ = self.create_publisher(Twist, 'motor_cmd', 10)

        # PID para control lineal y angular
        self.linear_pid = PIDController(1.0, 0.0, 0.1)
        self.angular_pid = PIDController(2.0, 0.0, 0.2)

        self.current_step = 0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Simulaciones básicas de posición/orientación
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def cmd_vel_callback(self, msg):
        self.desired_linear = msg.linear.x
        self.desired_angular = msg.angular.z

    def control_loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.start_time

        # Ejecutar trayectoria cuadrada
        side_duration = 4  # segundos para cada lado (ajustable)
        step = elapsed // side_duration

        if step != self.current_step:
            self.get_logger().info(f"Paso {step}/4")
            self.current_step = step
            self.start_time = now

        if step < 4:
            linear_setpoint = 0.2  # m/s
            angular_setpoint = 0.0
        elif step < 8:
            linear_setpoint = 0.0
            angular_setpoint = math.pi / 4  # rad/s
        else:
            linear_setpoint = 0.0
            angular_setpoint = 0.0

        # Simulamos "lectura" del encoder
        linear_actual = self.desired_linear if hasattr(self, 'desired_linear') else 0.0
        angular_actual = self.desired_angular if hasattr(self, 'desired_angular') else 0.0

        # Control PID
        v = self.linear_pid.compute(linear_setpoint, linear_actual)
        w = self.angular_pid.compute(angular_setpoint, angular_actual)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
