# control_trayectoria/pid_control.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class TrajectoryPID(Node):
    def __init__(self):
        super().__init__('pid_control')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.goals = [
            {'x': 2.0, 'y': 0.0},
            {'x': 2.0, 'y': 2.0},
            {'x': 0.0, 'y': 2.0},
            {'x': 0.0, 'y': 0.0}
        ]
        self.current_goal = 0

        self.pid_linear = PID(1.0, 0.0, 0.2)
        self.pid_angular = PID(4.0, 0.0, 0.3)

        self.last_time = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.pose['theta'] = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        goal = self.goals[self.current_goal]
        dx = goal['x'] - self.pose['x']
        dy = goal['y'] - self.pose['y']
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.pose['theta'])

        twist = Twist()

        if distance > 0.1:
            twist.linear.x = self.pid_linear.compute(distance, dt)
            twist.angular.z = self.pid_angular.compute(angle_error, dt)
        else:
            self.current_goal = (self.current_goal + 1) % len(self.goals)

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
