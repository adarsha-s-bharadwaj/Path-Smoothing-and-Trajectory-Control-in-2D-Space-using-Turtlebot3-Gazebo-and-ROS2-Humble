import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, hypot, pi
from nav_assignment.trajectory_generator import generate_trajectory

def normalize(a):
    while a > pi: a -= 2*pi
    while a < -pi: a += 2*pi
    return a

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.x = self.y = self.yaw = 0.0
        self.trajectory = generate_trajectory()

        self.phase = 'GO_TO_START'
        self.start_time = None

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = atan2(siny, cosy)

    def control_loop(self):
        cmd = Twist()

        # ----- Phase 1: Go to start -----
        if self.phase == 'GO_TO_START':
            tx, ty, _ = self.trajectory[0]
            dx, dy = tx - self.x, ty - self.y
            dist = hypot(dx, dy)

            if dist < 0.15:
                self.phase = 'TRACK'
                self.start_time = self.get_clock().now().nanoseconds * 1e-9
                self.cmd_pub.publish(Twist())
                return

            cmd.linear.x = 0.2
            cmd.angular.z = 1.5 * normalize(atan2(dy, dx) - self.yaw)
            self.cmd_pub.publish(cmd)
            return

        # ----- Phase 2: Time-based tracking -----
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.start_time

        target = None
        for p in self.trajectory:
            if p[2] >= t:
                target = p
                break

        # End of time-parameterized trajectory
        if target is None:
            end_x, end_y, _ = self.trajectory[-1]
            dx = end_x - self.x
            dy = end_y - self.y
            dist = hypot(dx, dy)

            if dist < 0.1:
                # reached final point
                self.cmd_pub.publish(Twist())
                return
            else:
                # spatial convergence to final waypoint
                cmd.linear.x = 0.15
                cmd.angular.z = 1.5 * normalize(atan2(dy, dx) - self.yaw)
                self.cmd_pub.publish(cmd)
                return

        tx, ty, _ = target
        dx, dy = tx - self.x, ty - self.y

        cmd.linear.x = 0.25
        cmd.angular.z = 2.0 * normalize(atan2(dy, dx) - self.yaw)
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(TrajectoryController())
    rclpy.shutdown()
