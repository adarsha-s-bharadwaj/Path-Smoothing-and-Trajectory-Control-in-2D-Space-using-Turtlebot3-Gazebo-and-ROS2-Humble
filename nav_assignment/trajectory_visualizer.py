"""
trajectory_visualizer.py

ROS 2 visualization node that publishes the planned trajectory
as a LINE_STRIP marker for RViz.

Purpose:
- Debugging
- Validation of spline smoothing
- Visual comparison between path and robot motion
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_assignment.trajectory_generator import generate_trajectory

class TrajectoryViz(Node):
    """
    Publishes trajectory visualization markers.
    """
     
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.pub = self.create_publisher(Marker, '/trajectory', 1)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        """
        Publish trajectory as a LINE_STRIP marker in RViz.
        """
        
        traj = generate_trajectory()

        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.g = 1.0
        m.color.a = 1.0

        for x, y, _ in traj:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            m.points.append(p)

        self.pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(TrajectoryViz())
    rclpy.shutdown()
