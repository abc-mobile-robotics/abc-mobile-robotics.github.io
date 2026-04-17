import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan,PointCloud2,PointField
from visualization_msgs.msg import Marker,MarkerArray

class wolf_node(Node):
    def __init__(self):
       super().__init__("wolf mode")

       # Declaring Variables
       self.variable = 0.0

       # Publishing the command to the turtlebot node 
       
       self.publishers = self.create_publisher(TwistStamped,"robot_0/cmd_vel",10)
       self.publishers = self.create_publisher(LaserScan,"robot_0/<node name>",10)
       self.publishers = self.create_publisher()

       # Subscribing to the robot node for update state

       self.subscriptions = self.create_subscription()
       self.subscriptions = self.create_subscription()

    def initialize_yolo():
        pass

    def wolf_drive():
        pass


def main():
    node = wolf_node()


if __name__ == "__main__":
    main()