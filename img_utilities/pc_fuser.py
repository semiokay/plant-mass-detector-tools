# Imports

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import os
import json
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# import open3d as o3d
from open3d import geometry as o3d_geometry
from open3d import utility as o3d_utility
from open3d import io as o3d_io
import numpy as np

# Constants

TIMER_FREQUENCY = 2


# Class definition

class NodeName(Node):
    def __init__(self):
        super().__init__('node_name')
        
        # Generate callback groups
        timerCBGroup = MutuallyExclusiveCallbackGroup()
        pcCBGroup = MutuallyExclusiveCallbackGroup()
        
        # Initialize shutter variables
        self.shutter = False
        self.shutter_count = 0
        self.first = True
        
        # Define point cloud save directory
        timestamp = datetime.now().strftime("%Y%m%d_%H  %M%S%f")
        self.save_directory = f"/home/andre/Documents/ros2/cv_venv_ws/extra/PointClouds/{timestamp}_snap"
        
        # Ensure the save directory exists
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Subscribe to the PointCloud2 topic
        self.pc_subscriber = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.pc_callback, 10, callback_group=pcCBGroup)
        self.get_logger().info("Subscribed to PointCloud2 topic")

        # Create a shutter_callback
        timer_period = 1.0/TIMER_FREQUENCY  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=timerCBGroup)

    # Class methods

    def timer_callback(self):
        if not self.shutter:
            input("Press Enter to take a snapshot")
            self.shutter = True
            if self.first:
                self.first = False
            else:
                self.shutter_count += 1
                

    def pc_callback(self, msg):
        if self.shutter:
            
            # Convert the PointCloud2 message to a list of points (x, y, z)
            cloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
             # Convert cloud_data to numpy array by extracting the individual x, y, z components
            points = []
            for point in cloud_data:
                points.append([point[0], point[1], point[2]])

            if points:
                # Convert the list of points into a numpy array
                np_points = np.array(points)

                # Convert the numpy array to an Open3D PointCloud object
                pcd = o3d_geometry.PointCloud()
                pcd.points = o3d_utility.Vector3dVector(np_points)

                # Flip the point cloud or it will be upside down
                pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

                # Save the point cloud to a .pcd file
                pc_filename = os.path.join(self.save_directory, "pc_%04d.pcd" % self.shutter_count)
                o3d_io.write_point_cloud(pc_filename, pcd)

                ex_filename = os.path.join(self.save_directory, "extrinsics_%04d.json" % self.shutter_count)
                with open(ex_filename, 'w') as f:
                    json_data = pose_to_json(self.odom_msg.pose)
                    json.dump(json_data, f, indent=4)
                
                self.get_logger().info(f"Saved PointCloud2 to {pc_filename} and Odometry to {ex_filename}")
                
                self.shutter = False


# Auxiliary functions

def pose_to_json(pose):
    return {
        "position": {
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
            "z": pose.pose.position.z
        },
        "orientation": {
            "w": pose.pose.orientation.w,
            "x": pose.pose.orientation.x,
            "y": pose.pose.orientation.y,
            "z": pose.pose.orientation.z
        } #,
        # "covariance": pose.covariance
    }


def spinMultipleThreads(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info(f"Starting {node.get_name()}, shut down with Ctrl+C\n") 
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down. \n\n\n\n\n")

def main(args=None):
    rclpy.init(args=args)
    node_name = NodeName()
    spinMultipleThreads(node_name)
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()