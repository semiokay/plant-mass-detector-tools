import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
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

MAX_DISTANCE_OF_POINTS_FROM_CAMERA = 1 # meters


# Class definition

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        
        # Generate callback groups
        timerCBGroup = MutuallyExclusiveCallbackGroup()
        pcCBGroup = MutuallyExclusiveCallbackGroup()
        odomCBGroup = MutuallyExclusiveCallbackGroup()
        
        # Initialize shutter variables
        self.shutter = False
        self.shutter_count = 0
        self.image_set = 0
        self.first = True
        self.odom_msg = None
        
        # Define point cloud save directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
        self.save_directory = f"/home/andre/Documents/ros2/cv_venv_ws/extra/PointClouds/{timestamp}_snap"
        
        # Ensure the save directory exists
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Subscribe to the PointCloud2 topic and /odom odometry readings
        self.pc_subscriber = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.pc_callback, 10, callback_group=pcCBGroup)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=odomCBGroup)
        self.pc_publisher = self.create_publisher(PointCloud2, '/input_pc', 10)
        self.pose_publisher = self.create_publisher(Pose, '/input_pose', 10)
        self.firing = self.create_publisher(Bool, '/firing', 10)

        self.get_logger().info("Subscribed to PointCloud2 topic")

        # Create a shutter_callback
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.shutter_callback, callback_group=timerCBGroup)

    # Class method

    def shutter_callback(self):
        if not self.shutter:
            command = input("Press Enter to take a snapshot -OR- Press any key + Enter to capture a new image set:")
            if (command == ''):
                self.shutter = True
                if self.first:
                    self.first = False
                else:
                    self.shutter_count += 1
            else:
                msg = Bool()
                msg.data = True
                self.firing.publish(msg)
                self.shutter_count = 0
                self.image_set += 1
                self.first = True
                self.get_logger().info("New image set generated.")

 
    def odom_callback(self, msg):
        # save odometry message to a state variable
        self.odom_msg = msg
                

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

                # Remove all points further than MAX_DISTANCE_OF_POINTS_FROM_CAMERA meters from the camera
                camera_position = np.array([0.0, 0.0, 0.0])
                pcd = self.filter_point_cloud_by_distance(pcd, camera_position, MAX_DISTANCE_OF_POINTS_FROM_CAMERA)

                # Publish data
                pcd_as_pc2 = self.place_open3d_data_in_point_cloud2(pcd, msg)
                self.pc_publisher.publish(pcd_as_pc2)
                self.pose_publisher.publish(self.odom_msg.pose.pose)

                # Save the point cloud to a .pcd file
                pc_filename = os.path.join(self.save_directory, "pc_%04d.pcd" % self.shutter_count)
                o3d_io.write_point_cloud(pc_filename, pcd)

                # Save the odometry to a .json file
                ex_filename = os.path.join(self.save_directory, "extrinsics_%04d.json" % self.shutter_count)
                with open(ex_filename, 'w') as f:
                    json_data = pose_to_json(self.odom_msg.pose)
                    json.dump(json_data, f, indent=4)
                
                self.get_logger().info(f"Saved PointCloud2 to {pc_filename} and Odometry to {ex_filename}")
                
                self.shutter = False


    def place_open3d_data_in_point_cloud2(self, pcd, reference_pointcloud2):
        # Convert Open3D PointCloud object to a PointCloud2 message
        header = reference_pointcloud2.header
        header.stamp = self.get_clock().now().to_msg()
        pcd_points = np.asarray(pcd.points)
        pcd_points = pcd_points.astype(np.float32)
        pcd_msg = pc2.create_cloud_xyz32(header=header, points=pcd_points)
        return pcd_msg


    def filter_point_cloud_by_distance(self, pcd, camera_position, max_distance):
        points = np.asarray(pcd.points)
        # Compute the distances between each point and the reference point
        distances = np.linalg.norm(points - camera_position, axis=1)
        # Filter points within the distance threshold
        filtered_points = points[distances <= max_distance]
        # Create a new point cloud with the filtered points
        filtered_pcd = o3d_geometry.PointCloud()
        filtered_pcd.points = o3d_utility.Vector3dVector(filtered_points)
        return filtered_pcd



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



# Main function

def main(args=None):
    rclpy.init(args=args)
    pc_saver = PointCloudSaver()
    spinMultipleThreads(pc_saver)
    pc_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()