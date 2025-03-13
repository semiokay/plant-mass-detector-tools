import random

import rclpy
from rclpy.node import Node
from cloud_interfaces.srv import FuseClouds
import open3d as o3d
from open3d.visualization import draw_geometries as plot_pc
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import json
import transforms3d.quaternions as q

# Constants

MAX_ITERATIONS = 50
RELATIVE_FITNESS = 1e-6 # Cutoff min change in fitness between iterations. Fitness refers to the alignment quality between the two point clouds during each iteration. It is typically defined as the ratio of the number of inlier points (points that are successfully matched between the two point clouds) to the total number of points in the target point cloud.
RELATIVE_RMSE = 1e-6 # Cutoff min change in RMSE between iterations. RMSE is a measure of the alignment error between the two point clouds. Specifically, it is the square root of the average squared distances between corresponding inlier points in the two point clouds.
ACCEPTANCE_DISTANCE_POINT_TO_POINT = 0.01 # meters


class ICPServer(Node):
    # A ROS2 Node with a Service Server for ICP.

    def __init__(self):
        super().__init__('icp_server')

        self.serv = self.create_service(srv_type=FuseClouds, srv_name='/icp_service', callback=self.icp_callback)
        self.count: int = 0

    def icp_callback(self, request: FuseClouds.Request, response: FuseClouds.Response) -> FuseClouds.Response:
        # Takes two point clouds and the poses of the cameras that generated them
        # Returns one fused point cloud with a reference pose of the first camera

        self.get_logger().info("\n...........................\nServer request received.\n...........................")
        pc1 = PointCloud2_to_open3d_point_cloud(request.pc1)
        plot_pc([pc1])
        pc1_position, pc1_orientation = parse_extrinsics(request.pc1_pose)
        pc2 = PointCloud2_to_open3d_point_cloud(request.pc2)
        plot_pc([pc2])
        pc2_position, pc2_orientation = parse_extrinsics(request.pc2_pose)
        estimated_transformation = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        estimated_transformation[:3, 3] = pc2_position - pc1_position
        estimated_transformation[:3, :3] = q.quat2mat(q.qmult(pc2_orientation, q.qinverse(pc1_orientation)))
        self.get_logger().info(f"{pc1_position}\n{pc1_orientation}\n{pc2_position}\n{pc2_orientation}\n{estimated_transformation}")

        # Align point clouds 
        self.get_logger().info("Aligning point clouds...")
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=MAX_ITERATIONS,relative_fitness=RELATIVE_FITNESS,relative_rmse=RELATIVE_RMSE)
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        reg_icp = o3d.pipelines.registration.registration_icp(pc1, pc2, ACCEPTANCE_DISTANCE_POINT_TO_POINT, estimated_transformation, estimation_method, criteria)
        self.get_logger().info("\n...........................\nAlignment finished.\n...........................")
        transformation_matrix = reg_icp.transformation
        inverse_transform = np.linalg.inv(transformation_matrix)
        self.get_logger().info(f"Transformation matrix:\n{transformation_matrix}\nInverse transformation matrix:\n{inverse_transform}")
        pc2.transform(inverse_transform)

        self.get_logger().info("\n...........................\nFusing matrix...\n...........................")
        pc_fused = pc1 + pc2
        response.pc_fused = self.place_open3d_data_in_point_cloud2(pc_fused, request.pc1)
        response.pc_pose = request.pc1_pose

        self.get_logger().info("\n...........................\nServer response published.\n...........................")

        plot_pc([pc_fused])

        return response
    
    def place_open3d_data_in_point_cloud2(self, pcd, reference_pointcloud2):
        # Convert Open3D PointCloud object to a PointCloud2 message
        header = reference_pointcloud2.header
        header.stamp = self.get_clock().now().to_msg()
        pcd_points = np.asarray(pcd.points)
        pcd_points = pcd_points.astype(np.float32)
        pcd_msg = point_cloud2.create_cloud_xyz32(header=header, points=pcd_points)
        return pcd_msg


# Auxiliary functions

def parse_extrinsics(pose):
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    orientation = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    return position, orientation


def PointCloud2_to_open3d_point_cloud(pc2_msg):
    cloud_data = point_cloud2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)
    # Convert cloud_data to numpy array by extracting the individual x, y, z components
    points = []
    for point in cloud_data:
        points.append([point[0], point[1], point[2]])

    if points:
        np_points = np.array(points)
        # Convert the numpy array to an Open3D PointCloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)
        # Flip the point cloud or it will be upside down
        # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    return pcd


# Main function

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        icp_server = ICPServer()
        rclpy.spin(icp_server)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    rclpy.shutdown()


if __name__ == '__main__':
    main()