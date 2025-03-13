import rclpy
import numpy as np
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import os
import datetime

# Constants


# Class definition

class VolumeEstimator(Node):
    def __init__(self):
        super().__init__('volume_estimator')

        # Define volume and point cloud save directory
        timestamp = datetime.now().strftime("%Y-%m-%d_%H_%M_%S%f")
        self.save_directory = f"/home/andre/Documents/ros2/cv_venv_ws/extra/PointClouds/{timestamp}_volumes"
        
        # Ensure the save directory exists
        os.makedirs(self.save_directory, exist_ok=True)

        # Define states
        self.count = 0
        self.volume_list = []
        
        # Subscribe to the PointCloud2 topic
        self.pc_subscriber = self.create_subscription(PointCloud2, '/output_pc', self.pc_callback, 10)

    def pc_callback(self, msg):
        # Convert PointCloud2 message to Open3D PointCloud
        pcd = self.PointCloud2_to_open3d_point_cloud(msg)
        
        # Display the point cloud
        o3d.visualization.draw_geometries([pcd])

        # Remove outliers
            # Remove ground
            # Remove pot
            # Remove other plants (approximately)
        o3d.visualization.draw_geometries([pcd])

        # Reconstruct surface
        radii = [0.005, 0.01, 0.02, 0.04] # Radii which the algorithm will use to determine surfaces
        rec_mesh = surface_reconstruction(pcd, radii) # Will take figurative balls with radii like in radii and roll them around the points, making surfaces where they are not able to pass through
        o3d.visualization.draw_geometries([pcd, rec_mesh])

        # Estimate volume

        volume = self.volume_estimator(rec_mesh)

        # Save the point cloud to a .pcd file, save volume to .csv file
        pc_filename = os.path.join(self.save_directory, "pc_%04d.pcd" % self.count)
        o3d.io.write_point_cloud(pc_filename, pcd)
        volume_filename = os.path.join(self.save_directory, "volume.csv")
        self.volume_list.append(volume)

    def PointCloud2_to_open3d_point_cloud(self, msg):
        # Convert PointCloud2 message to Open3D PointCloud
        xyz = np.array([[x, y, z] for x, y, z in self.pc2xyz(msg)])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd
    
    def pc2xyz(self, pc_msg):
        for p in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
            yield p

    def volume_estimator(self, rec_mesh):
        volume = rec_mesh.get_volume()
        return volume



# Auxiliary functions

def surface_reconstruction(pcd, radii):
    pcd.estimate_normals()  # ball pivot requires vertex normals
    pcd.orient_normals_consistent_tangent_plane(100) 
    surface_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting( pcd, o3d.utility.DoubleVector(radii) )
    return surface_mesh



# Main function

def main(args=None):
    rclpy.init(args=args)
    volume_estimator = VolumeEstimator()
    rclpy.spin(volume_estimator)
    volume_estimator.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()
