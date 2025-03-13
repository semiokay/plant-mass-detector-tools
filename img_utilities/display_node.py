import rclpy
import numpy as np
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# Constants


# Class definition

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        
        # Subscribe to the PointCloud2 topic
        self.pc_subscriber = self.create_subscription(PointCloud2, '/output_pc', self.pc_callback, 10)

    def pc_callback(self, msg):
        # Convert PointCloud2 message to Open3D PointCloud
        self.get_logger().info("...........................\nPoint cloud received.\n...........................")
        pcd = self.PointCloud2_to_open3d_point_cloud(msg)
        
        # Display the point cloud
        o3d.visualization.draw_geometries([pcd])

    def PointCloud2_to_open3d_point_cloud(self, msg):
        # Convert PointCloud2 message to Open3D PointCloud
        xyz = np.array([[x, y, z] for x, y, z in self.pc2xyz(msg)])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd
    
    def pc2xyz(self, pc_msg):
        for p in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
            yield p


def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()
    rclpy.spin(display_node)
    display_node.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()
