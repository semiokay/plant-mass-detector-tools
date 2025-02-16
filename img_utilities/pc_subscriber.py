import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import transforms3d

import os
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Constants

TIMER_FREQUENCY = 2

class PCSubscriber(Node):
    def __init__(self):
        super().__init__('pc_subscriber')
        
        # Generate callback groups
        timerCBGroup = MutuallyExclusiveCallbackGroup()
        imageCBGroup = MutuallyExclusiveCallbackGroup()
        gyroCBGroup = MutuallyExclusiveCallbackGroup()
        accelCBGroup = MutuallyExclusiveCallbackGroup()
        
        # Initialize shutter variables
        self.shutter_pc = False
        self.shutter_gyro = False
        self.shutter_accel = False
        self.shutter_count = 0
        self.first = True
        self.orientation = np.array([1, 0, 0, 0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.past_time = None
        
        # Define image save directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
        self.save_directory = f"/home/andre/Documents/ros2/cv_venv_ws/extra/Images/rosStream/{timestamp}_stream"
        
        # Ensure the save directory exists
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Create Quality of Service requirements to be able to use IMU
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        # Subscribe to the image topics
        self.pointCloud2_subscriber = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.pc_callback, 10, callback_group=imageCBGroup)
        self.gyro_subscriber = self.create_subscription(Imu, '/camera/camera/gyro/sample', self.gyro_callback, qos_policy, callback_group=gyroCBGroup)
        self.accel_subscriber = self.create_subscription(Imu, '/camera/camera/accel/sample', self.accel_callback, qos_policy, callback_group=accelCBGroup)
        self.get_logger().info("Subscribed to pointcloud2 and imu topics")

        # Create a shutter_callback
        timer_period = 1.0/TIMER_FREQUENCY # seconds
        self.timer = self.create_timer(timer_period, self.shutter_callback, callback_group=timerCBGroup)

    def shutter_callback(self):
        # if ((self.shutter_pc == False) and (self.shutter_gyro == False) and (self.shutter_accel == False)):
        if (self.shutter_pc == False):
            input("Press Enter to capture a point cloud view")
            self.shutter_pc = True 
            self.shutter_gyro = True
            self.shutter_accel = True
            if (self.first):
                self.first = False
            else:
                self.shutter_count += 1
    

    def pc_callback(self, msg):
        if (self.shutter_pc == True):
            # Process the PointCloud2 data
            open3d_pcd = self.pointCloud2_to_open3d(msg)
            
            # Flip the point cloud or it will be upside down
            open3d_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

            # Save the point cloud to the directory
            pc_filename = os.path.join(self.save_directory, "pc_%04d.ply" % self.shutter_count)
            o3d.io.write_point_cloud(pc_filename, open3d_pcd) # Save the point cloud, can be .ply, .pcd, .xyz, etc...
            
            self.get_logger().info(f"Saved point cloud to {pc_filename}")
            self.shutter_pc = False
            o3d.visualization.draw_geometries([open3d_pcd])


    def gyro_callback(self, msg):
        new_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.past_time is None:
            self.past_time = new_time
        else:
            dt = new_time - self.past_time
            self.past_time = new_time
            self.angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            # Create quaternion representation (q) and derivative (dq)
            q = np.hstack([[0], self.angular_velocity])
            dq = 0.5 * transforms3d.quaternions.qmult(self.orientation, q)
            self.orientation += dq * dt
            self.orientation /= np.linalg.norm(self.orientation)
        if (self.shutter_gyro == True):
            # Get  angular_velocity
            self.shutter_gyro = False


    def accel_callback(self, msg):
        self.acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if (self.shutter_accel == True):
            # Get linear_acceleration
            self.shutter_accel = False


    def pointCloud2_to_open3d(self, msg):
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_data_array = np.array(list(pc_data))
        if pc_data_array.dtype.names is not None: # If the array has named fields, apply filtering for NaNs
            mask = np.isfinite(pc_data_array['x']) & np.isfinite(pc_data_array['y']) & np.isfinite(pc_data_array['z']) # Create a mask that filters out NaNs in any field (x, y, z)
            pc_data_array = pc_data_array[mask] # Apply the mask to filter the rows
        points_2d_array = np.vstack([pc_data_array['x'], pc_data_array['y'], pc_data_array['z']]).T # Extract the 'x', 'y', and 'z' fields and stack them into a 2D array (N, 3)
        points_2d_array = points_2d_array.astype(np.float64) # Convert the array to float64 (required for Open3D)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_2d_array)
        return pcd



# Auxiliary functions

def spinMultipleThreads(node):

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info(f"Starting {node.get_name()}, shut down with Ctrl+C\n") 
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down. \n\n\n\n\n")


def RANSAC_cone(points, iterations=1000, threshold=0.1):
    # For detecting pots, must have a very low threshold for max_distance, since the pot is typically a perfect cylinder
    best_inliers_list = []
    best_plane_coefficients = None

    for iteration in iterations:
        if len(inliers_list) > (best_inliers_list):
            best_inliers_list = inliers_list 
            best_plane_coefficients = plane_coefficients 
    

    return best_inliers_list, best_plane_coefficients


def RANSAC_plane(pc, iterations=1000, threshold=0.01):
    # For detecting walls and floors
    best_inliers_list = []
    best_plane_coefficients = None
    points = np.asarray(pc.points)

    for iteration in range(iterations):
        # Sampling of 3 random points
        number_of_points = points.shape[0]
        number_of_outputs = 3
        random_indices = np.random.choice(number_of_points,number_of_outputs,replace=False)
        sample = points[random_indices]
        plane_coefficients = create_plane_from_3_points(sample[0], sample[1], sample[2])

        distances = np.abs(np.dot(points, plane_coefficients[0:3]) - plane_coefficients[3]) / np.linalg.norm(plane_coefficients[0:3])
        inliers_list = np.where(distances < threshold)[0]
        if len(inliers_list) > len(best_inliers_list):
            best_inliers_list = inliers_list 
            best_plane_coefficients = plane_coefficients 
    
    plot_inliers(pc, best_inliers_list)
    return best_inliers_list, best_plane_coefficients


def create_plane_from_3_points(p1, p2, p3):
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    a, b, c = normal / np.linalg.norm(normal)
    d = np.dot(normal, p1)
    plane_coefficients = np.array([a, b, c, d])
    return plane_coefficients


def plot_inliers(pc, inliers_list, rgb_triad=[0.20, 0.92, 0.69]):
    points = np.asarray(pc.points)
    inliers = o3d.geometry.PointCloud()
    inliers.points = o3d.utility.Vector3dVector(points[inliers_list])
    inliers.paint_uniform_color(rgb_triad) # Green-turquoise
    
    outliers = o3d.geometry.PointCloud()
    outlier_points = np.delete(points, inliers_list, axis=0)
    outliers.points = o3d.utility.Vector3dVector(outlier_points)
    outliers.paint_uniform_color([0.71, 0.71, 0.71]) # Grey

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

    o3d.visualization.draw_geometries([inliers, outliers])

    return inliers, outliers


# Main function

def main(args=None):
    
    rclpy.init(args=args)
    pc_subscriber = PCSubscriber()
    spinMultipleThreads(pc_subscriber)
    rclpy.shutdown()
    pc_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
