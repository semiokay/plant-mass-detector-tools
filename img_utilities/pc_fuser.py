# Imports

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool
import os
import json
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
from cloud_interfaces.srv import FuseClouds

# Constants

TIMER_FREQUENCY = 2
SERVICE_WAIT_TIMEOUT = 1.0 # s


# Class definition

class PCFuser(Node):
    # This node collects pointcloud2 data from /input_pc and pose data from /input_pose 
    # and fuses it with existing pointcloud data using icp (from open3d).
    # Once it receives a firing command from /firing, it publishes the accumulated point clouds to /output_pc
    # and clears the point cloud list, allowing it to do a reset.
    # This node uses the /icp_service node
    def __init__(self):
        super().__init__('pc_fuser')
        
        # Generate callback groups
        timerCBGroup = MutuallyExclusiveCallbackGroup()
        pcCBGroup = MutuallyExclusiveCallbackGroup()
        firingCBGroup = MutuallyExclusiveCallbackGroup()
        clientCBGroup = MutuallyExclusiveCallbackGroup()
        
        # Initialize state variables
        self.pc = None
        self.pc_pose = None
        self.pc_new = None
        self.pc_new_pose = None
        self.available = True
        self.firing_signal = False
        self.sync = False
        
        # Subscribe to the PointCloud2 topic
        self.firing = self.create_subscription(Bool, '/firing', self.firing_callback, 10, callback_group=firingCBGroup)
        self.pc_subscriber = self.create_subscription(PointCloud2, '/input_pc', self.pc_callback, 10, callback_group=pcCBGroup)
        self.pose_subscriber = self.create_subscription(Pose, '/input_pose', self.pose_callback, 10, callback_group=pcCBGroup)
        self.pc_publisher = self.create_publisher(PointCloud2, '/output_pc', 10)
        self.cli = self.create_client(FuseClouds, "/icp_service", callback_group=clientCBGroup)
        self.waitForService()

        # Create a timer_callback
        timer_period = 1.0/TIMER_FREQUENCY  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=timerCBGroup)

    # Class methods

    def timer_callback(self):
        if self.available & self.firing_signal:
            self.pc_publisher.publish(self.pc)
            self.get_logger().info("...........................\nPoint cloud published.\n...........................")
            self.pc = None
            self.pc_pose = None
            self.firing_signal = False
    
    
    def pc_callback(self, msg):
        if self.pc == None: 
            self.pc = msg
        elif (self.pc != None) & (self.pc_pose != None):
            self.pc_new = msg
            self.attempt_sync_fusion()
        else:
            pass
    

    def firing_callback(self, msg):
        self.firing_signal = msg.data
        self.get_logger().info("...........................\nFiring signal received.\n...........................")


    def pose_callback(self, msg):
        if self.pc_pose == None:
            self.pc_pose = msg
        elif (self.pc != None) & (self.pc_pose != None):
            self.pc_new_pose = msg
            self.attempt_sync_fusion()
        else:
            pass

  
    def waitForService(self):
        
        while not self.cli.wait_for_service( timeout_sec = SERVICE_WAIT_TIMEOUT ):
            self.get_logger().info("Service unavailable. Waiting again...\n")
        
        self.get_logger().info("Contact with service successful!\n")


    def attempt_sync_fusion(self):
        if (self.pc_new != None) and (self.pc_new_pose != None):
            # Create server message
            fuse_msg = FuseClouds.Request()
            fuse_msg.pc1 = self.pc
            fuse_msg.pc1_pose = self.pc_pose
            fuse_msg.pc2 = self.pc_new
            fuse_msg.pc2_pose = self.pc_new_pose

            # Send to server
            self.available = False
            self.future = self.cli.call_async(fuse_msg)
            self.future.add_done_callback(self.handle_response)

            # Reset incoming data
            self.pc_new = None
            self.pc_new_pose = None


    def handle_response(self, future):
        try:
            response = future.result()
            self.pc = response.pc_fused
            self.pc_pose = response.pc_pose
            self.available = True
            self.get_logger().info("...........................\nServer response received.\n...........................")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

        return response



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
    pc_fuser = PCFuser()
    spinMultipleThreads(pc_fuser)
    pc_fuser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()