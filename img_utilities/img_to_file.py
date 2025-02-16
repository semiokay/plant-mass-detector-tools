import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Constants

TIMER_FREQUENCY = 2

class RealSenseImageSaver(Node):
    def __init__(self):
        super().__init__('realsense_image_saver')
        
        # Generate callback groups
        timerCBGroup = MutuallyExclusiveCallbackGroup()
        imageCBGroup = MutuallyExclusiveCallbackGroup()
        
        # Initialize shutter variables
        self.shutter_rgb = False
        self.shutter_depth = False 
        self.shutter_count = 0
        self.first = True
        
        # Initialize CvBridge for converting ROS images to OpenCV images
        self.bridge = CvBridge()
        
        # Define image save directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
        self.save_directory = f"/home/andre/Documents/ros2/cv_venv_ws/extra/Images/rosStream/{timestamp}_stream"
        
        # Ensure the save directory exists
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Subscribe to the RGB and Depth image topics
        self.rgb_subscriber = self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10, callback_group=imageCBGroup)
        self.depth_subscriber = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10, callback_group=imageCBGroup)
        self.get_logger().info("Subscribed to RGB and Depth topics")

        # Create a shutter_callback
        timer_period = 1.0/TIMER_FREQUENCY # seconds
        self.timer = self.create_timer(timer_period, self.shutter_callback, callback_group=timerCBGroup)

    def shutter_callback(self):
        if (self.shutter_rgb == False and self.shutter_depth == False):
            input("Press Enter to take a picture")
            self.shutter_rgb = True
            self.shutter_depth = True 
            if (self.first):
                self.first = False
            else:
                self.shutter_count += 1
    
    def rgb_callback(self, msg):
        if (self.shutter_rgb == True):
            # Convert the ROS image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Get timestamp for the filename
            # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
            
            # Save the RGB image to the directory
            rgb_filename = os.path.join(self.save_directory, "rgb_%04d.png" % self.shutter_count)
            cv2.imwrite(rgb_filename, cv_image)
            
            self.get_logger().info(f"Saved RGB image to {rgb_filename}")
            self.shutter_rgb = False

    def depth_callback(self, msg):
        if (self.shutter_depth == True):
            # Convert the ROS image message to a CV2 image (depth is usually 16-bit single channel)
            cv_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            
            # Get timestamp for the filename
            # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
            
            # Save the depth image to the directory
            depth_filename = os.path.join(self.save_directory, "depth_%04d.png" % self.shutter_count)
            cv2.imwrite(depth_filename, cv_image)
            
            self.get_logger().info(f"Saved Depth image to {depth_filename}")
            self.shutter_depth = False


# Auxiliary functions

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
    image_saver = RealSenseImageSaver()
    spinMultipleThreads(image_saver)
    rclpy.shutdown()
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
