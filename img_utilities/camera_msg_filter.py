import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import numpy as np
import transforms3d
import os
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String

# Constants
# timer frequency determined by when imu data is received by the camera
ALPHA = 0.1
STABILIZATION_LIMIT = 300
PROXIMITY = 0.05
VERBOSE = False
VERBOSE_ACCEL = False
VERBOSE_ORIENTATION = False
ROTATION = np.array([
        [0, 0, -1],
        [1, 0, 0],
        [0, 1, 0]
    ]) # Used in the Realsense to ENU function only
SPIN = np.array([-1,-1,-1]) # Used in the gyro callback only to affect the sign of the angular velocity from msg, useful because realsense is LHR and ENU is RHR
Q_GRAVITY_GLOBAL = np.array([0,0,0,-1]) # Global gravity vector in quaternion form

class CameraMessageFilter(Node):

    def __init__(self):
        super().__init__('camera_message_filter')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('alpha', ALPHA),
            ]
        )
        # States 
        self.angular_velocity = None # np.array([0.0, 0.0, 0.0])
        self.acceleration = None # np.array([0.0, 0.0, 0.0])
        self.acceleration_accum = np.array([0.0, 0.0, -9.8])
        self.alpha = self.get_parameter('alpha').value
        self.orientation = None
        self.past_time = None
        self.past_time_accel = None
        self.processing_gravity_counter = 0
        self.translation = None
        self.velocity = None
        self.gravity_magnitude = 9.81

        # Generate callback groups
        gyroCBGroup = MutuallyExclusiveCallbackGroup()
        accelCBGroup = MutuallyExclusiveCallbackGroup()

        # Create Quality of Service requirements to be able to use IMU
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.gyro_subscriber = self.create_subscription(Imu, '/camera/camera/gyro/sample', self.gyro_callback, qos_policy, callback_group=gyroCBGroup)
        self.accel_subscriber = self.create_subscription(Imu, '/camera/camera/accel/sample', self.accel_callback, qos_policy, callback_group=accelCBGroup)
        self.imu_publisher = self.create_publisher(Imu, '/imu_with_orientation', 10)
        self.marker_publisher = self.create_publisher(Marker, '/orientation_marker', 10)
        self.initial_broadcaster = StaticTransformBroadcaster(self)
        self.pose_broadcaster = TransformBroadcaster(self)
        if VERBOSE:
            self.get_logger().info("Class initiated.")
        

    # Class functions

    def gyro_callback(self, msg):
        new_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # If this is the first time the gyro data is received, set the past time to the current time
        # Also, if the initial orientation has not been set, keep setting the past time to the current time. It is needed to calculate orientation
        if self.past_time is None or self.orientation is None:
            self.past_time = new_time
        else: # when self.orientation has been set (processing gravity has finished) and past_time has been set, proceed
            self.angular_velocity = realsense_to_enu(SPIN*np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
            self.update_orientation(self.angular_velocity, new_time)
            self.publish_imu()
            self.publish_marker()


    def accel_callback(self, msg):
        while (self.processing_gravity_counter < STABILIZATION_LIMIT):
            self.processing_gravity(msg)
        
        if self.orientation is None:
            self.acceleration = realsense_to_enu(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
            if VERBOSE or VERBOSE_ACCEL:
                self.get_logger().info(f"Acceleration (Orientation in None) is: {self.acceleration[0]}, {self.acceleration[1]}, {self.acceleration[2]}")
        else:
            if self.past_time_accel is None:
                self.past_time_accel = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.translation = np.array([0.0, 0.0, 0.0])
                self.velocity = np.array([0.0, 0.0, 0.0])

            # Acceleration corrected to ENU convention, Z must be up. Currently, X,Y,Z = y_ENU,z_ENU,-x_ENU
            self.compensate_acceleration(msg, self.gravity_magnitude) # acceleration updated in this function
            # acceleration_local = realsense_to_enu(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])) 
            # gravity = rotate_by_quaternion(self.orientation, Q_GRAVITY_GLOBAL*self.gravity_magnitude)
            # acceleration_compensated_local = -gravity + np.array([0, acceleration_local[0], acceleration_local[1], acceleration_local[2]])
            # self.acceleration = (9.81/self.gravity_magnitude)*rotate_by_quaternion(transforms3d.quaternions.qconjugate(self.orientation),acceleration_compensated_local)[1:]
            # self.acceleration_accum = self.acceleration*self.alpha*self.alpha + self.acceleration_accum*(1-self.alpha*self.alpha)
            # self.acceleration = self.acceleration_accum
            self.translation, self.velocity = self.update_translation(self.acceleration, self.velocity, self.translation, self.past_time_accel, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            tf_current = TransformStamped_from_orientation_and_translation(self.orientation, self.translation, "updated_camera", "world", self.get_clock().now().to_msg())
            self.pose_broadcaster.sendTransform(tf_current)
        if VERBOSE or VERBOSE_ACCEL:
            self.get_logger().info(f"Acceleration is: {self.acceleration[0]}, {self.acceleration[1]}, {self.acceleration[2]}")


    def publish_imu(self):
        if self.angular_velocity is None or self.acceleration is None or self.orientation is None:
            pass
        else:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.linear_acceleration.x = self.acceleration[0]
            msg.linear_acceleration.y = self.acceleration[1]
            msg.linear_acceleration.z = self.acceleration[2]
            msg.angular_velocity.x = self.angular_velocity[0]
            msg.angular_velocity.y = self.angular_velocity[1]
            msg.angular_velocity.z = self.angular_velocity[2]
            msg.orientation.w = self.orientation[0]
            msg.orientation.x = self.orientation[1]
            msg.orientation.y = self.orientation[2]
            msg.orientation.z = self.orientation[3]
            
            self.imu_publisher.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg.data)
            if VERBOSE or VERBOSE_ORIENTATION:
                self.get_logger().info(f"Orientation is: {self.orientation[0]}, {self.orientation[1]}, {self.orientation[2]}, {self.orientation[3]}")


    def processing_gravity(self, msg):

        # Finds the average acceleration and establishes that as the gravity vector
        # It also establishes the initial orientation of the camera based on the gravity vector
        # Important results: self.acceleration_accum, self.orientation
        # Less important results that can be extracted if needed: gravity_triad
        # Stops processing after STABILIZATION_LIMIT consecutive measurements are within PROXIMITY of the average

        # Get acceleration
        acceleration = realsense_to_enu(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))

        # Find average acceleration based on new measurement and previous average, based on alpha
        self.acceleration_accum =  ( 1 - self.alpha )* self.acceleration_accum + self.alpha * acceleration
        if VERBOSE:
                self.get_logger().info(
                    f"Acceleration is: {acceleration[0]}, {acceleration[1]}, {acceleration[2]}\n" +
                    f"Accum Accele is: {self.acceleration[0]}, {self.acceleration[1]}, {self.acceleration[2]}"
                )
        if (np.linalg.norm(acceleration - self.acceleration_accum) < PROXIMITY):
            self.processing_gravity_counter += 1 # This seems like it will increase to infinity, but this function will not be called past STAIBILIZATION_LIMIT
        else:
            self.processing_gravity_counter = 0

        if self.processing_gravity_counter >= STABILIZATION_LIMIT:
            self.get_logger().info(f"Gravity stabilized at {self.acceleration_accum}")
            # Calculate triad from negative gravity (normalized). This is the GLOBAL coordinate system in LOCAL coordinates
            self.gravity_magnitude = np.linalg.norm(self.acceleration_accum)
            normalized_accel_accum = self.acceleration_accum/np.linalg.norm(self.acceleration_accum)

            # Broadcast initial transform, set self.orientation to initial orientation in quaternion form
            translation = np.array([0.0,0.0,0.0])
            self.orientation = find_transform_quaternion(-normalized_accel_accum,np.array([0,0,1]))
            tf_initial = TransformStamped_from_orientation_and_translation(self.orientation, translation, "initial_camera", "world", self.get_clock().now().to_msg())
            self.initial_broadcaster.sendTransform(tf_initial)
            print(tf_initial)

            self.acceleration_accum = np.array([0.0, 0.0, 0.0]) # Reset gravity accumulator


    def update_orientation(self, angular_velocity, new_time):
        # Calculate orientation based on angular velocity
        dt = new_time - self.past_time
        self.past_time = new_time
        q = np.hstack([[0], angular_velocity])
        dq = 0.5 * transforms3d.quaternions.qmult(self.orientation, q)
        self.orientation += dq * dt
        self.orientation /= np.linalg.norm(self.orientation)


    def update_translation(self, acceleration, velocity, translation, past_time, new_time):
        # Calculate translation based on acceleration
        dt = new_time - past_time
        self.past_time_accel = new_time
        velocity += dt * acceleration
        translation += dt * velocity
        if np.linalg.norm(translation) >= 2:
            translation = 2*translation/np.linalg.norm(translation)
            velocity = np.array([0.0, 0.0, 0.0])

        return translation, velocity


    def publish_marker(self):
        # Create a marker object
        marker = Marker()
        
        # Header: set the frame_id (coordinate frame in which the vector is expressed)
        marker.header = Header()
        marker.header.frame_id = "world"  # or any other reference frame you want
        
        marker.type = Marker.ARROW  # Define the marker type as an arrow
        
        marker.action = Marker.ADD  # We are adding the marker (not modifying or deleting)
        
        # Position the arrow at the origin (you can change this to any desired location)
        marker.pose.position = Point(x=0.0, y=0.0, z=0.0)
        
        # Set the direction of the arrow using the vector components (x, y, z)
        marker.pose.orientation.x = self.orientation[0]
        marker.pose.orientation.y = self.orientation[1]
        marker.pose.orientation.z = self.orientation[2]
        marker.pose.orientation.w = self.orientation[3]
        
        # Set the scale of the arrow (this controls the size of the arrow)
        marker.scale.x = 0.1  # The length of the arrow (X direction)
        marker.scale.y = 0.02  # The width of the arrow shaft
        marker.scale.z = 0.02  # The width of the arrow head
        
        # Set the color of the arrow (RGBA format)
        marker.color.a = 1.0  # Full opacity
        marker.color.r = 1.0  # Red color
        
        # Set the direction of the arrow (this is where the vector is represented)
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=0.0, z=-1.0)]  # Example vector: (1, 2, 0)
        
        # Publish the marker
        self.marker_publisher.publish(marker)


    def compensate_acceleration(self, msg, g_mag):
        # Get local acceleration from imu and convert to ENU coordinates and put it in the global frame
        accel_msg = realsense_to_enu(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])) 
        acceleration_local = np.array([0, accel_msg[0], accel_msg[1], accel_msg[2]])
        acceleration_global = rotate_by_quaternion(transforms3d.quaternions.qconjugate(self.orientation),acceleration_local)
        
        # Compensation tactics
        alpha_scaling = 5.0
        movement_threshold = 0.25
        # Remove gravity factor and normalize gravity acceleration to 9.81
        acceleration_compensated_global = (9.81/g_mag) * (acceleration_global - Q_GRAVITY_GLOBAL*g_mag)
        alpha = self.alpha * alpha_scaling
        # set state variables
        self.acceleration_accum = acceleration_compensated_global[1:]*alpha + self.acceleration_accum*(1-alpha)
        self.acceleration = (self.acceleration_accum > movement_threshold) * self.acceleration_accum


# Auxiliary Functions

def spinMultipleThreads(node):

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info(f"Starting {node.get_name()}, shut down with Ctrl+C\n") 
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down. \n\n\n\n\n")


def triad_from_normal(normal):
    # Take first 3 values from normal
    normal = np.array([normal[0],normal[1],normal[2]])  

    # Define reference axes
    x_axis = np.array([1,0,0])
    y_axis = np.array([0,1,0])

    # Find point orthogonal to normal and reference y-axis. This will be the triad's x-axis.
    x_normal = np.cross(y_axis,normal)
    # Handle case where normal and y-axis are in the same direction and cross product is 0
    if (x_normal == np.zeros((1,3))).all: 
        x_normal = np.cross(x_axis,normal)    
    
    # Create final part of the triad (y) and normalize
    y_normal = np.cross(normal,x_normal)
    x_normal = 1/np.linalg.norm(x_normal) * x_normal
    y_normal = 1/np.linalg.norm(y_normal) * y_normal
    normal   = 1/np.linalg.norm(normal)   * normal

    triad = np.vstack((x_normal,y_normal,normal))
    
    return triad


def find_transform_quaternion(local_v, global_v):
    # Finds transform quaternion that transforms local_v to global_v
    
    # First, check whether the inputs are 3-position vectors or 4-position quaternions
    if ((len(local_v) < 3) and (len(local_v) > 4)) or ((len(global_v) < 3) and (len(global_v) > 4)):
        raise ValueError("The input vectors must have either 3 or 4 elements.")
    if len(local_v) == 4:
        # Make sure quaternions are pure quaternions
        if local_v[0] != 0:
            raise ValueError("The first element of the quaternion input must be 0.")
        local_v = np.array([local_v[1],local_v[2],local_v[3]])
    if len(global_v) == 4: 
        # Make sure quaternions are pure quaternions 
        if local_v[0] != 0:
            raise ValueError("The first element of the quaternion input must be 0.")
        global_v = np.array([global_v[1],global_v[2],global_v[3]])

    # Normalize the vectors
    local_v = local_v/np.linalg.norm(local_v)
    global_v = global_v/np.linalg.norm(global_v)

    # Calculate the transform quaternion
    if np.isclose(np.dot(local_v, global_v), -1):
        new_v = np.array([1,0,0])
        if np.isclose(np.dot(local_v, new_v), -1):
            new_v = np.array([0,1,0])
        axis_of_rotation = np.cross(local_v, new_v)
        transform_quaternion =  np.array([0,axis_of_rotation[0],axis_of_rotation[1],axis_of_rotation[2]]) 
    elif np.isclose(np.dot(local_v, np.array([1,0,0])), 1):
        transform_quaternion = np.array([1,0,0,0])
    else:
        axis_of_rotation = np.cross(local_v, global_v)
        rotation_angle = np.arccos(np.dot(local_v, global_v))
        transform_quaternion =  transforms3d.quaternions.axangle2quat(axis_of_rotation, rotation_angle)
    
    return transform_quaternion/np.linalg.norm(transform_quaternion)


def rotate_by_quaternion(transformQuaternion, vectorOrQuaternion):
    # Rotate a vector by a quaternion following right-hand rule convention
    # Returns a QUATERNION, not a vector
    # quaternion = [w, x, y, z]
    # vectorOrQuaternion = [x, y, z] or [w, x, y, z]
    # result = quaternion * vector * quaternion_conjugate
    # quaternion_conjugate = [w, -x, -y, -z]

    # Check if the input is a quaternion or a vector and convert to quaternion if it is a vector
    if (len(vectorOrQuaternion) < 3) or (len(vectorOrQuaternion) > 4):
        raise ValueError("The input must have either 3 or 4 elements.")
    elif len(vectorOrQuaternion) == 3:
        quaternion = np.array([0, vectorOrQuaternion[0], vectorOrQuaternion[1], vectorOrQuaternion[2]])
    else:
        quaternion = vectorOrQuaternion

    # Calculate the result = T*v*T-1
    Tr = transformQuaternion/np.linalg.norm(transformQuaternion) # normalize transform
    Tr_conj = np.array([Tr[0], -Tr[1], -Tr[2], -Tr[3]]) # calculate the conjugate
    result = transforms3d.quaternions.qmult(Tr, quaternion)
    result = transforms3d.quaternions.qmult(result, Tr_conj)

    return result


def rotation_matrix_to_quaternion(rotation_matrix):
    # Converts a 3x3 rotation matrix to a quaternion [w, x, y, z].
    # Based on methodology presented in "Introduction to Robotics: Mechanics and Control" by John J. Craig

    m = rotation_matrix
    sum_of_diagonals = np.trace(m)
    
    if sum_of_diagonals > 0:
        scaling_factor = np.sqrt(sum_of_diagonals + 1.0) * 2
        qw =                0.25 * scaling_factor
        qx = (m[2, 1] - m[1, 2]) / scaling_factor
        qy = (m[0, 2] - m[2, 0]) / scaling_factor
        qz = (m[1, 0] - m[0, 1]) / scaling_factor
    
    else:
        if (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]): # top-left index is greatest trace contributor
            scaling_factor = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            qw = (m[2, 1] - m[1, 2]) / scaling_factor
            qx =                0.25 * scaling_factor
            qy = (m[0, 1] + m[1, 0]) / scaling_factor
            qz = (m[0, 2] + m[2, 0]) / scaling_factor
        elif m[1, 1] > m[2, 2]: # middle index is greatest trace contributor
            scaling_factor = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            qw = (m[0, 2] - m[2, 0]) / scaling_factor
            qx = (m[0, 1] + m[1, 0]) / scaling_factor
            qy =                0.25 * scaling_factor
            qz = (m[1, 2] + m[2, 1]) / scaling_factor
        else: # bottom-right index is greatest trace contributor
            scaling_factor = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            qw = (m[1, 0] - m[0, 1]) / scaling_factor
            qx = (m[0, 2] + m[2, 0]) / scaling_factor
            qy = (m[1, 2] + m[2, 1]) / scaling_factor
            qz =                0.25 * scaling_factor
    
    return np.array([qw, qx, qy, qz])


def TransformStamped_from_homogeneous_matrix(matrix, child_frame_id, parent_frame_id, time):
    translation = matrix[0:3,3]
    orientation = transforms3d.quaternions.mat2quat(matrix[:3,:3]) # rotation_matrix_to_quaternion(matrix[:3,:3])
    ts = TransformStamped_from_orientation_and_translation(orientation, translation, child_frame_id, parent_frame_id, time)

    return ts


def TransformStamped_from_orientation_and_translation(orientation, translation, child_frame_id, parent_frame_id, time):
    ts = TransformStamped()
    ts.header.stamp = time
    ts.header.frame_id = parent_frame_id
    ts.child_frame_id = child_frame_id
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = translation
    ts.transform.rotation.w, ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z = orientation

    return ts


def find_quaternion_transform_from_local_to_global_z_axis(local_quaternion):
    # Find transform T to local_quaternion from global_z = np.array([0,0,0,1])
    # multiplying by the inverse, Ts=(T[0],-T[1:end]), will transform from local to global_z
    # T*global = local 
    # Ts*local = global
    # quaternion multiplication of q1 * q2 = [[r1*r2 - np.dot(v1,v2)],[r1*v2 - r2*v1 + np.cross(v1,v2)]]
    # np.cross(v1,v2) = [y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - y1*x2]
    # In this case,  q1 = T, q2 = global_quaternion (0,0,0,1), and the result q1*q2 = local_quaternion
    # Transforms3d notation: T*global = local = transforms3d.quaternions.qmult(T,global)
    
    # Return normalized transform
    return np.array([
        local_quaternion[3],
        -local_quaternion[2],
        local_quaternion[1],
        -local_quaternion[0]
    ])/np.linalg.norm(local_quaternion)


def realsense_to_enu(realsense_triad):
    # Convert from realsense coordinate system to ENU
    # Realsense: x = right, y = down, z = forward
    # ENU: x = forward, y = left, z = up
    return ROTATION.dot(realsense_triad) # This .dot is normal matrix multiplication

# Main function

def main(args=None):
    rclpy.init(args=args)
    camera_message_filter = CameraMessageFilter()
    spinMultipleThreads(camera_message_filter)
    camera_message_filter.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()