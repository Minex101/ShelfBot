import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Subscribe to the camera and get the camera intrinsic parameters
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscrbe to the camera and get the image feed
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )

        # Publisher to publish the detected ArUco marker pose
        self.pose_publisher = self.create_publisher(
            PoseStamped, 
            '/aruco_pose',
            10
        )

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50) # Select the ArUco dictionary to use
        self.aruco_params = cv2.aruco.DetectorParameters_create() # Create the ArUco detector parameters

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.15

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('⚠️ Camera intrinsic parameters not yet received.')
            return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Convert the ROS frame message to an OpenCV frame
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert the frame to grayscale
        corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, self.aruco_dict, parameters=self.aruco_params) # Detect the ArUco markers in the frame
        if ids is None:
            return  # No markers detected

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id != 0:  # Only process marker ID 0
                continue
        
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
            ) # Estimate the pose of the detected marker

            pose = PoseStamped()
            pose.header = msg.header

            # Translation Vector (tvec)
            pose.pose.position.x = tvec[0][0][0]
            pose.pose.position.y = tvec[0][0][1]
            pose.pose.position.z = tvec[0][0][2]

            # Rotation Vector (rvec) to Quaternion
            R = cv2.Rodrigues(rvec[0])[0] # Convert the rotation vector to a rotation matrix
            quaternion = self.rotation_matrix_to_quaternion(R) # Convert the rotation matrix to a quaternion
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(pose)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        trace = np.trace(R) # Calculate the trace of the rotation matrix

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s

        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s

        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s

        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s

            z = 0.25 * s

        return [x, y, z, w]

def main(args=None):

    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)

if __name__ == '__main__':
    main()