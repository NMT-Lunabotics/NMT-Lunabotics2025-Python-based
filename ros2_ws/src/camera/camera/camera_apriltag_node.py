#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np #math and data types
import math

# -------------------- Changeable Variables --------------------
tag_type = cv2.aruco.DICT_6X6_250           # Change according to which tag library
markerLength = 0.269                        # Size of marker, (meters)
map_size = 750                              # The size of the displayed maps in pixels
scale = 25                                  # How many pixels per meter
history_length = 10                         # For smoothing, how many past positions are used
max_pos_jump = 0.15                         # Max jump in a single frame
zoom=5                                      # Scaleing factor for map
# ROS settings
camera_topic = '/camera0/image_raw'         # Camera topic to liston to
enable_display=False                        # Should extra display stuff show, or only bare minimum

# -------------------- Camera setup --------------------
camMatrix = np.array([[390.70924213, 0., 320.5518661],[0., 391.15479783, 239.81762185],[0., 0., 1.]])
distCoeffs = np.array([[-0.04335213, 0.0489175, 0.00186086, 0.00056816, 0.03206625]])

dictionary = cv2.aruco.getPredefinedDictionary(tag_type)
parameters = cv2.aruco.DetectorParameters_create()
alpha_x, alpha_z, alpha_yaw = 0.1, 0.2, 0.4

# -------------------- Functioning Functions --------------------
class AprialTagPose:
    def __init__(self):
        # Map initalizer stuff
        self.prev_X = 0.0
        self.prev_Z = 0.0
        self.prev_yaw = None
        self.pos_history = np.zeros((history_length, 2), dtype=np.float32)
        self.pos_index = 0
        self.pos_count = 0
        self.base_map = np.ones((map_size, map_size, 3), dtype=np.uint8) * 255
        cv2.circle(self.base_map, (map_size // 2, map_size // 2), 5, (255, 0, 0), -1)

    # Limits angles above 180 and below -180
    @staticmethod
    def unwrap_angle(prev_angle, new_angle):
        diff = new_angle - prev_angle
        if diff > math.pi:
            new_angle -= 2 * math.pi
        elif diff < -math.pi:
            new_angle += 2 * math.pi
        return new_angle

    # Tries to filter out noise
    @staticmethod
    def filter_outlier(new_pos, pos_history, pos_index, pos_count):
        new_pos = np.array(new_pos, dtype=np.float32)
        pos_history[pos_index] = new_pos
        pos_index = (pos_index + 1) % history_length
        pos_count = min(pos_count + 1, history_length)
        median_pos = np.median(pos_history[:pos_count], axis=0)
        if np.linalg.norm(new_pos - median_pos) > max_pos_jump:
            return pos_history[pos_index - 2 if pos_index > 1 else 0], pos_history, pos_index, pos_count
        return new_pos, pos_history, pos_index, pos_count

    # Draws the display grid on the map
    @staticmethod
    def draw_grid(img, scale):
        h, w, _ = img.shape
        for x in range(0, w, scale):
            cv2.line(img, (x, 0), (x, h), (220, 220, 220), 1)
        for y in range(0, h, scale):
            cv2.line(img, (0, y), (w, y), (220, 220, 220), 1)

    # Draws the camera arrow on the map
    @staticmethod
    def draw_camera(img, cam_x, cam_z, yaw=0, scale=50, size=10):
        h, w, _ = img.shape
        ix = int(cam_x * scale * zoom + w // 2)
        iy = int(-cam_z * scale * zoom + h // 2)
        tip_x = int(ix + size * math.sin(-yaw)* zoom)
        tip_y = int(iy - size * math.cos(-yaw)* zoom)
        cv2.arrowedLine(img, (ix, iy), (tip_x, tip_y), (0, 255, 0), 2, tipLength=0.4)
        cv2.circle(img, (ix, iy), 3, (0, 0, 255), -1)

    # Draws where it thinks the robot is located, angle of robot
    @staticmethod
    def draw_robot_orientation(img, cam_x, cam_z, box_size=30):
        h, w, _ = img.shape
        cx = int(cam_x * scale + w // 2)
        cy = int(-cam_z * scale + h // 2)
        w_rect, h_rect = box_size, box_size // 2
        corners = np.array([
            [-w_rect / 2, -h_rect / 2],
            [w_rect / 2, -h_rect / 2],
            [w_rect / 2, h_rect / 2],
            [-w_rect / 2, h_rect / 2]
        ])
        rotated = corners + np.array([cx, cy])
        pts = rotated.astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(img, [pts], (0, 128, 255))
        cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)

    # Picks up frames for the program to read
    @staticmethod
    def get_frames(img):
        if img is None:
            return None
        return img

    # Code for detecting and returning tags
    @staticmethod
    def detect_markers(img):
        corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
        return corners, ids

    # Takes data and returns a position
    def estimate_pose(self, corners, ids, img):
        X, Z, yaw = self.prev_X, self.prev_Z, self.prev_yaw or 0
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs)
            rvec, tvec = rvecs[0][0], tvecs[0][0]
            R_marker, _ = cv2.Rodrigues(rvec)
            T_marker_to_cam = np.eye(4)
            T_marker_to_cam[:3, :3] = R_marker
            T_marker_to_cam[:3, 3] = tvec.flatten()
            T_cam_to_marker = np.linalg.inv(T_marker_to_cam)
            X_new, _, Z_new = T_cam_to_marker[:3, 3]
            R_cam = T_cam_to_marker[:3, :3]
            yaw_new = -math.atan2(R_cam[0, 2], R_cam[2, 2])
            if self.prev_yaw is not None:
                yaw_new = self.unwrap_angle(self.prev_yaw, yaw_new)
            filtered, self.pos_history, self.pos_index, self.pos_count = self.filter_outlier([X_new, Z_new], self.pos_history, self.pos_index, self.pos_count)
            X_smooth = alpha_x * filtered[0] + (1 - alpha_x) * self.prev_X
            Z_smooth = alpha_z * filtered[1] + (1 - alpha_z) * self.prev_Z
            x_vec, z_vec = math.cos(yaw_new), math.sin(yaw_new)
            prev_x, prev_z = math.cos(self.prev_yaw or 0), math.sin(self.prev_yaw or 0)
            x_s = alpha_yaw * x_vec + (1 - alpha_yaw) * prev_x
            z_s = alpha_yaw * z_vec + (1 - alpha_yaw) * prev_z
            yaw_smooth = math.atan2(z_s, x_s)
            self.prev_X, self.prev_Z, self.prev_yaw = X_smooth, Z_smooth, yaw_smooth
            X, Z, yaw = X_smooth, Z_smooth, yaw_smooth
        return X, Z, yaw

    # Handles windows and drawing
    def visualize(self, X, Z, yaw, rgb_img):
        map_vis = np.ones_like(rgb_img)
        self.draw_grid(map_vis, scale)
        self.draw_robot_orientation(map_vis, X, Z, box_size=30)
        self.draw_camera(map_vis, X, Z, yaw=yaw, scale=scale, size=10)
        return map_vis

# -------------------- ROS stuff --------------------
class AprialTagPoseNode(Node):
    def __init__(self):
        # Initalize classes and node
        super().__init__('camera_apriltag_node')
        self.tag_instance = AprialTagPose()
        self.bridge = CvBridge()
        self.latest_image = None

        # Setup the camera topic and pull display settings
        self.declare_parameter('camera_topic', '/camera0/image_raw')
        self.declare_parameter('publish_map', enable_display)
        self.declare_parameter('publish_processed', enable_display)
        self.camera_topic = self.get_parameter('camera_topic').value
        self.publish_map = self.get_parameter('publish_map').value
        self.publish_processed = self.get_parameter('publish_processed').value

        # Setup class publishers
        self.pose_pub = self.create_publisher(Pose, '/apriltag/pose', 10)
        if self.publish_map: self.map_pub = self.create_publisher(Image, '/apriltag/map', 10)
        if self.publish_processed: self.processed_pub = self.create_publisher(Image, '/apriltag/raw_image', 10)

        # Setup stuff for updating tag pose estimate
        self.create_subscription(Image,self.camera_topic,self.image_callback,10)
        self.timer = self.create_timer(0.01, self.timer_callback)  
        self.frame_rate = None
        self.frame_shape = None

    # Template for ros to cv2 image
    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.frame_shape is None: self.frame_shape = img.shape
        self.latest_image = img

    # Apriltag updater callback
    def timer_callback(self):
        # Setup image properties and update tag estimates
        if self.latest_image is None: return
        img = self.latest_image
        corners, ids = self.tag_instance.detect_markers(img)
        X, Z, yaw = self.tag_instance.estimate_pose(corners, ids, img)

        # Prep then post apriltag pose data
        msg = Pose()
        msg.distance = math.sqrt(float(X)**2 + float(Z)**2)
        msg.angle = float(yaw)
        if ids is not None and len(ids) > 0:
            tag_id = int(ids[0][0])
            tag_id = max(-128, min(127, tag_id))
            msg.id = tag_id
        else: msg.id = -1 
        self.pose_pub.publish(msg)

        # If enabled publish map and raw_image modifyied video streams
        if self.publish_processed:
            processed_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            processed_msg.header.stamp = self.get_clock().now().to_msg()
            self.processed_pub.publish(processed_msg)
        if self.publish_map:
            map_img = self.tag_instance.visualize(X, Z, yaw, img)
            map_msg = self.bridge.cv2_to_imgmsg(map_img, encoding='bgr8')
            map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprialTagPoseNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
