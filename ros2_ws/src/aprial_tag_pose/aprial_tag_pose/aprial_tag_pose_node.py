#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aprial_tag_pose.msg import Pose 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np #math and data types
import pyrealsense2 as rs #important for the camera
import math

# -------------------- Settings --------------------
# Camera settings
framerate=60                                # Frame rate of camera stream
resolution={"x":640,"y":480}

# Marker settings
tag_type = cv2.aruco.DICT_6X6_250           # Change to match used tag libray
markerLength = 0.269                        # Size of markers in m
map_size = 750                              # Displayed map size
scale = 25                                  # Density of pixles per m
history_length = 10                         # Histroy length used for smoothing
max_pos_jump = 0.15                         # Max position jump per single frame
alpha_x, alpha_z, alpha_yaw = 0.1, 0.2, 0.4

# -------------------- Camera setup --------------------
camMatrix = np.array([[390.70924213, 0., 320.5518661],[0., 391.15479783, 239.81762185],[0., 0., 1.]])
distCoeffs = np.array([[-0.04335213, 0.0489175, 0.00186086, 0.00056816, 0.03206625]])

dictionary = cv2.aruco.getPredefinedDictionary(tag_type)
parameters = cv2.aruco.DetectorParameters_create()

pipe = rs.pipeline() #establishes camera operations
cfg = rs.config()
cfg.enable_stream(rs.stream.color, resolution["x"], resolution["y"], rs.format.bgr8, framerate)
cfg.enable_stream(rs.stream.depth, resolution["x"], resolution["y"], rs.format.z16, framerate)
pipe.start(cfg)
align = rs.align(rs.stream.color)

spatial = rs.spatial_filter()
temporal = rs.temporal_filter() #the fourth dimension

# -------------------- Utility Class --------------------
class AprialTagPose:
    def __init__(self):
        self.prev_X = 0.0
        self.prev_Z = 0.0
        self.prev_yaw = None
        self.pos_history = np.zeros((history_length, 2), dtype=np.float32)
        self.pos_index = 0
        self.pos_count = 0
        self.base_map = np.ones((map_size, map_size, 3), dtype=np.uint8) * 255
        cv2.circle(self.base_map, (map_size // 2, map_size // 2), 5, (255, 0, 0), -1)

    #limits angles above 180 and below -180
    @staticmethod
    def unwrap_angle(prev_angle, new_angle):
        diff = new_angle - prev_angle
        if diff > math.pi:
            new_angle -= 2 * math.pi
        elif diff < -math.pi:
            new_angle += 2 * math.pi
        return new_angle

    #tries to filter out noise
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

    #draws the display grid on the map
    @staticmethod
    def draw_grid(img, scale):
        h, w, _ = img.shape
        for x in range(0, w, scale):
            cv2.line(img, (x, 0), (x, h), (220, 220, 220), 1)
        for y in range(0, h, scale):
            cv2.line(img, (0, y), (w, y), (220, 220, 220), 1)

    #draws the camera arrow on the map
    @staticmethod
    def draw_camera(img, cam_x, cam_z, yaw=0, scale=50, size=10):
        h, w, _ = img.shape
        ix = int(cam_x * scale + w // 2)
        iy = int(-cam_z * scale + h // 2)
        tip_x = int(ix + size * math.sin(-yaw))
        tip_y = int(iy - size * math.cos(-yaw))
        cv2.arrowedLine(img, (ix, iy), (tip_x, tip_y), (0, 255, 0), 2, tipLength=0.4)
        cv2.circle(img, (ix, iy), 3, (0, 0, 255), -1)

    #draws where it thinks the robot is located, at the same angle as the servo
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

    # -------------------- Frame Handling --------------------
    #picks up frames for the program to read
    @staticmethod
    def get_frames():
        frames = pipe.poll_for_frames()
        if not frames:
            return None, None
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not color_frame or not depth_frame:
            return None, None
        try:
            depth_frame = spatial.process(depth_frame)
            depth_frame = temporal.process(depth_frame)
        except Exception as e:
            print("Depth filter error:", e)
        img = np.asanyarray(color_frame.get_data())
        return img, depth_frame

    #code for detecting and returning tags
    @staticmethod
    def detect_markers(img):
        corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
        return corners, ids

    #Takes data and returns a position
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
    def visualize(self, X, Z, yaw, rgb_img, depth_frame, rgb_camera, depth_camera, pose_map):
        map_vis = self.base_map.copy()
        self.draw_grid(map_vis, scale)
        self.draw_robot_orientation(map_vis, X, Z, box_size=30)
        self.draw_camera(map_vis, X, Z, yaw=yaw, scale=scale, size=10)

        info_text = f"Pos: ({X:.2f}, {Z:.2f}) m | Yaw: {math.degrees(yaw):.1f}"
        cv2.putText(map_vis, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(map_vis, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        if rgb_camera: cv2.imshow("Camera RGB Vision", rgb_img)

        if depth_camera and depth_frame is not None:
            depth_vis = np.asanyarray(depth_frame.get_data())
            depth_vis = cv2.convertScaleAbs(depth_vis, alpha=0.03)  # scale factor may need adjustment
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow("Camera Depth Vision", depth_vis)

        if pose_map: cv2.imshow("Pose Map", map_vis)
        cv2.waitKey(1)

# -------------------- ROS Node --------------------
class AprialTagPoseNode(Node):
    def __init__(self):
        super().__init__('aprial_tag_pose_node')
        self.ser = AprialTagPose()
        self.timer = self.create_timer(1/framerate, self.timer_callback)
        self.pose_pub = self.create_publisher(Pose, '/aprial_tag/pose', 10)
        self.declare_parameter('visual_display', False)
        self.declare_parameter('rgb_camera', True)
        self.declare_parameter('depth_camera', True)
        self.declare_parameter('pose_map', True)

        self.visual_display = self.get_parameter('visual_display').value
        self.rgb_camera = self.get_parameter('rgb_camera').value
        self.depth_camera = self.get_parameter('depth_camera').value
        self.pose_map = self.get_parameter('pose_map').value

        # Create camera topic for camera stream
        self.image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.bridge = CvBridge()

    def timer_callback(self):
        img, depth = self.ser.get_frames()
        if img is None:
            return
        
        # Publish raw camera stream to camera topic
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_pub.publish(image_msg)

        corners, ids = self.ser.detect_markers(img)
        X, Z, yaw = self.ser.estimate_pose(corners, ids, img)
        if(self.visual_display): self.ser.visualize(X, Z, yaw, img, depth, self.rgb_camera, self.depth_camera, self.pose_map)

        msg = Pose()
        msg.distance = math.sqrt(float(X)**2 + float(Z)**2)
        msg.angle = float(yaw)
        if ids is not None and len(ids) > 0:
            tag_id = int(ids[0][0])
            tag_id = max(-128, min(127, tag_id))
            msg.id = tag_id
        else:
            msg.id = -1 
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprialTagPoseNode()
    try:
        rclpy.spin(node)
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()