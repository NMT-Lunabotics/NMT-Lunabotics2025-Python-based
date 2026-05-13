#!/usr/bin/env python3
"""
ROS 2 node that detects AprilTags with an Intel RealSense camera and
publishes absolute pose corrections to suppress odometry / scan-matcher drift.

Architecture overview
---------------------

  /camera/color/image_raw  ──► [ArUco detector]
  /camera/color/camera_info ──► [intrinsics latch]
                                    │
                        ┌───────────▼──────────────┐
                        │  For each known tag seen  │
                        │  compute_robot_pose_in_map│
                        │  (see pose_math.py)       │
                        └───────────┬──────────────┘
                                    │
              ┌─────────────────────┼───────────────────────┐
              ▼                     ▼                        ▼
  /aruco/pose_correction    /aruco/loop_closure_correction  /aruco/debug_image
  PoseWithCovarianceStamped  PoseWithCovarianceStamped      sensor_msgs/Image
  (consumed by EKF map node) (reset scan matcher drift)     (for RViz)

EKF integration
---------------
Add the following to dual_ekf_navsat_params.yaml under ekf_filter_node_map:

    pose0: /aruco/pose_correction
    pose0_config: [true,  true,  false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_queue_size: 5
    pose0_differential: false
    pose0_relative: false
    pose0_rejection_threshold: 2.0   # Mahalanobis gating – rejects wild outliers

Parameters
----------
tag_map_file        : path to config/tag_map.yaml
marker_length       : physical tag side length in metres (default 0.269)
cam_x_offset        : camera position forward of base_link in metres
cam_y_offset        : camera position left of base_link in metres
cam_yaw_offset      : camera yaw relative to robot heading in radians
pos_sigma           : 1-sigma position uncertainty published in covariance (m)
yaw_sigma           : 1-sigma yaw uncertainty published in covariance (rad)
min_marker_area_px  : minimum marker size in pixels² – rejects tiny/far detections
max_z_distance      : maximum tag distance in metres to accept (rejects bad depth)
publish_debug_image : if True, publish annotated image on /aruco/debug_image
map_frame           : name of the map frame  (default 'map')
base_frame          : name of the robot base frame (default 'base_link')
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .pose_math import (
    build_pose_covariance,
    compute_robot_pose_in_map,
    marker_area_px,
    yaw_to_quaternion,
)
from .tag_database import TagDatabase


class ArucoLocalizerNode(Node):
    """Detects AprilTags and publishes absolute pose corrections."""

    def __init__(self):
        super().__init__('aruco_localizer')

        # Declare all parameters with sensible defaults

        self.declare_parameter('tag_map_file', '')
        self.declare_parameter('marker_length', 0.269)    # metres
        self.declare_parameter('cam_x_offset', 0.15)      # forward of base_link
        self.declare_parameter('cam_y_offset', 0.0)       # left of base_link
        self.declare_parameter('cam_yaw_offset', 0.0)     # camera yaw rel. to heading
        self.declare_parameter('pos_sigma', 0.05)         # 5 cm position uncertainty
        self.declare_parameter('yaw_sigma', 0.05)         # ~3° heading uncertainty
        self.declare_parameter('min_marker_area_px', 400.0)
        self.declare_parameter('max_z_distance', 5.0)     # metres
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        
        # Read parameters
        
        tag_file        = self.get_parameter('tag_map_file').value
        self.marker_len = self.get_parameter('marker_length').value
        self.cam_x      = self.get_parameter('cam_x_offset').value
        self.cam_y      = self.get_parameter('cam_y_offset').value
        self.cam_yaw    = self.get_parameter('cam_yaw_offset').value
        self.pos_sigma  = self.get_parameter('pos_sigma').value
        self.yaw_sigma  = self.get_parameter('yaw_sigma').value
        self.min_area   = self.get_parameter('min_marker_area_px').value
        self.max_z      = self.get_parameter('max_z_distance').value
        self.debug_pub  = self.get_parameter('publish_debug_image').value
        self.map_frame  = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        
        # Tag database
        
        self.tag_db = TagDatabase(tag_file, logger=self.get_logger())

        
        # Camera intrinsics (populated from /camera/color/camera_info once)
        
        self.cam_matrix   = None
        self.dist_coeffs  = None
        self._cam_info_received = False

        # ------------------------------------------------------------------
        # ArUco detector – same dictionary as the original script
        # ------------------------------------------------------------------
        self._aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25H9)
        self._aruco_params = cv2.aruco.DetectorParameters_create()
        # Tighten corner refinement for better sub-pixel accuracy
        self._aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.bridge = CvBridge()

        # Throttle: don't publish more often than once per N frames when
        # the same tag is continuously visible (avoids flooding the EKF).
        self._last_publish_time: dict = {}
        self._publish_min_interval = 0.5  # seconds

        # ------------------------------------------------------------------
        # QoS: sensor data – best effort, keep last
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        # CameraInfo – RELIABLE/VOLATILE to match realsense2_camera driver
        cam_info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self._camera_info_callback,
            qos_profile=cam_info_qos,
        )

        # Color image stream
        self._image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._image_callback,
            qos_profile=sensor_qos,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/aruco/pose_correction',
            10,
        )

        # Loop-closure correction for the scan matcher node
        self._loop_closure_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/aruco/loop_closure_correction',
            10,
        )

        if self.debug_pub:
            self._debug_pub = self.create_publisher(Image, '/aruco/debug_image', 5)
        else:
            self._debug_pub = None

        self.get_logger().info(
            f'ArUco Localizer ready. '
            f'Known tags: {self.tag_db.all_ids()},  '
            f'marker length: {self.marker_len}m,  '
            f'cam offset: ({self.cam_x:.3f}, {self.cam_y:.3f}) m'
        )

    # ------------------------------------------------------------------
    # Camera info callback – runs once, then effectively stops
    # ------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._cam_info_received:
            return  # already have intrinsics

        k = msg.k  # row-major 3×3 intrinsic matrix
        self.cam_matrix = np.array([
            [k[0], k[1], k[2]],
            [k[3], k[4], k[5]],
            [k[6], k[7], k[8]],
        ], dtype=np.float64)

        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)

        self._cam_info_received = True
        self.get_logger().info(
            f'Camera intrinsics received. '
            f'fx={k[0]:.2f}  fy={k[4]:.2f}  '
            f'cx={k[2]:.2f}  cy={k[5]:.2f}'
        )
        # Destroy subscription – we no longer need it
        self.destroy_subscription(self._cam_info_sub)

    # ------------------------------------------------------------------
    # Main image callback
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        # Wait for intrinsics before processing
        if not self._cam_info_received:
            return

        # Convert ROS Image to OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self._aruco_dict, parameters=self._aruco_params
        )

        if ids is None:
            if self._debug_pub:
                self._publish_debug_image(frame, msg.header)
            return

        # Estimate 3D poses for all detected markers
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_len, self.cam_matrix, self.dist_coeffs
        )

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        for i, tag_id_arr in enumerate(ids):
            tag_id = int(tag_id_arr[0])

            # Only process tags whose map position we know
            if not self.tag_db.is_known(tag_id):
                continue

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # --- Quality gates ---

            # 1. Marker must be large enough in the image (close enough / well-lit)
            area = marker_area_px(corners[i])
            if area < self.min_area:
                self.get_logger().debug(
                    f'Tag {tag_id}: area {area:.0f}px² < min {self.min_area:.0f}, skip'
                )
                continue

            # 2. Tag must not be too far away (tvec[2] = depth in camera Z)
            z_dist = float(tvec[2])
            if z_dist > self.max_z or z_dist <= 0.0:
                self.get_logger().debug(
                    f'Tag {tag_id}: depth {z_dist:.2f}m out of range, skip'
                )
                continue

            # --- Compute robot pose in map ---
            tag_x, tag_y, tag_yaw = self.tag_db.get_pose(tag_id)

            try:
                robot_x, robot_y, robot_yaw = compute_robot_pose_in_map(
                    rvec, tvec,
                    tag_x, tag_y, tag_yaw,
                    self.cam_x, self.cam_y, self.cam_yaw,
                )
            except Exception as exc:
                self.get_logger().error(
                    f'Pose computation failed for tag {tag_id}: {exc}'
                )
                continue

            # --- Throttle repeated publications of the same tag ---
            last_t = self._last_publish_time.get(tag_id, 0.0)
            if now_sec - last_t < self._publish_min_interval:
                continue
            self._last_publish_time[tag_id] = now_sec

            # --- Build and publish PoseWithCovarianceStamped ---
            pose_msg = self._build_pose_msg(
                msg.header.stamp, robot_x, robot_y, robot_yaw
            )
            self._pose_pub.publish(pose_msg)

            self.get_logger().info(
                f'Tag {tag_id}: robot pose in map = '
                f'({robot_x:.3f}, {robot_y:.3f}, '
                f'{math.degrees(robot_yaw):.1f}°)  '
                f'depth={z_dist:.2f}m  area={area:.0f}px²'
            )

            # --- Loop-closure handling ---
            if self.tag_db.is_first_detection(tag_id):
                self.tag_db.record_first_seen(tag_id, robot_x, robot_y, robot_yaw)
            else:
                correction = self.tag_db.get_loop_closure_correction(
                    tag_id, robot_x, robot_y, robot_yaw
                )
                if correction is not None:
                    dx, dy, dyaw = correction
                    drift_magnitude = math.hypot(dx, dy)
                    if drift_magnitude > 0.05:  # only publish if drift > 5 cm
                        corr_msg = self._build_correction_msg(
                            msg.header.stamp, dx, dy, dyaw
                        )
                        self._loop_closure_pub.publish(corr_msg)

            # Draw on frame for debug image
            if self._debug_pub:
                cv2.drawFrameAxes(
                    frame, self.cam_matrix, self.dist_coeffs,
                    rvec, tvec, self.marker_len * 0.5
                )
                label = (
                    f'ID:{tag_id} '
                    f'({robot_x:.2f},{robot_y:.2f}) '
                    f'{math.degrees(robot_yaw):.0f}deg'
                )
                c = corners[i][0]
                cx_px = int(c[:, 0].mean())
                cy_px = int(c[:, 1].mean())
                cv2.putText(
                    frame, label, (cx_px - 60, cy_px - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

        if self._debug_pub:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            self._publish_debug_image(frame, msg.header)

    # ------------------------------------------------------------------
    # Message builders
    # ------------------------------------------------------------------

    def _build_pose_msg(
        self,
        stamp,
        x: float,
        y: float,
        yaw: float,
    ) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.map_frame

        q = yaw_to_quaternion(yaw)
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']

        msg.pose.covariance = build_pose_covariance(self.pos_sigma, self.yaw_sigma)
        return msg

    def _build_correction_msg(
        self,
        stamp,
        dx: float,
        dy: float,
        dyaw: float,
    ) -> PoseWithCovarianceStamped:
        """
        Publish the loop-closure drift as a pose delta so the scan matcher
        node can correct its accumulated position.
        The frame_id 'loop_closure_delta' distinguishes this from absolute fixes.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'loop_closure_delta'

        q = yaw_to_quaternion(dyaw)
        msg.pose.pose.position.x    = dx
        msg.pose.pose.position.y    = dy
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']

        msg.pose.covariance = build_pose_covariance(self.pos_sigma, self.yaw_sigma)
        return msg

    def _publish_debug_image(self, frame: np.ndarray, header) -> None:
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = header
            self._debug_pub.publish(debug_msg)
        except CvBridgeError as exc:
            self.get_logger().debug(f'Debug image publish error: {exc}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()