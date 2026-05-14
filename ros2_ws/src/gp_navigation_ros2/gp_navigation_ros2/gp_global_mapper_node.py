#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class GPGlobalMapper(Node):
    def __init__(self) -> None:
        super().__init__('gp_global_mapper')

        self.declare_parameter('input_topic', '/gp_costmap')
        self.declare_parameter('output_topic', '/gp_costmap_global')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('motion_gate_distance_m', 0.15)
        self.declare_parameter('motion_gate_yaw_deg', 8.0)
        self.declare_parameter('global_resolution', 0.10)
        self.declare_parameter('global_width', 400)
        self.declare_parameter('global_height', 400)
        self.declare_parameter('global_origin_x', -20.0)
        self.declare_parameter('global_origin_y', -20.0)
        self.declare_parameter('occupied_threshold', 80)
        self.declare_parameter('free_threshold', 15)
        self.declare_parameter('min_consensus', 3)
        self.declare_parameter('max_observation_count', 50)
        self.declare_parameter('use_patch_header_stamp', False)
        self.declare_parameter('tf_timeout_sec', 0.15)
        self.declare_parameter('publish_debug_logs', False)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.motion_gate_distance_m = float(self.get_parameter('motion_gate_distance_m').value)
        self.motion_gate_yaw_deg = float(self.get_parameter('motion_gate_yaw_deg').value)
        self.resolution = float(self.get_parameter('global_resolution').value)
        self.width = int(self.get_parameter('global_width').value)
        self.height = int(self.get_parameter('global_height').value)
        self.origin_x = float(self.get_parameter('global_origin_x').value)
        self.origin_y = float(self.get_parameter('global_origin_y').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.min_consensus = int(self.get_parameter('min_consensus').value)
        self.max_observation_count = int(self.get_parameter('max_observation_count').value)
        self.use_patch_header_stamp = bool(self.get_parameter('use_patch_header_stamp').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.publish_debug_logs = bool(self.get_parameter('publish_debug_logs').value)

        n_cells = self.width * self.height
        self.avg_cost: List[float] = [0.0] * n_cells
        self.obs_count: List[int] = [0] * n_cells
        self.occ_count: List[int] = [0] * n_cells
        self.free_count: List[int] = [0] * n_cells

        self.last_publish_stamp = self.get_clock().now()
        self.last_fuse_pose: Optional[Tuple[float, float, float]] = None
        self.pending_publish = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(OccupancyGrid, self.input_topic, self.grid_callback, qos)
        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, qos)

        timer_period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'gp_global_mapper started: input={self.input_topic}, output={self.output_topic}, '
            f'global_frame={self.global_frame}, size={self.width}x{self.height}, resolution={self.resolution:.3f}'
        )

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def shortest_angle_diff(a: float, b: float) -> float:
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def idx(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def should_fuse(self, x: float, y: float, yaw: float) -> bool:
        if self.last_fuse_pose is None:
            return True

        last_x, last_y, last_yaw = self.last_fuse_pose
        dist = math.hypot(x - last_x, y - last_y)
        yaw_deg = abs(math.degrees(self.shortest_angle_diff(yaw, last_yaw)))
        return dist >= self.motion_gate_distance_m or yaw_deg >= self.motion_gate_yaw_deg

    def saturating_increment(self, value: int) -> int:
        return min(value + 1, self.max_observation_count)

    def fuse_cell(self, gx: int, gy: int, cost: int) -> None:
        if not self.in_bounds(gx, gy):
            return

        i = self.idx(gx, gy)
        c = self.obs_count[i]
        self.avg_cost[i] = (self.avg_cost[i] * c + float(cost)) / float(c + 1)
        self.obs_count[i] = self.saturating_increment(c)

        if cost >= self.occupied_threshold:
            self.occ_count[i] = self.saturating_increment(self.occ_count[i])
            if self.free_count[i] > 0:
                self.free_count[i] -= 1
        elif cost <= self.free_threshold:
            self.free_count[i] = self.saturating_increment(self.free_count[i])
            if self.occ_count[i] > 0:
                self.occ_count[i] -= 1

    def lookup_patch_transform(self, msg: OccupancyGrid):
        stamp = msg.header.stamp if self.use_patch_header_stamp else rclpy.time.Time().to_msg()
        try:
            return self.tf_buffer.lookup_transform(
                self.global_frame,
                msg.header.frame_id,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed {msg.header.frame_id} -> {self.global_frame}: {exc}')
            return None

    def grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn('Received empty gp_costmap patch')
            return

        transform = self.lookup_patch_transform(msg)
        if transform is None:
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        yaw = self.yaw_from_quaternion(transform.transform.rotation)
        if not self.should_fuse(tx, ty, yaw):
            return

        fused = 0
        skipped_unknown = 0

        for v in range(msg.info.height):
            for u in range(msg.info.width):
                local_index = v * msg.info.width + u
                value = int(msg.data[local_index])
                if value < 0:
                    skipped_unknown += 1
                    continue

                x_local = msg.info.origin.position.x + (u + 0.5) * msg.info.resolution
                y_local = msg.info.origin.position.y + (v + 0.5) * msg.info.resolution

                p = PointStamped()
                p.header = msg.header
                p.point.x = x_local
                p.point.y = y_local
                p.point.z = 0.0

                try:
                    p_map = do_transform_point(p, transform)
                except Exception as exc:
                    self.get_logger().warn(f'Point transform failed: {exc}')
                    return

                gx, gy = self.world_to_grid(p_map.point.x, p_map.point.y)
                if self.in_bounds(gx, gy):
                    self.fuse_cell(gx, gy, value)
                    fused += 1

        self.last_fuse_pose = (tx, ty, yaw)
        self.pending_publish = True

        if self.publish_debug_logs:
            self.get_logger().info(
                f'Fused patch from {msg.header.frame_id}: fused={fused}, skipped_unknown={skipped_unknown}, '
                f'pose=({tx:.2f}, {ty:.2f}, yaw={math.degrees(yaw):.1f} deg)'
            )

    def build_published_grid(self) -> OccupancyGrid:
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.global_frame
        out.info.resolution = self.resolution
        out.info.width = self.width
        out.info.height = self.height
        out.info.origin.position.x = self.origin_x
        out.info.origin.position.y = self.origin_y
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0

        data: List[int] = [-1] * (self.width * self.height)
        for i in range(len(data)):
            if self.obs_count[i] == 0:
                data[i] = -1
                continue

            if self.occ_count[i] >= self.min_consensus and self.occ_count[i] > self.free_count[i]:
                data[i] = 100
            elif self.free_count[i] >= self.min_consensus and self.free_count[i] > self.occ_count[i]:
                data[i] = 0
            else:
                avg = int(round(self.avg_cost[i]))
                data[i] = max(0, min(100, avg))

        out.data = data
        return out

    def timer_callback(self) -> None:
        if not self.pending_publish:
            return

        out = self.build_published_grid()
        self.pub.publish(out)
        self.pending_publish = False
        self.last_publish_stamp = self.get_clock().now()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GPGlobalMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
