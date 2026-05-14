#!/usr/bin/env python3

import math
import random
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Quaternion


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class GPExplorer(Node):
    def __init__(self):
        super().__init__('gp_explorer')

        # ---- Parameters ----
        self.declare_parameter('gp_costmap_topic', '/gp_costmap')
        self.declare_parameter('nav_action_name', '/navigate_to_pose')

        # Frames
        self.declare_parameter('global_frame', 'map')      # Nav2 global frame
        self.declare_parameter('local_frame', 'odom')      # gp_costmap frame
        self.declare_parameter('base_frame', 'body')       # your base frame (body or base_link)

        # Sampling / behavior
        self.declare_parameter('fov_deg', 140.0)           # forward arc in local frame
        self.declare_parameter('min_goal_dist', 1.0)       # meters
        self.declare_parameter('max_goal_dist', 4.0)       # meters
        self.declare_parameter('num_candidates', 120)
        self.declare_parameter('patch_radius_cells', 2)    # averaging radius in cells
        self.declare_parameter('unknown_is_bad', True)     # treat -1 as invalid

        # Cost thresholds
        self.declare_parameter('max_cell_cost', 90)        # reject candidates if patch avg exceeds this
        self.declare_parameter('min_known_fraction', 0.7)  # how much of patch must be known cells

        # Timing
        self.declare_parameter('goal_cooldown_s', 2.0)
        self.declare_parameter('tf_timeout_s', 0.2)

        # Anti-thrash / stability
        self.declare_parameter('min_score_improve', 5.0)      # require improvement vs last goal score
        self.declare_parameter('min_goal_separation', 0.75)   # meters; don't send a new goal too close
        self.declare_parameter('min_robot_move_m', 0.20)      # meters; require robot to move before new goal

        self.current_goal_xy = None
        self.declare_parameter('goal_change_dist', 0.75)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Nav2 action client ----
        action_name = self.get_parameter('nav_action_name').value
        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        # ---- Costmap subscription ----
        topic = self.get_parameter('gp_costmap_topic').value
        self.costmap: Optional[OccupancyGrid] = None
        self.sub = self.create_subscription(OccupancyGrid, topic, self.costmap_cb, 10)

        # ---- State ----
        self.active_goal = False
        self.last_goal_time = self.get_clock().now()

        self._last_goal_xy = None      # (x,y) in odom
        self._last_goal_score = None   # float
        self._last_robot_xy = None     # (x,y) in odom when last goal was sent

        self.get_logger().info(f"Subscribed to {topic}, Nav2 action {action_name}")



        # main loop timer
        self.timer = self.create_timer(0.5, self.tick)

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    # ---------------- Core loop ----------------
    def tick(self):
        if self.costmap is None:
            return

        # Don’t spam goals
        cooldown = float(self.get_parameter('goal_cooldown_s').value)
        if (self.get_clock().now() - self.last_goal_time) < Duration(seconds=cooldown):
            return

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Nav2 NavigateToPose action not available yet")
            return

        if self.active_goal:
            return

        best = self.pick_best_goal_local()
        if best is None:
            self.get_logger().warn("No valid GP goal found (all candidates too costly/unknown)")
            self.last_goal_time = self.get_clock().now()
            return

        gx_odom, gy_odom, gyaw, score = best

        # ---- Anti-thrash: require meaningful improvement over last goal ----
        min_improve = float(self.get_parameter('min_score_improve').value)
        if self._last_goal_score is not None:
            # lower score is better; require score to be at least (min_improve) better
            if score > (self._last_goal_score - min_improve):
                self.last_goal_time = self.get_clock().now()
                return

        # ---- Anti-thrash: don’t send a new goal too close to last goal ----
        min_sep = float(self.get_parameter('min_goal_separation').value)
        if self._last_goal_xy is not None:
            lx, ly = self._last_goal_xy
            if (gx_odom - lx) ** 2 + (gy_odom - ly) ** 2 < (min_sep * min_sep):
                self.last_goal_time = self.get_clock().now()
                return

        # ---- Anti-thrash: require robot to have moved since last goal ----
        # (prevents repeated goals while the robot is stuck or oscillating in place)
        if self._last_robot_xy is not None:
            rx, ry = self._last_robot_xy
            min_move = float(self.get_parameter('min_robot_move_m').value)
            # current robot pose is needed; use TF local_frame<-base_frame (latest)
            base_frame = self.get_parameter('base_frame').value
            local_frame = self.get_parameter('local_frame').value
            tf_timeout = float(self.get_parameter('tf_timeout_s').value)
            try:
                tf_now = self.tf_buffer.lookup_transform(
                    local_frame,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=tf_timeout),
                )
                curx = tf_now.transform.translation.x
                cury = tf_now.transform.translation.y
                if (curx - rx) ** 2 + (cury - ry) ** 2 < (min_move * min_move):
                    self.last_goal_time = self.get_clock().now()
                    return
            except Exception:
                # If TF fails here, don’t block exploration; just proceed
                pass

        goal_map = self.transform_goal_to_global(gx_odom, gy_odom, gyaw)


        if goal_map is None:
            self.get_logger().warn("TF failed; cannot transform goal to global frame")
            self.last_goal_time = self.get_clock().now()
            return

        # Save goal state (for hysteresis / spacing)
        self._last_goal_xy = (gx_odom, gy_odom)
        self._last_goal_score = float(score)

        # Save robot position at time of goal (used by min_robot_move_m)
        try:
            base_frame = self.get_parameter('base_frame').value
            local_frame = self.get_parameter('local_frame').value
            tf_timeout = float(self.get_parameter('tf_timeout_s').value)
            tf_now = self.tf_buffer.lookup_transform(
                local_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout),
            )
            self._last_robot_xy = (tf_now.transform.translation.x, tf_now.transform.translation.y)
        except Exception:
            self._last_robot_xy = None
        min_change = float(self.get_parameter('goal_change_dist').value)

        if self.current_goal_xy is not None:
            dx = goal_map.pose.position.x - self.current_goal_xy[0]
            dy = goal_map.pose.position.y - self.current_goal_xy[1]
            if math.hypot(dx, dy) < min_change:
                return


        self.send_nav_goal(goal_map)
        self.current_goal_xy = (
            goal_map.pose.position.x,
            goal_map.pose.position.y
        )




    # ---------------- Goal selection ----------------
    def pick_best_goal_local(self) -> Optional[Tuple[float, float, float, float]]:


        cm = self.costmap
        assert cm is not None

        # robot pose in local frame (odom)
        base_frame = self.get_parameter('base_frame').value
        local_frame = self.get_parameter('local_frame').value
        tf_timeout = float(self.get_parameter('tf_timeout_s').value)

        try:
            tf = self.tf_buffer.lookup_transform(
                local_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {local_frame}<-{base_frame}: {e}")
            return None

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        fov_deg = float(self.get_parameter('fov_deg').value)
        half = math.radians(fov_deg) * 0.5
        rmin = float(self.get_parameter('min_goal_dist').value)
        rmax = float(self.get_parameter('max_goal_dist').value)
        n = int(self.get_parameter('num_candidates').value)

        best_score = 1e9
        best_goal = None

        for i in range(n):
            ang = -half + (2.0 * half) * (float(i) / float(n - 1))
            dist = random.uniform(rmin, rmax)

            gx = rx + dist * math.cos(yaw + ang)
            gy = ry + dist * math.sin(yaw + ang)
            gyaw = yaw + ang  # face roughly toward the goal direction

            score = self.score_goal_on_costmap(gx, gy)
            if score is None:
                continue

            if score < best_score:
                best_score = score
                best_goal = (gx, gy, gyaw)

        if best_goal is not None:
            self.get_logger().info(
                f"Selected GP goal (odom): x={best_goal[0]:.2f} y={best_goal[1]:.2f} score={best_score:.1f}"
            )
            return (best_goal[0], best_goal[1], best_goal[2], float(best_score))
        return None



    def score_goal_on_costmap(self, x: float, y: float) -> Optional[float]:
        cm = self.costmap
        assert cm is not None

        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        w = cm.info.width
        h = cm.info.height

        # world -> cell
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return None

        r = int(self.get_parameter('patch_radius_cells').value)
        unknown_is_bad = bool(self.get_parameter('unknown_is_bad').value)
        max_cell_cost = int(self.get_parameter('max_cell_cost').value)
        min_known_frac = float(self.get_parameter('min_known_fraction').value)

        # average cost in patch
        total = 0.0
        count = 0
        known = 0

        for dy in range(-r, r + 1):
            yy = cy + dy
            if yy < 0 or yy >= h:
                continue
            for dx in range(-r, r + 1):
                xx = cx + dx
                if xx < 0 or xx >= w:
                    continue
                idx = yy * w + xx
                c = int(cm.data[idx])

                count += 1
                if c >= 0:
                    known += 1
                    total += float(c)
                else:
                    if unknown_is_bad:
                        return None  # reject immediately

        if count == 0:
            return None

        known_frac = known / float(count)
        if known_frac < min_known_frac:
            return None

        avg = total / float(max(known, 1))
        if avg > max_cell_cost:
            return None

        # score = avg cost (lower is better)
        return avg

    # ---------------- TF transform to global frame ----------------
    def transform_goal_to_global(self, x_local: float, y_local: float, yaw_local: float) -> Optional[PoseStamped]:
        global_frame = self.get_parameter('global_frame').value
        local_frame = self.get_parameter('local_frame').value
        tf_timeout = float(self.get_parameter('tf_timeout_s').value)

        # We want map pose. Use latest TF map<-odom.
        try:
            tf = self.tf_buffer.lookup_transform(
                global_frame,
                local_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {global_frame}<-{local_frame}: {e}")
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        map_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # rotate+translate point
        c = math.cos(map_yaw)
        s = math.sin(map_yaw)
        x_map = tx + (c * x_local - s * y_local)
        y_map = ty + (s * x_local + c * y_local)
        yaw_map = map_yaw + yaw_local

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = global_frame
        ps.pose.position.x = float(x_map)
        ps.pose.position.y = float(y_map)
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(yaw_map)
        return ps

    # ---------------- Nav2 action ----------------
    def send_nav_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.active_goal = True
        self.last_goal_time = self.get_clock().now()

        self.get_logger().info(f"Sending goal in {pose.header.frame_id}: x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}")

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.active_goal = False
            self.last_goal_time = self.get_clock().now()
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f"Goal finished with status={status}")
        self.active_goal = False
        self.last_goal_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = GPExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safely tear down even if context was already shutdown elsewhere
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()







