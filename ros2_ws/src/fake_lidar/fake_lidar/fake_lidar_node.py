#!/usr/bin/env python3

import os
os.environ["GDK_DISABLE_SHM"] = "1"
import  math, cv2, time, random
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from aprial_tag_pose.msg import Pose
from serial_command.msg import Command 
from math import sin, cos
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar_node')
        # Enable ui
        self.enable_visual = True

        # FPS
        self.last_time = cv2.getTickCount()
        self.fps = 0

        # Calulate map dimension of aerna in pixel format
        self.map_width_px = 1029
        self.map_height_px = 751
        self.map_width_m = 6.88
        self.map_height_m = 6.0
        self.px_per_m_x = self.map_width_px / self.map_width_m
        self.px_per_m_y = self.map_height_px / self.map_height_m

        # Record robots position, angle, and size, with set inital conditions
        self.x=200
        self.y=600
        self.yaw = random.uniform(0, 2*math.pi)
        self.robot_width = 3

        # Set lidar simulator settings
        self.lidar_range=7.5
        self.lidar_rays=360
        self.depthcamera_angle=0 #-pi/2 to pi/2

        # Variables used to store data during simulation
        self.v_left=0
        self.v_right=0
        self.last_time=0
        self.tags=None
        self.hit_tags=None

        self.pivot_x=0
        self.pivot_y=0
        self.points=[]
    
        # Load map
        script_dir = os.path.dirname(os.path.realpath(__file__))
        map_path = os.path.join(script_dir, 'comp_map_marker2.png')
        self.map = cv2.imread(map_path, cv2.IMREAD_COLOR)
        self.height, self.width, _ = self.map.shape

        # Real lidar and aprial tag simulations are very system exspensive so precalulate wall locations
        img = self.map.astype(np.uint8)

        mask = np.mean(img, axis=2) < 128
        coords = np.argwhere(mask).astype(np.float32)
        colors = img[mask]

        ids = np.zeros(len(colors), dtype=np.float32)
        ids[(colors[:,1] > colors[:,0]) & (colors[:,1] > colors[:,2])] = 2  # green → 2
        ids[(colors[:,0] > colors[:,1]) & (colors[:,0] > colors[:,2])] = 3  # blue → 3
        ids[(colors[:,2] > colors[:,0]) & (colors[:,2] > colors[:,1])] = 1  # red → 1


        self.wall_coords = np.column_stack((coords, ids))



        # Set aprial tag simular settings
        self.min_tag_visible_pixels = 3
        self.tag_max_distance = 2.0
        self.tag_max_angle = math.radians(30)


        # Publishers that data is pushed to, used to fake real data
        self.pub_lidar = self.create_publisher(LaserScan, 'scan', 10)
        self.pub_tags = self.create_publisher(Pose, 'aprial_tag/pose', 10)
        #self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # Subscriber used to lision to incomming serial commands so that simulator can match real robot
        self.sub_serial = self.create_subscription(Command,'/serial/writer',self.handle_serial_command,10)

        # update rates of diffrent simulator aspects
        self.timer = self.create_timer(0.3, self.publish_all)
        self.robot_timer = self.create_timer(0.01, lambda: self.update_pose(0.01))

        # OpenCV visualization
        if self.enable_visual:
            cv2.namedWindow('Map View', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Map View', self.map_width_px, self.map_height_px)
            cv2.createTrackbar("Yaw", "Map View", int(math.degrees(self.yaw)) % 360, 360, self.on_slider)
            cv2.createTrackbar("Camera yaw", "Map View", int(math.degrees(self.depthcamera_angle)) % 360, 90, self.on_slider2)
            cv2.setTrackbarMin('Camera yaw', 'Map View', -90)

            cv2.setMouseCallback("Map View", self.on_click)
            self.window_created = False
            self.close_window = False
            self.render_timer = self.create_timer(0.03, lambda: self.draw_map(self.tags))

    # Mouse click updates robot location
    def on_click(self, event, x, y, flags, parms):
        # For easy testing move robot based on location on screen that is clicked
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x = x
            self.y = y
        elif event==cv2.EVENT_RBUTTONDOWN: self.close_window=True
    
    # Try to force window close, doesn't seem to work always
    def check_window_close(self):
        if cv2.getWindowProperty("Map View", cv2.WND_PROP_VISIBLE) < 1 and self.window_created == True and self.close_window == True:
            print("Window closed by user, shutting down ROS...")
            rclpy.shutdown()

    # Slider callback
    def on_slider(self, val):
        # For easy rotation add a slider to rotate the robot
        self.yaw = math.radians(val)
    def on_slider2(self, val):
        # For easy rotation add a slider to rotate the robot
        self.depthcamera_angle = math.radians(val)

    # lidar scan
    def lidar_scan(self):
        # Setup additional scan settings and used variables
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / self.lidar_rays
        scan.range_min = 0.05
        scan.range_max = self.lidar_range
        scan.ranges = []
        self.hit_tags = []

        self.current_scan_hits = []

        rx, ry = self.x, self.y
        for i in range(self.lidar_rays):
            angle = scan.angle_min + i*scan.angle_increment + self.yaw
            dx, dy = math.cos(angle), -math.sin(angle)
            vecs = self.wall_coords[:, :2] - np.array([ry, rx])
            proj = vecs[:,0]*dy + vecs[:,1]*dx
            proj = np.where(proj > 0, proj, np.inf)
            perp = np.abs(vecs[:,0]*dx - vecs[:,1]*dy)
            mask = perp < 0.5
            masked_proj = proj[mask]
            if masked_proj.size > 0:
                hit_i = np.argmin(masked_proj)
                dist_px = masked_proj[hit_i]
                scan.ranges.append(dist_px / self.px_per_m_x)
                hit_x = rx + dx * dist_px
                hit_y = ry + dy * dist_px
                self.current_scan_hits.append((hit_x, hit_y))
                hit_idx = np.flatnonzero(mask)[hit_i]
                color_id = self.wall_coords[hit_idx, 2]
                if color_id != 0: self.hit_tags.append((hit_x, hit_y, int(color_id)))
            else:
                scan.ranges.append(self.lidar_range)
                self.current_scan_hits.append(None)

        self.pub_lidar.publish(scan)

    # Tag detection (pixel-based, fast & correct)
    def detect_tags(self):
        detected_tags = {}

        # Organize hits by tag color (1=red, 2=green, 3=blue)
        tag_hits = {1: [], 2: [], 3: []}
        for hx, hy, color_id in self.hit_tags:
            tag_hits[int(color_id)].append((hx, hy))

        rx, ry = self.x, self.y

        for tag_id, hits in tag_hits.items():
            if len(hits) < self.min_tag_visible_pixels:
                continue  # Not enough hits to count as seeing the tag

            # Convert to NumPy array for vectorized math
            hits_arr = np.array(hits, dtype=np.float32)

            # Compute distances (meters)
            dx_m = (hits_arr[:,0] - rx) / self.px_per_m_x
            dy_m = (ry - hits_arr[:,1]) / self.px_per_m_y  # invert y for map coords
            distances = np.hypot(dx_m, dy_m)

            # Compute relative angles to robot yaw
            angles = np.arctan2(dy_m, dx_m) - self.yaw - self.depthcamera_angle
            angles = (angles + np.pi) % (2*np.pi) - np.pi  # wrap to [-pi, pi]

            # Mask hits within distance & angle constraints
            mask = (distances <= self.tag_max_distance) & (np.abs(angles) <= self.tag_max_angle)
            if np.sum(mask) >= self.min_tag_visible_pixels:
                # Pick the closest valid hit as representative
                closest_idx = np.argmin(distances[mask])
                detected_tags[tag_id] = (
                    hits_arr[mask][closest_idx, 0],  # hx
                    hits_arr[mask][closest_idx, 1],  # hy
                    distances[mask][closest_idx],
                    angles[mask][closest_idx]
                )

        # Publish ROS messages
        if not detected_tags:
            msg = Pose()
            msg.id = -1
            msg.distance = 0.0
            msg.angle = 0.0
            self.pub_tags.publish(msg)
        else:
            for tag_id, (_, _, distance, angle_rel) in detected_tags.items():
                msg = Pose()
                msg.id = int(tag_id)
                msg.distance = float(distance)
                msg.angle = float(angle_rel)
                self.pub_tags.publish(msg)

        # Return simplified tag positions for visualization
        return {tag_id: (hx, hy) for tag_id, (hx, hy, _, _) in detected_tags.items()}

    # Draw robot and tags
    def draw_map(self, detected_tags=None):
        if not self.enable_visual: return

        vis = self.map.copy()
        ix, iy = int(self.x), int(self.y)

        rect_length = self.robot_width*2*10
        rect_width = self.robot_width*10

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        dx = rect_length / 2
        dy = rect_width / 2
        corners = [
            (int(ix + dx*c - dy*s), int(iy - dx*s - dy*c)),
            (int(ix - dx*c - dy*s), int(iy + dx*s - dy*c)),
            (int(ix - dx*c + dy*s), int(iy + dx*s + dy*c)),
            (int(ix + dx*c + dy*s), int(iy - dx*s + dy*c))
        ]
        cv2.polylines(vis, [np.array(corners)], isClosed=True, color=(0,255,0), thickness=2)

        pivot_ix, pivot_iy = int(self.pivot_x), int(self.pivot_y)
        cv2.circle(vis, (pivot_ix, pivot_iy), 5, (219, 19, 159), -1)  

        for px, py in self.points:
            cv2.circle(vis, (int(px), int(py)), 5, (0,0,255), -1)

        # Depth camera direction indicator line
        length = 20
        cv2.line(vis, (ix, iy), (int(ix+length*math.cos(self.yaw+self.depthcamera_angle)), int(iy-length*math.sin(self.yaw+self.depthcamera_angle))), (0,0,255), 2)

        if detected_tags:
            for tag_id, (tx, ty) in detected_tags.items():
                color = (128, 128, 128)  
                cv2.circle(vis, (int(tx), int(ty)), 5, color, -1)
                cv2.putText(vis, str(tag_id), (int(tx)+5, int(ty)+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


        current_time = cv2.getTickCount()
        dt = (current_time - self.last_time)/cv2.getTickFrequency()
        self.fps = 0.9*self.fps + 0.1*(1.0/dt) if dt>0 else self.fps
        self.last_time = current_time
        cv2.putText(vis, f"FPS: {self.fps:.1f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
        cv2.imshow("Map View", vis)
        self.window_created=True
        cv2.waitKey(1)

    # Publish both LiDAR and tags
    def publish_all(self):
        self.lidar_scan()
        self.tags = self.detect_tags()
        #self.publish_odom()

    # Lision to motor commands to simulate motor movment
    def handle_serial_command(self, msg):
        if msg.command == 'M':
            self.v_left = max(-30, min(30, msg.data[0]))
            self.v_right = max(-30, min(30, msg.data[1]))
            self.last_command_time = time.time()
        if msg.command == 'A':
            self.depthcamera_angle = max(-90, min(90, msg.data[0]))
            self.last_command_time = time.time()

    # Calulate the new position of robot based on the command recived
    def update_pose(self, dt):
        scale = 2.4 / 15.0
        v_l = self.v_left * scale
        v_r = self.v_right * scale
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.robot_width

        if self.v_left == 0 and self.v_right != 0:  
            self.pivot_x = int(self.x) + (self.robot_width*5)*math.sin(self.yaw)
            self.pivot_y = int(self.y) + (self.robot_width*5)*math.cos(self.yaw)
            radius = self.robot_width
            dtheta = v_r / radius * dt
            dx = self.x - self.pivot_x
            dy = self.y - self.pivot_y
            cos_d = math.cos(dtheta)
            sin_d = math.sin(dtheta)
            self.x = self.pivot_x + dx*cos_d - dy*sin_d
            self.y = self.pivot_y + dx*sin_d + dy*cos_d
            self.yaw += dtheta

        elif self.v_right == 0 and self.v_left != 0: 
            self.pivot_x = int(self.x) - (self.robot_width*5)*math.sin(self.yaw)
            self.pivot_y = int(self.y) - (self.robot_width*5)*math.cos(self.yaw)
            radius = self.robot_width
            dtheta = -v_l / radius * dt 
            dx = self.x - self.pivot_x
            dy = self.y - self.pivot_y
            cos_d = math.cos(dtheta)
            sin_d = math.sin(dtheta)
            self.x = self.pivot_x + dx*cos_d - dy*sin_d
            self.y = self.pivot_y + dx*sin_d + dy*cos_d
            self.yaw += dtheta
        else:

            v = (v_r + v_l) / 2.0
            omega = (v_r - v_l) / self.robot_width

            self.x += v * math.cos(self.yaw) * dt * self.px_per_m_x
            self.y -= v * math.sin(self.yaw) * dt * self.px_per_m_x
            self.yaw += omega * dt
            self.yaw = (self.yaw + math.pi) % (2*math.pi) - math.pi


        if self.v_right == 0:
            self.pivot_x = int(self.x)+(self.robot_width*5)*math.sin(self.yaw)
            self.pivot_y = int(self.y)+(self.robot_width*5)*math.cos(self.yaw)
        elif self.v_left == 0:
            self.pivot_x = int(self.x)-(self.robot_width*5)*math.sin(self.yaw)
            self.pivot_y = int(self.y)-(self.robot_width*5)*math.cos(self.yaw) 
        else:
            self.pivot_x = self.x
            self.pivot_y = self.y

        #self.points.append((self.x, self.y))
        #if len(self.points) > 1200:
        #    self.points.pop(0)

        if time.time() - getattr(self, 'last_command_time', 0) > 0.5:
            self.v_left = 0
            self.v_right = 0

    # Publish some fake odometry for using the fake lidar sensor
    def publish_odom(self):
            def quaternion_from_euler(roll, pitch, yaw):
                qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
                qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
                qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
                qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
                return [qx, qy, qz, qw]

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x / self.px_per_m_x
            odom.pose.pose.position.y = self.y / self.px_per_m_y
            odom.pose.pose.position.z = 0.0

            q = quaternion_from_euler(0, 0, self.yaw)
            odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            odom.pose.covariance = [0.0001]*36
            odom.twist.covariance = [0.0001]*36

            v = (self.v_left + self.v_right)/2.0 * (2.4 / 30.0)
            omega = (self.v_right - self.v_left)/self.robot_width
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = omega

            self.pub_odom.publish(odom)

def main():
    rclpy.init()
    node = FakeLidar()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        node.check_window_close()

if __name__ == "__main__":
    main()