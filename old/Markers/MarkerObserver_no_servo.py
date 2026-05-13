import cv2
import numpy as np #math and data types
import pyrealsense2 as rs #important for the camera
import math
import time
import threading

# -------------------- Changeable Variables --------------------

tag_type = cv2.aruco.DICT_6X6_250 #change according to which tag library
markerLength = 0.269  # meters


sweep_step = 40 #how far the motor rotates each step of sweep
sweep_pause = 0.8 #time before sweeping

dead_zone = 40 #dead zone where it won't try to center it further
step = 3 #step size for centering the tag

allowed_angular_deviation = 10 #how much can the robot angle disagree with the targeted angle before taking action
dist_goal = 1 #how close the robot should try to get, if it is off target angle
dist_goal_aligned = 0.5 #how close the robot should try to get, if it is on target angle

map_size = 750 #the size of the displayed maps in pixels
scale = 25 #how many pixels per meter

history_length = 10 #for smoothing, how many past positions are used
max_pos_jump = 0.15 #max jump in a single frame

# -------------------- Camera setup --------------------
camMatrix = np.array([[390.70924213, 0., 320.5518661],
                      [0., 391.15479783, 239.81762185],
                      [0., 0., 1.]])
distCoeffs = np.array([[-0.04335213, 0.0489175, 0.00186086, 0.00056816, 0.03206625]])

dictionary = cv2.aruco.getPredefinedDictionary(tag_type)

parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)


pipe = rs.pipeline() #establishes camera operations
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)
align = rs.align(rs.stream.color)

spatial = rs.spatial_filter()
temporal = rs.temporal_filter() #the fourth dimension

alpha_x, alpha_z, alpha_yaw = 0.1, 0.2, 0.4
prev_X, prev_Z, prev_yaw = 0.0, 0.0, None


pos_history = np.zeros((history_length, 2), dtype=np.float32)
pos_index = 0
pos_count = 0
target = 0

#other map stuff
tag_pixel = map_size // 2
base_map = np.ones((map_size, map_size, 3), dtype=np.uint8) * 255
cv2.circle(base_map, (tag_pixel, tag_pixel), 5, (255, 0, 0), -1)


# -------------------- Functioning Functions --------------------

#limits angles above 180 and below -180
def unwrap_angle(prev_angle, new_angle): #
    diff = new_angle - prev_angle
    if diff > math.pi:
        new_angle -= 2 * math.pi
    elif diff < -math.pi:
        new_angle += 2 * math.pi
    return new_angle

#does the same thing but for degrees
def limit_angle(angle):
    if angle < -180:
        angle += 360
    elif angle > 180:
        angle -= 360
    return angle

#tries to filter out noise
def filter_outlier(new_pos):
    global pos_history, pos_index, pos_count
    new_pos = np.array(new_pos, dtype=np.float32)
    pos_history[pos_index] = new_pos
    pos_index = (pos_index + 1) % history_length
    pos_count = min(pos_count + 1, history_length)

    median_pos = np.median(pos_history[:pos_count], axis=0)
    if np.linalg.norm(new_pos - median_pos) > max_pos_jump:
        return pos_history[pos_index - 2 if pos_index > 1 else 0]
    return new_pos

#draws the display grid on the map
def draw_grid(img, scale):
    h, w, _ = img.shape
    for x in range(0, w, scale):
        cv2.line(img, (x, 0), (x, h), (220, 220, 220), 1)
    for y in range(0, h, scale):
        cv2.line(img, (0, y), (w, y), (220, 220, 220), 1)

#draws the camera arrow on the map
def draw_camera(img, cam_x, cam_z, yaw=0, scale=50, size=10):
    h, w, _ = img.shape
    ix = int(cam_x * scale + w // 2)
    iy = int(-cam_z * scale + h // 2)
    tip_x = int(ix + size * math.sin(-yaw))
    tip_y = int(iy - size * math.cos(-yaw))
    cv2.arrowedLine(img, (ix, iy), (tip_x, tip_y), (0, 255, 0), 2, tipLength=0.4)
    cv2.circle(img, (ix, iy), 3, (0, 0, 255), -1)

#draws where it thinks the robot is located, at the same angle as the servo
def draw_robot_orientation(img, cam_x, cam_z, box_size=30):
    h, w, _ = img.shape
    cx = int(cam_x * scale + w // 2)
    cy = int(-cam_z * scale + h // 2)
    w_rect, h_rect = box_size, box_size // 2
    theta = -math.radians(135)
    corners = np.array([
        [-w_rect / 2, -h_rect / 2],
        [w_rect / 2, -h_rect / 2],
        [w_rect / 2, h_rect / 2],
        [-w_rect / 2, h_rect / 2]
    ])
    R = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ])
    rotated = corners @ R.T
    rotated += np.array([cx, cy])
    pts = rotated.astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(img, [pts], (0, 128, 255))
    cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)
    arrow_length = box_size // 2
    tip_x = int(cx + arrow_length * math.cos(theta))
    tip_y = int(cy + arrow_length * math.sin(theta))
    cv2.arrowedLine(img, (cx, cy), (tip_x, tip_y), (0, 255, 0), 2, tipLength=0.4)

# -------------------- Frame Handling --------------------
#picks up frames for the program to read
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
def detect_markers(img):
    corners, ids, _ = detector.detectMarkers(img)
    return corners, ids

#Takes data and returns a position
def estimate_pose(corners, ids, prev_X, prev_Z, prev_yaw):
    X, Z, yaw = prev_X, prev_Z, prev_yaw or 0
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
        if prev_yaw is not None:
            yaw_new = unwrap_angle(prev_yaw, yaw_new)
        X_filtered, Z_filtered = filter_outlier([X_new, Z_new])
        X_smooth = alpha_x * X_filtered + (1 - alpha_x) * prev_X
        Z_smooth = alpha_z * Z_filtered + (1 - alpha_z) * prev_Z
        x_vec, z_vec = math.cos(yaw_new), math.sin(yaw_new)
        prev_x, prev_z = math.cos(prev_yaw or 0), math.sin(prev_yaw or 0)
        x_s = alpha_yaw * x_vec + (1 - alpha_yaw) * prev_x
        z_s = alpha_yaw * z_vec + (1 - alpha_yaw) * prev_z
        yaw_smooth = math.atan2(z_s, x_s)
        prev_X, prev_Z, prev_yaw = X_smooth, Z_smooth, yaw_smooth
        X, Z, yaw = X_smooth, Z_smooth, yaw_smooth
    return X, Z, yaw, prev_X, prev_Z, prev_yaw

# -------------------- Interactions --------------------
#runs when the map is clicked anywhere. Turns a clicked coord into real distances
def map_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        map_img, cam_X, cam_Z = param
        # pixels to meters
        real_X = -(x - map_size // 2) / scale
        real_Z = (y - map_size // 2) / scale
        # by cam position
        real_X += cam_X
        real_X = - real_X
        real_Z += cam_Z

        global targ_X_Dist
        targ_X_Dist = real_X
        global targ_Z_Dist
        targ_Z_Dist = real_Z
        global target
        target = True
        global angToTarget
        angToTarget = find_angle(real_X, real_Z, 0)

        print(f"Clicked Map: pixel=({x},{y}) -> real-world=({real_X:.2f}, {real_Z:.2f}) m")
        print(f"Need to rotate {angToTarget:.1f}")

#finds the target angle from the current heading to position on map
def find_angle(newX, newZ, currentAngle):
    new_angle = math.atan2(newZ, newX) * 180 / math.pi - 90
    new_angle -= currentAngle - 60
    limit_angle(new_angle)

    return new_angle

# Handles windows and drawing
def visualize(X, Z, yaw, img):
    map_vis = base_map.copy()
    draw_grid(map_vis, scale)
    draw_robot_orientation(map_vis, X, Z, box_size=30)
    draw_camera(map_vis, X, Z, yaw=yaw, scale=scale, size=10)
    info_text = f"Pos: ({X:.2f}, {Z:.2f}) m | Yaw: {math.degrees(yaw):.1f}"
    cv2.putText(map_vis, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(map_vis, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

    cv2.imshow("Camera Vision", img)
    cv2.imshow("Camera Map", map_vis)
    cv2.setMouseCallback("Camera Map", map_click, param=(map_vis, X, Z))


# -------------------- Main Loop --------------------
tick = 0
tickrate = 10 #slows down how often it checks navigation
try:
    while True:
        #main code, handles marker detection and tracking
        img, depth = get_frames()
        if img is None:
            time.sleep(0.01)
            continue
        corners, ids = detect_markers(img)
        X, Z, yaw, prev_X, prev_Z, prev_yaw = estimate_pose(corners, ids, prev_X, prev_Z, prev_yaw)
        visualize(X, Z, yaw, img)


        if target and tick%tickrate == 0:
            end = False
            distance = math.sqrt( targ_X_Dist * targ_X_Dist + targ_Z_Dist * targ_Z_Dist)
            angToTarget = find_angle(targ_X_Dist, targ_Z_Dist, 0)


            if distance < dist_goal_aligned: #If the robot is within range of the goal, end navigation
                end = True
                print("Within goal")

            elif distance < dist_goal and angToTarget < allowed_angular_deviation: #if within partial range but still on track, continue
                end = False
            elif distance < dist_goal and angToTarget > allowed_angular_deviation: #if within partial range and off track, end
                end = True
                print("Within goal with deviation")

            else :
                end = False



            if not end:
                if abs(angToTarget) < allowed_angular_deviation: #Has the correct angle to the target
                    print(f"Forward! {distance:.1f} meters")
                    #code for forwards
                else:
                    if angToTarget > 0:
                        print(f"lefty loosy {angToTarget:.1f} degrees")
                        #code for left

                    else:
                        print(f"righty tighty{angToTarget:.1f} degrees")
                        #code for right
            else:
                target = False
                end = False


        if cv2.waitKey(1) & 0xFF == 27:
            break
        time.sleep(0.01)
        tick += 1

finally:
    pipe.stop()
    cv2.destroyAllWindows()