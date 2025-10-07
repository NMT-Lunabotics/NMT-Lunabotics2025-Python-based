import math

class PathFinder:
    def __init__(self, safety_distance=20):
        self.safety_distance = safety_distance
        self.avoiding = False
        self.avoid_direction = 0  # radians

    def get_safety_distance(self):
        return self.safety_distance

    def compute_heading(self, robot_pos, goal_pos, sensor_data):
        """
        Decide heading for the robot.
        - Normally: move toward goal
        - If obstacle within safety distance: steer aside
        - Exception: if goal itself is within safety distance, go straight unless blocked
        """
        dx = goal_pos[0] - robot_pos[0]
        dy = goal_pos[1] - robot_pos[1]
        direct_angle = math.atan2(dy, dx)
        dist_to_goal = math.hypot(dx, dy)

        # --- CASE 1: No sensor data ---
        if not sensor_data:
            return dx, dy

        # --- CASE 2: Is goal itself within safety distance? ---
        goal_is_close = dist_to_goal <= self.safety_distance

        # Check for nearby obstacles
        too_close = []
        for d, angle, _ in sensor_data:
            if d <= self.safety_distance:
                too_close.append(angle)

        if too_close:
            # If goal is also close, check if obstacle is blocking line to goal
            if goal_is_close:
                if self._goal_blocked(sensor_data, robot_pos, goal_pos):
                    # Goal behind wall → avoid
                    self.avoiding = True
                else:
                    # Goal is close and reachable → ignore avoidance
                    self.avoiding = False
                    return dx, dy
            else:
                # Regular case: avoid
                self.avoiding = True

            if self.avoiding:
                avg_blocked = sum(too_close) / len(too_close)
                # steer left or right
                if -math.pi/2 < avg_blocked < math.pi/2:  # obstacle in front
                    self.avoid_direction = direct_angle + math.pi/2
                else:
                    self.avoid_direction = direct_angle - math.pi/2
                return math.cos(self.avoid_direction), math.sin(self.avoid_direction)

        # --- CASE 3: No avoidance needed ---
        self.avoiding = False
        return dx, dy

    def _goal_blocked(self, sensor_data, robot_pos, goal_pos):
        """
        Check if there's an obstacle between robot and goal.
        We approximate: if lidar detects something closer than the goal
        along roughly the same angle, then the goal is blocked.
        """
        dx = goal_pos[0] - robot_pos[0]
        dy = goal_pos[1] - robot_pos[1]
        goal_angle = math.atan2(dy, dx)
        goal_dist = math.hypot(dx, dy)

        for d, angle, _ in sensor_data:
            # If obstacle angle is close to goal angle and closer than goal → blocked
            if abs(angle - goal_angle) < math.radians(10) and d < goal_dist:
                return True
        return False
