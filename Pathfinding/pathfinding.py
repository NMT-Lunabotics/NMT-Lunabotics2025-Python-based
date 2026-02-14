import math

class PathPlanner:
    def __init__(self, safety_distance=20):
        # Safety and motion parameters
        self.safety_distance = safety_distance
        self.long_distance = safety_distance * 2
        self.close_distance = safety_distance / 2
        self.step_size = 1.0

        # Internal state
        self.state = "IDLE"
        self.heading = None
        self.prev_pos = None
        self.last_clear_angle = None
        self.obstacle_side = None  # "left" or "right"

    # ------------------------------------------------------------
    # Utility functions
    # ------------------------------------------------------------
    def _angle_to_goal(self, pos, goal):
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        return math.atan2(dy, dx)

    def _angle_diff(self, a, b):
        diff = (a - b + math.pi) % (2 * math.pi) - math.pi
        return diff

    def _distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _is_obstacle_near(self, robot_position, environment_map, distance):
        """Return True if any obstacle pixel is within given radius."""
        for angle in range(0, 360, 10):
            check_x = int(robot_position[0] + distance * math.cos(math.radians(angle)))
            check_y = int(robot_position[1] - distance * math.sin(math.radians(angle)))
            if not (0 <= check_x < environment_map.get_width() and 0 <= check_y < environment_map.get_height()):
                continue
            color = environment_map.get_at((check_x, check_y))
            if sum(color[:3]) < 200:
                return True
        return False

    # ------------------------------------------------------------
    # Core heading computation
    # ------------------------------------------------------------
    def compute_heading(self, robot_position, goal_position, sensor_data, environment_map):
        """
        Main function called by main loop.
        Returns: (dx, dy, should_stop)
        """

        # --- fail-safe initialization ---
        if goal_position is None:
            # No goal clicked yet
            return 0.0, 0.0, True

        # If this is the very first call, set heading directly toward goal
        if self.heading is None:
            self.heading = self._angle_to_goal(robot_position, goal_position)
            self.prev_pos = tuple(robot_position)

        # If robot is already at goal
        dist_to_goal = math.hypot(goal_position[0] - robot_position[0],
                                  goal_position[1] - robot_position[1])
        if dist_to_goal < 5:
            self.state = "IDLE"
            return 0.0, 0.0, True

        # --------------------------------------------------------
        # Detect obstacles
        # --------------------------------------------------------
        obstacle_close = self._is_obstacle_near(robot_position, environment_map, self.close_distance)
        obstacle_near = self._is_obstacle_near(robot_position, environment_map, self.safety_distance)
        obstacle_long = self._is_obstacle_near(robot_position, environment_map, self.long_distance)

        # --------------------------------------------------------
        # State transitions
        # --------------------------------------------------------
        if obstacle_close:
            self.state = "AVOID_CLOSE"
        elif obstacle_near:
            self.state = "AVOID_SAFETY"
        elif obstacle_long:
            self.state = "AVOID_LONG"
        else:
            self.state = "GO_TO_GOAL"

        # --------------------------------------------------------
        # Behavior based on current state
        # --------------------------------------------------------
        if self.state == "GO_TO_GOAL":
            target_angle = self._angle_to_goal(robot_position, goal_position)
            self.heading = target_angle
            dx = math.cos(target_angle)
            dy = math.sin(target_angle)
            should_stop = False

        elif self.state == "AVOID_LONG":
            # Strafe slightly away from nearest obstacle direction
            # Heuristic: turn 30 degrees to whichever side is more open
            left_clear = not self._is_obstacle_near(robot_position, environment_map, self.safety_distance * 1.2)
            right_clear = not self._is_obstacle_near(
                (robot_position[0] + 10, robot_position[1]),
                environment_map,
                self.safety_distance * 1.2
            )

            if left_clear and not right_clear:
                turn = math.radians(30)
            elif right_clear and not left_clear:
                turn = -math.radians(30)
            else:
                turn = math.radians(30)

            self.heading += turn
            dx = math.cos(self.heading)
            dy = math.sin(self.heading)
            should_stop = False

        elif self.state == "AVOID_CLOSE":
            # Rotate in place to clear immediate obstacle
            self.heading += math.radians(45)
            dx, dy = 0.0, 0.0
            should_stop = False

        elif self.state == "AVOID_SAFETY":
            # Reverse briefly then adjust heading
            dx = -math.cos(self.heading)
            dy = -math.sin(self.heading)
            should_stop = False

        else:
            # Default fallback
            dx, dy, should_stop = 0.0, 0.0, True

        # --------------------------------------------------------
        # Safety fallback: ensure nonzero motion
        # --------------------------------------------------------
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            move_angle = self._angle_to_goal(robot_position, goal_position)
            dx, dy = math.cos(move_angle), math.sin(move_angle)

        return dx, dy, should_stop
