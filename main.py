from SLAM_Simulation import env, sensors, pathfinding
import pygame
import math

# Build environment
environment = env.buildEnvironment((1349, 2048))
environment.originalMap = environment.map.copy()

# Create lidar
laser = sensors.LaserSensor(200, environment.originalMap, uncertainty=(0.5, 0.01))

# Pathfinding helper
planner = pathfinding.PathFinder(safety_distance=20)

# Clear screen for mapping
environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()

# --- ROBOT SETUP ---
robot_position = [100, 100]   # starting position
goal_queue = []               # queue of goals
speed = 2                     # pixels per frame

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Left click to add a new goal
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            goal_queue = [list(event.pos)]  # reset queue with only this new goal

    # Collect lidar data
    sensor_data = laser.sense_obstacles()
    environment.dataStorage(sensor_data)

    # --- MOVE ROBOT TOWARD CURRENT GOAL ---
    if goal_queue:
        current_goal = goal_queue[0]

        # Ask pathfinding for next heading
        heading = planner.compute_heading(robot_position, current_goal, sensor_data)
        dx, dy = heading
        dist_to_goal = math.hypot(current_goal[0] - robot_position[0],
                                  current_goal[1] - robot_position[1])

        if dist_to_goal > speed:
            norm = math.hypot(dx, dy)
            if norm > 0:
                robot_position[0] += speed * dx / norm
                robot_position[1] += speed * dy / norm
        else:
            goal_queue.pop(0)  # reached current goal

    # Update laser position
    laser.position = (int(robot_position[0]), int(robot_position[1]))

    # Show lidar data
    environment.show_sensorData()

    # Draw robot trail (persistent, stays in infomap)
    robot_pos_int = (int(robot_position[0]), int(robot_position[1]))
    pygame.draw.circle(environment.infomap, (0, 255, 0), robot_pos_int, 5)

    # Draw goals (persistent)
    for goal in goal_queue:
        pygame.draw.circle(environment.infomap, (255, 0, 0), (goal[0], goal[1]), 5)

    # Blit persistent info
    environment.map.blit(environment.infomap, (0, 0))

    # Draw safety ring (temporary, refreshes every frame)
    pygame.draw.circle(environment.map, (0, 200, 0), robot_pos_int,
                       planner.get_safety_distance(), 1)

    pygame.display.update()
