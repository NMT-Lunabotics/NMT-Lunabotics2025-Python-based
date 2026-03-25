import env
import sensors
import pathfinding
import pygame

environment = env.buildEnvironment((864, 1536))
environment.originalMap = environment.map.copy()
laser = sensors.LaserSensor(200, environment.originalMap, uncertainty=(0.5, 0.01))
planner = pathfinding.PathPlanner(safety_distance=20)

environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()

# Starting position: middle of bottom-right quarter
robot_position = [1152, 648]

goal_position = None
speed = 1
running = True

print("Simulation started.")
print("Type target relative coordinates (x y) in pixels, e.g., '100 -50' moves 100px right, 50px up.")
print("Close the window or press Ctrl+C in terminal to quit.\n")

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Handle keyboard input for goal coordinates ---
    if goal_position is None:
        # Get user input directly from console
        try:
            coords = input("Enter relative goal coordinates (x y): ")
            if not coords.strip():
                continue
            dx_input, dy_input = map(float, coords.split())
            # Compute absolute goal position based on robot’s current position
            goal_position = [
                robot_position[0] + dx_input,
                robot_position[1] + dy_input
            ]
            environment.draw_goal(goal_position)
            print(f"Goal set to: {goal_position}")
        except ValueError:
            print("Invalid input. Please enter two numbers, e.g., 200 -100.")
            continue
        except EOFError:
            break  # gracefully exit if user ends input (Ctrl+Z or Ctrl+D)

    # --- Lidar sensing and movement ---
    if goal_position:
        sensor_data = laser.sense_obstacles()
        dx, dy, should_stop = planner.compute_heading(
            robot_position, goal_position, sensor_data, environment.originalMap
        )

        # Move only if not too close to wall
        if not should_stop:
            robot_position[0] += dx * speed
            robot_position[1] += dy * speed

        laser.position = robot_position
        environment.dataStorage(sensor_data)
        environment.show_sensorData()
        environment.draw_robot(robot_position)

        # Check goal reached (within 5 pixels)
        if ((robot_position[0] - goal_position[0]) ** 2 +
            (robot_position[1] - goal_position[1]) ** 2) ** 0.5 < 5:
            print("Goal reached!\n")
            goal_position = None  # Reset for next command

    environment.map.blit(environment.infomap, (0, 0))
    pygame.display.update()
