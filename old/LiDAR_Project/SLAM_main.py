from SLAM_PROJECT import SLAM_env, SLAM_Sensors
import pygame
environment = SLAM_env.buildEnvironment((600,1200))
environment.originalMap = environment.map.copy()
Laser=SLAM_Sensors.Laser(200,environment.externalMap,uncertainty=(0.5,0.01))
environment.map.fill((0,0,0))
environment.infomap = environment.map.copy()

running = True
while running:
    sensorON=False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pos = pygame.mouse.get_pos()
            Laser.position = pos
        sensor_data = Laser.sense_obstacles()  # returns a list (maybe empty)
        if sensor_data:  # only store/draw when we have hits
            environment.dataStorage(sensor_data)
            environment.show_sensorData()
        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()

        if pygame.mouse.get_focused():
            sensorON=True
        elif not pygame.mouse.get_focused():
            sensorON=False
    if sensorON:
            position = pygame.mouse.get_pos()
            Laser.position = position
            sensor_data = Laser.sense_obstacles()
            environment.dataStorage(sensor_data)
            environment.show_sensorData()
    environment.map.blit(environment.infomap,(0,0))
    pygame.display.update()
