import math
import pygame
#hello

class buildEnvironment:
    def __init__(self, MapDimensions):
        pygame.init()
        self.pointcloud = []
        self.externalMap = pygame.image.load('Map.png')
        self.maph, self.mapw = MapDimensions
        self.MapWindowName = 'LIDAR Simulation'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap, (0, 0))

        # Colors
        self.black = (0, 0, 0)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)

    def AD2pos(self, distance, angle, robotPosition):
        x = distance * math.cos(angle) + robotPosition[0]
        y = -distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))

    def dataStorage(self, data):
        if not data:
            return
        for element in data:
            point = self.AD2pos(element[0], element[1], element[2])
            if point not in self.pointcloud:
                self.pointcloud.append(point)

    def show_sensorData(self):
        self.infomap = self.map.copy()
        for point in self.pointcloud:
            self.infomap.set_at((int(point[0]), int(point[1])), self.red)

    def draw_robot(self, position):
        pygame.draw.circle(self.infomap, self.green, (int(position[0]), int(position[1])), 3)

    def draw_goal(self, position):
        pygame.draw.circle(self.infomap, (0, 0, 255), (int(position[0]), int(position[1])), 5)
