import pygame
import config


class StationaryVehicle:
    def __init__(self, screen):
        self.x = config.NB_LEFT_CENTER - config.AV_WIDTH // 2  # Centered in left NB lane
        stop_line_y = config.HEIGHT // 2 + 50           # Front aligns with closer stop line
        self.y = stop_line_y
        self.rect = pygame.Rect(self.x, self.y, config.AV_WIDTH, config.AV_HEIGHT)
        self.screen = screen
 
    def draw(self):
        pygame.draw.rect(self.screen, config.RED, self.rect)  # Green vehicle
 