import pygame

# Constants
HEADERLESS = True
WIDTH, HEIGHT = 800, 600
FPS = 60
CAR_WIDTH, CAR_HEIGHT = 40, 20  # Cross-traffic dimensions of about 3.2m x 1.6m
AV_WIDTH, AV_HEIGHT = 20, 40    # Controlled AV
# SPAWN_RATE = 0.03
TRAFFIC_FLOW = 6480 # Measured in vehicles/hour (converted to probability of spawning per frame ~3%)
CROSS_SPEED = 3 # equivalent to 14.4 m/s or 51 km/hr
AV_SPEED = 3 # equivalent to 14.4 m/s or 51 km/hr
INTERSECTION_BOX = pygame.Rect(WIDTH // 2 - 75, HEIGHT // 2 - 50, 150, 100)
LANE_WIDTH = 50 # Set to correspond to about 4 meters, i.e. 8cm per pixel
ROAD_CENTER = WIDTH // 2
NB_LEFT_CENTER = ROAD_CENTER - LANE_WIDTH     # Left NB lane
NB_RIGHT_CENTER = ROAD_CENTER                 # Right NB lane (for AV)
SB_CENTER = ROAD_CENTER + LANE_WIDTH          # Single southbound lane
 
# Colors
WHITE = (255, 255, 255)
ROAD = (50, 50, 50)
LANE = (200, 200, 200)
RED = (200, 50, 50)
BLUE = (50, 50, 200)
GREEN = (50, 200, 50)
YELLOW = (255, 255, 0)
GRAY = (150, 150, 150)
BLACK = (0,0,0)
DRIVE_PATH = {0 : 'straight', 1 : 'left', 2 : 'right'}