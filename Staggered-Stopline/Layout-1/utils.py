import pygame
import math
from matplotlib.path import Path
import config

def should_av_go(cross_traffic, av, blocker):
    req_dist_upper = 2*config.LANE_WIDTH+40+config.AV_HEIGHT # (av.y + AV_HEIGHT) - HEIGHT
    # req_time_upper = (req_dist_upper)/AV_SPEED
    req_time_upper = time_to_cover_distance(req_dist_upper, av.acceleration, config.AV_SPEED)
    req_space_upper = req_time_upper*config.CROSS_SPEED
 
    req_dist_lower = config.LANE_WIDTH+40+config.AV_HEIGHT # (av.y + AV_HEIGHT) - HEIGHT
    # req_time_lower = (req_dist_lower)/AV_SPEED
    req_time_lower = time_to_cover_distance(req_dist_lower, av.acceleration, config.AV_SPEED)
    req_space_lower = req_time_lower*config.CROSS_SPEED
 
    for car in cross_traffic:
 
        if not is_car_in_fov(car, av, blocker):
            continue  # Ignore cars outside FOV
 
        if car.vx > 0:
            x_diff = (car.x+config.CAR_WIDTH//2) - (av.x-config.AV_WIDTH//2)
        else:
            x_diff = (car.x-config.CAR_WIDTH//2) - (av.x+config.AV_WIDTH//2)
 
        # if car.is_in_intersection(): # or abs(car.x - av.x) < req_space + AV_WIDTH:
        #     return False
        if car.vx < 0 and x_diff > 0 and abs(x_diff) < req_space_lower + 1:
            return False
        elif av.intended_direction != 'left':
            if car.vx > 0 and x_diff < 0 and abs(x_diff) < req_space_upper + 1:
                return False
       
    return True

def draw_roads(screen):
    screen.fill(config.GRAY)
 
    # === ROAD SURFACES ===
 
    # Vertical road: now 3 lanes wide (150px)
    road_width = 3 * config.LANE_WIDTH
    pygame.draw.rect(screen, config.ROAD, (config.ROAD_CENTER - road_width // 2, 0, road_width, config.HEIGHT))
 
    # Horizontal road: still 100px tall
    pygame.draw.rect(screen, config.ROAD, (0, config.HEIGHT // 2 - 50, config.WIDTH, 100))
 
    # Intersection box
    pygame.draw.rect(screen, config.BLACK, config.INTERSECTION_BOX, 2)
 
    # Dashed lane dividers (vertical)
    for y in range(0, config.HEIGHT, 40):
        # Between NB lanes
        pygame.draw.line(screen, config.LANE, (config.NB_LEFT_CENTER + config.LANE_WIDTH // 2, y), (config.NB_LEFT_CENTER + config.LANE_WIDTH // 2, y + 20), 2)
        # Between NB and SB lanes
        # pygame.draw.line(screen, LANE, (NB_RIGHT_CENTER + LANE_WIDTH // 2, y), (NB_RIGHT_CENTER + LANE_WIDTH // 2, y + 20), 2)
        pygame.draw.line(screen, config.LANE, (config.NB_RIGHT_CENTER + config.LANE_WIDTH // 2, 0), (config.NB_RIGHT_CENTER + config.LANE_WIDTH // 2, config.HEIGHT), 2)
 
    # Dashed horizontal lane dividers
    for x in range(0, config.WIDTH, 40):
        pygame.draw.line(screen, config.LANE, (x, config.HEIGHT // 2), (x + 20, config.HEIGHT // 2), 2)
 
    # === STOP LINES (staggered) ===
 
    # Left NB lane (closer)
    stop_y_left = config.HEIGHT // 2 + 50
    pygame.draw.line(screen, config.WHITE,
        (config.NB_LEFT_CENTER - config.LANE_WIDTH // 2 + 5, stop_y_left),
        (config.NB_LEFT_CENTER + config.LANE_WIDTH // 2 - 5, stop_y_left),
        3
    )
 
    # Right NB lane (AV lane, further back)
    stop_y_right = config.HEIGHT // 2 + 90
    pygame.draw.line(screen, config.WHITE,
        (config.NB_RIGHT_CENTER - config.LANE_WIDTH // 2 + 5, stop_y_right),
        (config.NB_RIGHT_CENTER + config.LANE_WIDTH // 2 - 5, stop_y_right),
        3
    )
 
def time_to_cover_distance(distance, acceleration=0.05, max_velocity=3.0):
    # Time to reach max velocity
    t_accel = max_velocity / acceleration

    # Distance covered while accelerating to max velocity
    s_accel = 0.5 * acceleration * t_accel**2

    if distance <= s_accel:
        # Case A: Never reach max velocity
        t_total = math.sqrt(2 * distance / acceleration)
    else:
        # Case B: Accelerate, then cruise
        s_cruise = distance - s_accel
        t_cruise = s_cruise / max_velocity
        t_total = t_accel + t_cruise

    return t_total
 
def is_car_in_fov(car, av, blocker):
    fov_triangle = av.get_fov_polygon(blocker)
    fov_path = Path(fov_triangle)
 
    # Get the four corners of the car rectangle
    corners = [
        car.rect.topleft,
        car.rect.topright,
        car.rect.bottomleft,
        car.rect.bottomright
    ]
 
    # If any corner is inside the FOV triangle, return True
    return any(fov_path.contains_point(corner) for corner in corners)