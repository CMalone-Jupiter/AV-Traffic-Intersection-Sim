import pygame
import random
import math
from matplotlib.path import Path
import config
import numpy as np

def travel_time(distance, v0, a, vmax):
    """
    Calculate the time it takes for a car to travel a given distance
    with an initial velocity v0, constant acceleration a, and a maximum velocity vmax.

    Parameters:
        distance (float): distance to travel (m)
        v0 (float): initial velocity (m/s)
        a (float): acceleration (m/s^2)
        vmax (float): maximum velocity (m/s)

    Returns:
        float: total time to travel the distance (s)
    """
    # Distance required to reach vmax
    d_accel = (vmax**2 - v0**2) / (2 * a)

    if distance <= d_accel:
        # Case 1: does not reach vmax
        vf = math.sqrt(v0**2 + 2 * a * distance)
        t = (vf - v0) / a
    else:
        # Case 2: reaches vmax, then cruises
        t_accel = (vmax - v0) / a
        d_cruise = distance - d_accel
        t_cruise = d_cruise / vmax
        t = t_accel + t_cruise

    return t


def should_av_go_col_zone(cross_traffic, av, blocker):
    """
    Original collision zone decision function
    Uses temporal overlap detection
    
    Args:
        cross_traffic: List of cross-traffic vehicles
        av: Autonomous vehicle object
        blocker: Blocking object or list of blocking objects
    """
    # Ensure blocker is handled correctly for list compatibility
    blockers = blocker if isinstance(blocker, list) else ([blocker] if blocker is not None else [])
    
    for car in cross_traffic:
        if not is_car_in_fov(car, av, blockers):
            continue  # Ignore cars outside FOV

        if car.direction == 'left':
            if car.drive_path != 'left' and (car.x+config.CAR_WIDTH) > config.LOWER_CONFLICT_ZONE[0]:
                car_t1 = travel_time(abs(car.x-config.LOWER_CONFLICT_ZONE[1]), config.CROSS_SPEED, abs(car.acceleration), config.CROSS_SPEED)
                car_t2 = travel_time(abs((car.x+config.CAR_WIDTH)-config.LOWER_CONFLICT_ZONE[0]), config.CROSS_SPEED, abs(car.acceleration), config.CROSS_SPEED)

                if av.col_zone_times[0,0] < car_t2 and car_t1 < av.col_zone_times[0,1]:
                    return False
        else:
            if car.drive_path != 'left' and car.x < config.UPPER_CONFLICT_ZONE[1]:
                car_t1 = travel_time(abs((car.x+config.CAR_WIDTH)-config.UPPER_CONFLICT_ZONE[0]), config.CROSS_SPEED, abs(car.acceleration), config.CROSS_SPEED)
                car_t2 = travel_time(abs(car.x-config.UPPER_CONFLICT_ZONE[1]), config.CROSS_SPEED, abs(car.acceleration), config.CROSS_SPEED)

                if av.col_zone_times[1,0] < car_t2 and car_t1 < av.col_zone_times[1,1]:
                    return False

    return True


def should_av_go(cross_traffic, av, blocker):
    """
    Original spatial decision function
    """
    req_dist_upper = 2*config.LANE_WIDTH+40+config.AV_HEIGHT
    req_time_upper = time_to_cover_distance(req_dist_upper, av.acceleration, config.AV_SPEED)
 
    req_dist_lower = config.LANE_WIDTH+40+config.AV_HEIGHT

    if av.intended_direction == 'right':
        req_dist_lower += config.AV_HEIGHT//2

    req_time_lower = time_to_cover_distance(req_dist_lower, av.acceleration, config.AV_SPEED)

    req_space_lower = req_time_lower*config.CROSS_SPEED
    req_space_upper = req_time_upper*config.CROSS_SPEED

    visible_range_lower = abs((av.rect.left + config.AV_WIDTH) - visible_x_range_at_y(av, blocker, config.HEIGHT/2+config.LANE_WIDTH/2)[1])
    visible_range_upper = abs(av.rect.left - visible_x_range_at_y(av, blocker, config.HEIGHT/2-config.LANE_WIDTH/2)[0])

    if av.inch_behave:
        if visible_range_lower < req_space_lower or visible_range_upper < req_space_upper:
            if not av.inching:
                print("[DECISION] FOV is too reduced to move! Starting to inch forward.")
                av.inching = True
            return False
        else:
            if av.inching:
                print("[DECISION] FOV is sufficient, stopping inching behaviour.")
                av.inching = False
 
    for car in cross_traffic:
 
        if not is_car_in_fov(car, av, blocker):
            continue  # Ignore cars outside FOV

        if car.drive_path == 'right' and car.turn_stage == 1:
            return False
        
        if car.turn_stage > 1:
            continue
        
        if car.vx == 0 and abs(car.vy) > config.CROSS_SPEED//2:
            continue

        if car.direction == 'right':
            x_diff = av.x - (car.x+config.CAR_WIDTH)
        else:
            x_diff = car.x - (av.x+config.AV_WIDTH)
 
        if x_diff >= 0:
            if car.direction == 'left' and abs(x_diff) < req_space_lower + 1:
                return False
            elif av.intended_direction != 'left':
                if car.direction == 'right' and abs(x_diff) < req_space_upper + 1:
                    return False
            
        if config.WIDTH//2-config.CAR_WIDTH < car.x < config.WIDTH//2+config.CAR_WIDTH:
            return False
       
    return True


def draw_roads(screen):
    """Draw the road layout"""
    screen.fill(config.GRAY)
 
    # === ROAD SURFACES ===
    road_width = 3 * config.LANE_WIDTH
    pygame.draw.rect(screen, config.ROAD, (config.ROAD_CENTER - road_width // 2, 0, road_width, config.HEIGHT))
    pygame.draw.rect(screen, config.ROAD, (0, config.HEIGHT // 2 - 50, config.WIDTH, 100))
 
    # Intersection box
    pygame.draw.rect(screen, config.BLACK, config.INTERSECTION_BOX, 2)

    # Draw conflict zones
    alpha = 128
    rect_surface = pygame.Surface((config.LANE_WIDTH, config.LANE_WIDTH), pygame.SRCALPHA)
    rect_surface.fill((*config.ORANGE, alpha))
    screen.blit(rect_surface, (config.LOWER_CONFLICT_ZONE[0], config.LOWER_CONFLICT_ZONE[2]))
    pygame.draw.rect(screen, config.DARK_ORANGE, (*(config.LOWER_CONFLICT_ZONE[0], config.LOWER_CONFLICT_ZONE[2]), *(config.LANE_WIDTH, config.LANE_WIDTH)), width=2)
    screen.blit(rect_surface, (config.UPPER_CONFLICT_ZONE[0], config.UPPER_CONFLICT_ZONE[2]))
    pygame.draw.rect(screen, config.DARK_ORANGE, (*(config.UPPER_CONFLICT_ZONE[0], config.UPPER_CONFLICT_ZONE[2]), *(config.LANE_WIDTH, config.LANE_WIDTH)), width=2)
 
    # Dashed lane dividers (vertical)
    for y in range(0, config.HEIGHT, 40):
        pygame.draw.line(screen, config.LANE, (config.NB_LEFT_CENTER + config.LANE_WIDTH // 2, y), (config.NB_LEFT_CENTER + config.LANE_WIDTH // 2, y + 20), 2)
        pygame.draw.line(screen, config.LANE, (config.NB_RIGHT_CENTER + config.LANE_WIDTH // 2, 0), (config.NB_RIGHT_CENTER + config.LANE_WIDTH // 2, config.HEIGHT), 2)
 
    # Dashed horizontal lane dividers
    for x in range(0, config.WIDTH, 40):
        pygame.draw.line(screen, config.LANE, (x, config.HEIGHT // 2), (x + 20, config.HEIGHT // 2), 2)
 
    # === STOP LINES (staggered) ===
    stop_y_left = config.HEIGHT // 2 + 50
    pygame.draw.line(screen, config.WHITE,
        (config.NB_LEFT_CENTER - config.LANE_WIDTH // 2 + 5, stop_y_left),
        (config.NB_LEFT_CENTER + config.LANE_WIDTH // 2 - 5, stop_y_left),
        3
    )
 
    stop_y_right = config.HEIGHT // 2 + 90
    pygame.draw.line(screen, config.WHITE,
        (config.NB_RIGHT_CENTER - config.LANE_WIDTH // 2 + 5, stop_y_right),
        (config.NB_RIGHT_CENTER + config.LANE_WIDTH // 2 - 5, stop_y_right),
        3
    )
 

def time_to_cover_distance(distance, acceleration=0.05, max_velocity=3.0):
    """Calculate time to cover distance with acceleration"""
    t_accel = max_velocity / acceleration
    s_accel = 0.5 * acceleration * t_accel**2

    if distance <= s_accel:
        t_total = math.sqrt(2 * distance / acceleration)
    else:
        s_cruise = distance - s_accel
        t_cruise = s_cruise / max_velocity
        t_total = t_accel + t_cruise

    return t_total
 

def is_car_in_fov(car, av, blocker):
    """Check if car is within AV's field of view"""
    fov_triangle = av.get_fov_polygon(blocker)
    fov_path = Path(fov_triangle)
 
    corners = [
        car.rect.topleft,
        car.rect.topright,
        car.rect.bottomleft,
        car.rect.bottomright
    ]
 
    return any(fov_path.contains_point(corner) for corner in corners)


def visible_x_range_at_y(av, blockers, target_y):
    """
    Compute visible x-range at a given horizontal line y = target_y.
    Returns (min_x, max_x) that the AV can see along that line.
    """
    polygon = av.get_fov_polygon(blockers=blockers)
    intersections = []

    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]

        x1, y1 = p1
        x2, y2 = p2

        if (y1 < target_y and y2 < target_y) or (y1 > target_y and y2 > target_y):
            continue
        if y1 == y2:
            continue

        t = (target_y - y1) / (y2 - y1)
        x_at_y = x1 + t * (x2 - x1)
        intersections.append(x_at_y)

    if not intersections:
        return None

    intersections.sort()
    return (intersections[0], intersections[-1])


def should_av_go_integrated(cross_traffic, av, blockers, pomdp_agent=None):
    """
    INTEGRATED DECISION FUNCTION - POMDP + Collision Zone
    
    Combines:
    - POMDP belief updates for occlusion handling
    - Collision zone temporal conflict detection
    - FOV-aware observation model
    - Creeping behavior for information gathering
    
    Args:
        cross_traffic: List of cross-traffic vehicles
        av: Autonomous vehicle object
        blockers: List of blocking objects (can be single object or list)
        pomdp_agent: POMDPAgent instance (will create if None)
    
    Returns:
        bool: True if AV should proceed, False otherwise
    """
    try:
        from pomdp_unseen_cars_blocked_area import should_av_go_pomdp
        return should_av_go_pomdp(cross_traffic, av, blockers, pomdp_agent)
    except ImportError as e:
        print(f"[ERROR] Could not import POMDP module: {e}")
        print("[FALLBACK] Using collision zone method")
        # Ensure blockers is a list for collision zone method
        if not isinstance(blockers, list):
            blockers = [blockers] if blockers is not None else []
        return should_av_go_col_zone(cross_traffic, av, blockers)
    except Exception as e:
        print(f"[ERROR] POMDP integration error: {e}")
        print("[FALLBACK] Using collision zone method")
        # Ensure blockers is a list for collision zone method
        if not isinstance(blockers, list):
            blockers = [blockers] if blockers is not None else []
        return should_av_go_col_zone(cross_traffic, av, blockers)