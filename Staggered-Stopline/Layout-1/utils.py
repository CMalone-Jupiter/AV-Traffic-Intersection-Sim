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

    for car in cross_traffic:
        if not is_car_in_fov(car, av, blocker):
            continue  # Ignore cars outside FOV

        # if car.drive_path == 'right' and car.turn_stage == 1: #car.is_in_intersection() car.direction_int[car.direction]*(car.x-config.WIDTH//2)
        #     # print('[TURN STATUS] Not turning because cross traffic is in turn')
        #     return False
        
        # # if car.direction_int[car.direction]*(config.WIDTH//2-car.x) > -1*config.AV_WIDTH:
        # #     return False
        # if car.turn_stage > 1:
        #     continue
        
        # if car.vx == 0 and abs(car.vy) > config.CROSS_SPEED//2:
        #     continue
    
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
    req_dist_upper = 2*config.LANE_WIDTH+40+config.AV_HEIGHT # (av.y + AV_HEIGHT) - HEIGHT
    # req_time_upper = (req_dist_upper)/AV_SPEED
    req_time_upper = time_to_cover_distance(req_dist_upper, av.acceleration, config.AV_SPEED)
 
    req_dist_lower = config.LANE_WIDTH+40+config.AV_HEIGHT # (av.y + AV_HEIGHT) - HEIGHT

    if av.intended_direction == 'right':
        req_dist_lower += config.AV_HEIGHT//2

    # req_time_lower = (req_dist_lower)/AV_SPEED
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

        # req_space_lower = min(req_time_lower*config.CROSS_SPEED, (req_time_lower*car.vx)+(0.5*car.acceleration*req_time_lower**2))
        # req_space_upper = min(req_time_upper*config.CROSS_SPEED, (req_time_upper*car.vx)+(0.5*car.acceleration*req_time_upper**2)) #req_time_upper math.sqrt(car.vx**2+car.vy**2)
 
        if not is_car_in_fov(car, av, blocker):
            continue  # Ignore cars outside FOV

        if car.drive_path == 'right' and car.turn_stage == 1: #car.is_in_intersection() car.direction_int[car.direction]*(car.x-config.WIDTH//2)
            # print('[TURN STATUS] Not turning because cross traffic is in turn')
            return False
        
        # if car.direction_int[car.direction]*(config.WIDTH//2-car.x) > -1*config.AV_WIDTH:
        #     return False
        if car.turn_stage > 1:
            continue
        
        if car.vx == 0 and abs(car.vy) > config.CROSS_SPEED//2:
            continue

        if car.direction == 'right':
            x_diff = av.x - (car.x+config.CAR_WIDTH)
        else:
            x_diff = car.x - (av.x+config.AV_WIDTH)
 
        # if car.is_in_intersection(): # or abs(car.x - av.x) < req_space + AV_WIDTH:
        #     return False
        if x_diff >= 0:
            if car.direction == 'left' and abs(x_diff) < req_space_lower + 1:
                # print('[TURN STATUS] Not turning because left moving traffic')
                return False
            elif av.intended_direction != 'left':
                if car.direction == 'right' and abs(x_diff) < req_space_upper + 1:
                    # print('[TURN STATUS] Not turning because right moving traffic')
                    # print(f"[TURN STATUS] car at x: {car.x}, y: {car.y}")
                    return False
            
        if config.WIDTH//2-config.CAR_WIDTH < car.x < config.WIDTH//2+config.CAR_WIDTH:
            return False
       
    return True

def should_av_go_probabilistic(cross_traffic, av, blocker, threshold=0.9, num_samples=100):
    """
    Probabilistic decision on whether the AV should cross the intersection.
    Uses a Monte Carlo / HMM-style uncertainty model for cross-traffic predictions.
    
    Args:
        cross_traffic: list of cars
        av: autonomous vehicle object
        blocker: blocking object for FOV
        threshold: probability threshold for safe crossing
        num_samples: number of Monte Carlo samples
    """

    req_dist_upper = 2*config.LANE_WIDTH + 40 + config.AV_HEIGHT
    req_time_upper = time_to_cover_distance(req_dist_upper, av.acceleration, config.AV_SPEED)

    req_dist_lower = config.LANE_WIDTH + 40 + config.AV_HEIGHT
    if av.intended_direction == 'right':
        req_dist_lower += config.AV_HEIGHT // 2
    req_time_lower = time_to_cover_distance(req_dist_lower, av.acceleration, config.AV_SPEED)

    # Estimate required clearance in space
    req_space_lower = req_time_lower * config.CROSS_SPEED
    req_space_upper = req_time_upper * config.CROSS_SPEED

    safe_count = 0

    for _ in range(num_samples):
        safe = True

        for car in cross_traffic:
            if not is_car_in_fov(car, av, blocker):
                continue

            if car.turn_stage > 1:
                continue

            # --- Probabilistic velocity / acceleration model ---
            # Assume some uncertainty in observed velocity/acceleration
            vx = random.gauss(car.vx, config.CROSS_SPEED/10)  # mean=car.vx, std=0.5 m/s
            ax = random.gauss(car.acceleration, 0.05)  # mean=car.accel, std=0.2 m/s²

            # Predict position after req_time
            future_x_lower = car.x + vx*req_time_lower + 0.5*ax*req_time_lower**2
            future_x_upper = car.x + vx*req_time_upper + 0.5*ax*req_time_upper**2

            if car.direction == 'left':
                x_diff = av.x - future_x_lower
                if 0 <= x_diff < req_space_lower:
                    safe = False
            elif car.direction == 'right':
                x_diff = future_x_upper - av.x
                if av.intended_direction != 'left' and 0 <= x_diff < req_space_upper:
                    safe = False

            # If car occupies intersection center, treat as unsafe
            if config.WIDTH//2 - config.CAR_WIDTH < car.x < config.WIDTH//2 + config.CAR_WIDTH:
                safe = False

            if not safe:
                break

        if safe:
            safe_count += 1

    # Compute probability of safe crossing
    p_safe = safe_count / num_samples
    return p_safe > threshold

def gaussian_prob(x, mean, std):
    """Probability density of x under Gaussian(mean, std)."""
    return (1.0 / (std * np.sqrt(2*np.pi))) * np.exp(-0.5 * ((x - mean)/std)**2)

def should_av_go_hmm(cross_traffic, av, blocker, threshold=0.9):
    """
    HMM-based version of AV crossing decision.
    Explicitly models hidden states (braking, cruising, accelerating).
    """

    # --- Required clearance for AV ---
    req_dist_upper = 2*config.LANE_WIDTH + 40 + config.AV_HEIGHT
    req_time_upper = time_to_cover_distance(req_dist_upper, av.acceleration, config.AV_SPEED)

    req_dist_lower = config.LANE_WIDTH + 40 + config.AV_HEIGHT
    if av.intended_direction == 'right':
        req_dist_lower += config.AV_HEIGHT // 2
    req_time_lower = time_to_cover_distance(req_dist_lower, av.acceleration, config.AV_SPEED)

    req_space_lower = req_time_lower * config.CROSS_SPEED
    req_space_upper = req_time_upper * config.CROSS_SPEED

    # --- HMM setup ---
    hidden_states = ["braking", "cruising", "accelerating"]
    transitions = {
        "braking":      {"braking": 0.7, "cruising": 0.2, "accelerating": 0.1},
        "cruising":     {"braking": 0.1, "cruising": 0.7, "accelerating": 0.2},
        "accelerating": {"braking": 0.1, "cruising": 0.3, "accelerating": 0.6},
    }
    emissions = {
        "braking":      {"mean": -2.0, "std": 1.0},
        "cruising":     {"mean":  0.0, "std": 0.5},
        "accelerating": {"mean":  1.5, "std": 1.0},
    }

    # --- Probability of safety across all cars ---
    p_safe_total = 1.0

    for car in cross_traffic:
        if not is_car_in_fov(car, av, blocker):
            continue
        if car.turn_stage > 1:
            continue

        # --- Observation: acceleration ---
        obs_acc = car.acceleration

        # Compute posterior over hidden states given obs_acc
        state_probs = {}
        norm_factor = 0
        for s in hidden_states:
            likelihood = gaussian_prob(obs_acc, emissions[s]["mean"], emissions[s]["std"])
            # Assume uniform prior over states for simplicity
            state_probs[s] = likelihood
            norm_factor += likelihood
        for s in hidden_states:
            state_probs[s] /= (norm_factor + 1e-9)  # normalize

        # --- Predict collision probability ---
        p_collision = 0.0
        for s, prob_s in state_probs.items():
            # Adjust predicted vx based on state
            if s == "braking":
                pred_vx = max(0, car.vx - 1.0)  # slow down
            elif s == "accelerating":
                pred_vx = car.vx + 1.0
            else:
                pred_vx = car.vx

            # Future positions
            future_x_lower = car.x + pred_vx*req_time_lower
            future_x_upper = car.x + pred_vx*req_time_upper

            # Collision check (similar to deterministic version)
            if car.direction == 'left':
                x_diff = av.x - future_x_lower
                if 0 <= x_diff < req_space_lower:
                    p_collision += prob_s
            elif car.direction == 'right':
                x_diff = future_x_upper - av.x
                if av.intended_direction != 'left' and 0 <= x_diff < req_space_upper:
                    p_collision += prob_s

            if config.WIDTH//2 - config.CAR_WIDTH < car.x < config.WIDTH//2 + config.CAR_WIDTH:
                p_collision += prob_s

        # Probability that this car is safe = (1 - p_collision)
        p_safe_total *= (1 - p_collision)

    # Decision
    return p_safe_total > threshold


def should_av_go_with_unseen(cross_traffic, av, blocker, threshold=0.9, p_exist=0.3):
    """
    HMM-based AV crossing decision with consideration for unseen cars 
    at the edge of the FOV.
    
    Args:
        cross_traffic: observed cars
        av: autonomous vehicle
        blocker: blocking object
        threshold: minimum probability of safety required
        p_exist: prior probability that an unseen car exists at each FOV edge
    """

    # --- Required clearance for AV ---
    req_dist_upper = 2*config.LANE_WIDTH + 40 + config.AV_HEIGHT
    req_time_upper = time_to_cover_distance(req_dist_upper, av.acceleration, config.AV_SPEED)

    req_dist_lower = config.LANE_WIDTH + 40 + config.AV_HEIGHT
    if av.intended_direction == 'right':
        req_dist_lower += config.AV_HEIGHT // 2
    req_time_lower = time_to_cover_distance(req_dist_lower, av.acceleration, config.AV_SPEED)

    req_space_lower = req_time_lower * config.CROSS_SPEED
    req_space_upper = req_time_upper * config.CROSS_SPEED

    # --- Probability of safety across all cars (observed + unseen) ---
    p_safe_total = 1.0

    # --- First handle observed cars (like HMM version, but simpler here) ---
    for car in cross_traffic:
        if not is_car_in_fov(car, av, blocker):
            continue
        if car.turn_stage > 1:
            continue

        # Approximate probability of collision from this car
        pred_vx = car.vx
        future_x_lower = car.x + pred_vx * req_time_lower
        future_x_upper = car.x + pred_vx * req_time_upper

        p_collision = 0.0
        if car.direction == 'left':
            x_diff = av.x - future_x_lower
            if 0 <= x_diff < req_space_lower:
                p_collision = 1.0
        elif car.direction == 'right':
            x_diff = future_x_upper - av.x
            if av.intended_direction != 'left' and 0 <= x_diff < req_space_upper:
                p_collision = 1.0

        if config.WIDTH//2 - config.CAR_WIDTH < car.x < config.WIDTH//2 + config.CAR_WIDTH:
            p_collision = 1.0

        p_safe_total *= (1 - p_collision)

    # --- Now consider hypothetical unseen cars at FOV edges ---
    fov_edges = get_fov_edges(av)  # e.g., returns list of entry points [("left", x), ("right", x)]
    
    for edge in fov_edges:
        # Assume with probability p_exist, a car is entering unseen
        # And if it exists, it may collide with the AV
        if edge[0] == "left":
            # A car could enter from left at CROSS_SPEED
            hypothetical_x = edge[1]
            future_x_lower = hypothetical_x + config.CROSS_SPEED * req_time_lower
            x_diff = av.x - future_x_lower
            p_collision = 1.0 if (0 <= x_diff < req_space_lower) else 0.0
        elif edge[0] == "right":
            hypothetical_x = edge[1]
            future_x_upper = hypothetical_x + config.CROSS_SPEED * req_time_upper
            x_diff = future_x_upper - av.x
            p_collision = 1.0 if (av.intended_direction != 'left' and 0 <= x_diff < req_space_upper) else 0.0
        else:
            p_collision = 0.0

        # Collision probability weighted by existence prior
        p_safe_total *= (1 - p_exist * p_collision)

    return p_safe_total > threshold


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

    # Draw cross (horizontal and vertical lines)
    # cross_size = 5
    # pygame.draw.line(screen, config.WHITE, (config.WIDTH//2 - cross_size, config.HEIGHT//2), (config.WIDTH//2 + cross_size, config.HEIGHT//2), 2)
    # pygame.draw.line(screen, config.WHITE, (config.WIDTH//2, config.HEIGHT//2 - cross_size), (config.WIDTH//2, config.HEIGHT//2 + cross_size), 2)

    # Draw conflict zones
    # Create a surface with alpha for transparency
    alpha = 128
    rect_surface = pygame.Surface((config.LANE_WIDTH, config.LANE_WIDTH), pygame.SRCALPHA)  # allow alpha
    rect_surface.fill((*config.ORANGE, alpha))  # RGBA
    screen.blit(rect_surface, (config.LOWER_CONFLICT_ZONE[0], config.LOWER_CONFLICT_ZONE[2]))
    pygame.draw.rect(screen, config.DARK_ORANGE, (*(config.LOWER_CONFLICT_ZONE[0], config.LOWER_CONFLICT_ZONE[2]), *(config.LANE_WIDTH, config.LANE_WIDTH)), width=2)
    screen.blit(rect_surface, (config.UPPER_CONFLICT_ZONE[0], config.UPPER_CONFLICT_ZONE[2]))
    pygame.draw.rect(screen, config.DARK_ORANGE, (*(config.UPPER_CONFLICT_ZONE[0], config.UPPER_CONFLICT_ZONE[2]), *(config.LANE_WIDTH, config.LANE_WIDTH)), width=2)
 
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

def visible_x_range_at_y(av, blockers, target_y):
    """
    Compute visible x-range at a given horizontal line y = target_y.
    Returns (min_x, max_x) that the AV can see along that line.
    """
    polygon = av.get_fov_polygon(blockers=blockers)
    intersections = []

    # Loop over polygon edges
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]

        x1, y1 = p1
        x2, y2 = p2

        # Skip if edge does not cross horizontal line
        if (y1 < target_y and y2 < target_y) or (y1 > target_y and y2 > target_y):
            continue
        if y1 == y2:  # horizontal edge
            continue

        # Linear interpolation to find x at target_y
        t = (target_y - y1) / (y2 - y1)
        x_at_y = x1 + t * (x2 - x1)
        intersections.append(x_at_y)

    if not intersections:
        return None  # nothing visible at this line

    intersections.sort()
    return (intersections[0], intersections[-1])  # min and max visible x