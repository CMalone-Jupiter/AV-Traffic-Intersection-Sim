import pygame
import math
import config
import numpy as np
from utils import travel_time

class AutonomousVehicle:
    def __init__(self, screen):
        self.x = config.NB_RIGHT_CENTER - config.AV_WIDTH // 2
        self.y = config.HEIGHT // 2 + 90    # Front aligns with stop line
        self.vx = 0
        self.vy = 0  # start stationary
        self.max_vx = 0
        self.max_vy = -config.AV_SPEED  # target velocity moving up initially
        self.acceleration = 0.05  # tweak this for acceleration rate
        self.rect = pygame.Rect(self.x, self.y, config.AV_WIDTH, config.AV_HEIGHT)
        self.moving = False
        self.inching = False
        self.inch_behave = False
        self.collided = False
        self.manual_trigger = False
        self.intended_direction = 'straight'  # Options: 'left', 'straight', 'right'
        self.turn_angle = 0
        self.arc_cx = None
        self.arc_cy = None
        self.max_v_x = False
        self.max_v_y = False
        self.turn_stage = 0
        self.col_zone_times = np.array([[0,1000], [0, 1000]])
        self.screen = screen
 
    def update(self):
 
        if self.moving and not self.collided:
 
            # Accelerate vx towards max_vx
            if self.vx < self.max_vx:
                self.vx = min(self.vx + self.acceleration, self.max_vx)
            elif self.vx > self.max_vx:
                self.vx = max(self.vx - self.acceleration, self.max_vx)
 
            # Accelerate vy towards max_vy
            if self.vy < self.max_vy:
                self.vy = min(self.vy + self.acceleration, self.max_vy)
            elif self.vy > self.max_vy:
                self.vy = max(self.vy - self.acceleration, self.max_vy)
 
 
            if self.intended_direction == 'straight':
 
                self.max_vx = 0
                self.max_vy = -config.AV_SPEED  # target velocity moving up initially
                self.y += self.vy
                self.x += self.vx
 
            elif self.intended_direction == 'left':
                # Define intersection edge y-coordinate (adjust as per your intersection size)
                intersection_edge_y = config.HEIGHT // 2 + 50  # top edge of intersection box
 
                if not hasattr(self, 'turn_stage'):
                    self.turn_stage = 0  # 0 = drive straight up, 1 = turn, 2 = drive straight right
 
                if self.turn_stage == 0:
                    self.max_vx = 0
                    self.max_vy = -config.AV_SPEED  # target velocity moving up initially
                    # Drive straight up
                    self.y += self.vy
                    self.rect.y = self.y
                    if self.y <= intersection_edge_y:
                        # Reached edge of intersection, start turn
                        self.turn_stage = 1
                        self.max_vx = -config.AV_SPEED/2
                        self.max_vy = -config.AV_SPEED/2  # target velocity moving up initially
                        self.turn_angle = 0  # Start facing up (180 degrees)
                        self.radius = config.LANE_WIDTH-5     # Smaller radius for sharper turn
                        self.arc_cx = self.x - self.radius  # Center of arc is to the right
                        self.arc_cy = self.y
                        self.acceleration = 0.025
 
                elif self.turn_stage == 1:
                    # Perform the arc turn from facing up to facing right
                    angle_speed = config.AV_SPEED / self.radius
                    self.turn_angle += angle_speed  # Turning right (clockwise)
 
                    self.x = self.arc_cx + self.radius * math.cos(self.turn_angle)
                    self.y = self.arc_cy - self.radius * math.sin(self.turn_angle)
                    self.rect.center = (self.x, self.y)
 
                    # When turn angle reaches 90 degrees (pi/2), finish turn
                    if self.turn_angle >= math.pi / 2:
                        self.turn_stage = 2
                        self.vx = -math.sqrt(math.pow(self.vx,2)+math.pow(self.vy,2))
                        self.vy = 0
                        self.max_vx = -config.AV_SPEED
                        self.max_vy = 0
                        self.turn_angle = math.pi / 2
                        self.acceleration = 0.05
 
                elif self.turn_stage == 2:
                    # Drive straight right
                    self.x += self.vx
                    self.rect.x = self.x
 
            elif self.intended_direction == 'right':
                # Define intersection edge y-coordinate (adjust as per your intersection size)
                intersection_edge_y = config.HEIGHT // 2  # top edge of intersection box
 
                if not hasattr(self, 'turn_stage'):
                    self.turn_stage = 0  # 0 = drive straight up, 1 = turn, 2 = drive straight right
 
                if self.turn_stage == 0:
                    self.max_vx = 0
                    self.max_vy = -config.AV_SPEED  # target velocity moving up initially
                    # Drive straight up
                    self.y += self.vy
                    self.rect.y = self.y
                    if self.y <= intersection_edge_y:
                        # Reached edge of intersection, start turn
                        self.turn_stage = 1
                        self.turn_angle = math.pi  # Start facing up (180 degrees)
                        self.radius = config.LANE_WIDTH/2    # Smaller radius for sharper turn
                        self.arc_cx = self.x + self.radius  # Center of arc is to the right
                        self.arc_cy = self.y
                        self.max_vx = config.AV_SPEED/2
                        self.max_vy = -config.AV_SPEED/2  # target velocity moving up initially
                        self.acceleration = 0.025
 
                elif self.turn_stage == 1:
                    # Perform the arc turn from facing up to facing right
                    angle_speed = config.AV_SPEED / self.radius
                    self.turn_angle -= angle_speed  # Turning right (clockwise)
 
                    self.x = self.arc_cx + self.radius * math.cos(self.turn_angle)
                    self.y = self.arc_cy - self.radius * math.sin(self.turn_angle)
                    self.rect.center = (self.x, self.y)
 
                    # When turn angle reaches 90 degrees (pi/2), finish turn
                    if self.turn_angle <= math.pi / 2:
                        self.turn_stage = 2
                        self.vx = math.sqrt(math.pow(self.vx,2)+math.pow(self.vy,2))
                        self.vy = 0
                        self.max_vx = config.AV_SPEED
                        self.max_vy = 0  # target velocity moving up initially
                        self.turn_angle = math.pi / 2
                        self.acceleration = 0.05
 
                elif self.turn_stage == 2:
                    # Drive straight right
                    self.x += self.vx
                    self.rect.x = self.x
 
            self.rect.x = self.x
            self.rect.y = self.y

        elif self.inching and self.inch_behave:

            if self.rect.top < (config.HEIGHT//2 + config.LANE_WIDTH):
                print("[DECISION] AV has reached the edge of the intersection and must stop.")
                self.vx = 0
                self.vy = 0
                self.inching = False
            else:
                self.vx = 0
                self.vy = -0.5

                self.x += self.vx
                self.y += self.vy

                self.rect.x = self.x
                self.rect.y = self.y

        if self.y-config.LOWER_CONFLICT_ZONE[3] > 0:
            self.col_zone_times[0,0] = travel_time(self.y-config.LOWER_CONFLICT_ZONE[3], abs(self.vy), abs(self.acceleration), config.AV_SPEED)
            self.col_zone_times[0,1] = travel_time((self.y+config.AV_HEIGHT)-config.LOWER_CONFLICT_ZONE[2], abs(self.vy), abs(self.acceleration), config.AV_SPEED)
            self.col_zone_times[1,0] = travel_time(self.y-config.UPPER_CONFLICT_ZONE[3], abs(self.vy), abs(self.acceleration), config.AV_SPEED)
            self.col_zone_times[1,1] = travel_time((self.y+config.AV_HEIGHT)-config.UPPER_CONFLICT_ZONE[2], abs(self.vy), abs(self.acceleration), config.AV_SPEED)



 
    # def get_fov_polygon(self, blockers=None):
    #     eye_x = self.rect.centerx
    #     eye_y = self.rect.top  # AV faces up

    #     total_fov_deg = 179
    #     fov_height = 140
    #     half_fov_rad = math.radians(total_fov_deg / 2)
    #     fov_radius = fov_height / math.cos(half_fov_rad)
    #     base_angle = -math.pi / 2  # Facing up

    #     # Original FOV boundary angles
    #     left_angle = base_angle - half_fov_rad
    #     right_angle = base_angle + half_fov_rad

    #     left_point = (
    #         eye_x + fov_radius * math.cos(left_angle),
    #         eye_y + fov_radius * math.sin(left_angle),
    #     )
    #     right_point = (
    #         eye_x + fov_radius * math.cos(right_angle),
    #         eye_y + fov_radius * math.sin(right_angle),
    #     )

    #     # Make top edge horizontal
    #     top_y = min(left_point[1], right_point[1])
    #     left_point = (left_point[0], top_y)
    #     right_point = (right_point[0], top_y)

    #     if not blockers:
    #         return [(eye_x, eye_y), left_point, right_point]

    #     if not isinstance(blockers, list):
    #         blockers = [blockers]

    #     # Initialize clipping edges
    #     clipped_left_x = left_point[0]
    #     clipped_right_x = right_point[0]

    #     for blocker in blockers:
    #         if not blocker:
    #             continue
    #         block_pts = [
    #             (blocker.rect.left, blocker.rect.top),
    #             (blocker.rect.left, blocker.rect.bottom),
    #             (blocker.rect.right, blocker.rect.top),
    #             (blocker.rect.right, blocker.rect.bottom),
    #         ]

    #         for pt in block_pts:
    #             dx = pt[0] - eye_x
    #             dy = pt[1] - eye_y
    #             angle = math.atan2(dy, dx)

    #             # Check if the point lies inside the FOV
    #             if left_angle <= angle <= right_angle:
    #                 if dy == 0:
    #                     continue  # avoid div by zero

    #                 # Project ray to the FOV top boundary
    #                 scale = (top_y - eye_y) / dy
    #                 x_at_top = eye_x + dx * scale

    #                 # Left side update
    #                 if angle < base_angle and x_at_top > clipped_left_x:
    #                     clipped_left_x = x_at_top
    #                 # Right side update
    #                 elif angle > base_angle and x_at_top < clipped_right_x:
    #                     clipped_right_x = x_at_top

    #     clipped_left = (clipped_left_x, top_y)
    #     clipped_right = (clipped_right_x, top_y)
    #     return [(eye_x, eye_y), clipped_left, clipped_right]   

################################################################################

    def get_fov_polygon(self, blockers=None, fov_deg=179, fov_height=140, ray_density=2):
        """
        Returns a polygon representing the FOV, clipped by blockers.
        ray_density: angular resolution in degrees (smaller = more precise).
        """

        eye_x = self.rect.centerx
        eye_y = self.rect.top  # AV faces up

        half_fov_rad = math.radians(fov_deg / 2)
        base_angle = -math.pi / 2  # Facing up
        fov_radius = fov_height / math.cos(half_fov_rad)

        # Rays: FOV boundaries + blocker corners + dense samples
        ray_angles = [base_angle - half_fov_rad, base_angle + half_fov_rad]

        if blockers:
            if not isinstance(blockers, list):
                blockers = [blockers]
            for blocker in blockers:
                if not blocker:
                    continue

                for corner in [
                    (blocker.rect.left, blocker.rect.top),
                    (blocker.rect.left, blocker.rect.bottom),
                    (blocker.rect.right, blocker.rect.top),
                    (blocker.rect.right, blocker.rect.bottom),
                ]:
                    dx, dy = corner[0] - eye_x, corner[1] - eye_y
                    angle = math.atan2(dy, dx)
                    if base_angle - half_fov_rad <= angle <= base_angle + half_fov_rad:
                        ray_angles.append(angle)

        # Optional: fill in with uniform rays for smooth shape
        for a in np.arange(-half_fov_rad, half_fov_rad, math.radians(ray_density)):
            ray_angles.append(base_angle + a)

        ray_angles = sorted(set(ray_angles))

        # Cast each ray
        points = []
        for angle in ray_angles:
            dx, dy = math.cos(angle), math.sin(angle)
            ray_end = (eye_x + dx * fov_radius, eye_y + dy * fov_radius)
            closest_pt = ray_end
            min_dist = fov_radius

            # Check blockers
            if blockers:
                for blocker in blockers:

                    if not blocker:
                        continue
                    rect_lines = [
                        ((blocker.rect.left, blocker.rect.top), (blocker.rect.right, blocker.rect.top)),
                        ((blocker.rect.right, blocker.rect.top), (blocker.rect.right, blocker.rect.bottom)),
                        ((blocker.rect.right, blocker.rect.bottom), (blocker.rect.left, blocker.rect.bottom)),
                        ((blocker.rect.left, blocker.rect.bottom), (blocker.rect.left, blocker.rect.top)),
                    ]
                    for line in rect_lines:
                        hit = self.ray_intersect_segment((eye_x, eye_y), (dx, dy), line)
                        if hit:
                            dist = math.hypot(hit[0] - eye_x, hit[1] - eye_y)
                            if dist < min_dist:
                                min_dist = dist
                                closest_pt = hit

            points.append(closest_pt)

        return [(eye_x, eye_y)] + points


    def ray_intersect_segment(self, ray_origin, ray_dir, segment):
        """
        Return intersection point of ray (origin, dir) with line segment, or None if no intersection.
        ray_origin: (x, y)
        ray_dir: normalized (dx, dy)
        segment: ((x1,y1), (x2,y2))
        """
        x1, y1 = ray_origin
        dx, dy = ray_dir
        x3, y3 = segment[0]
        x4, y4 = segment[1]

        denom = (dx * (y3 - y4) - dy * (x3 - x4))
        if abs(denom) < 1e-6:
            return None  # parallel

        t = ((x3 - x1) * (y3 - y4) - (y3 - y1) * (x3 - x4)) / denom
        u = -((dx) * (y1 - y3) - (dy) * (x1 - x3)) / denom

        if t >= 0 and 0 <= u <= 1:
            return (x1 + t * dx, y1 + t * dy)
        return None

##################################################################################

    # def get_fov_polygon(self, blockers=None):

    #     eye_x = self.rect.centerx
    #     eye_y = self.rect.top  # AV faces up
 
    #     total_fov_deg = 179
    #     fov_height = 140
    #     half_fov_rad = math.radians(total_fov_deg / 2)
    #     fov_radius = fov_height / math.cos(half_fov_rad)
    #     base_angle = -math.pi / 2  # Facing up
 
    #     # Compute original left and right points of the FOV triangle
    #     left_angle = base_angle - half_fov_rad
    #     right_angle = base_angle + half_fov_rad
 
    #     left_point = (
    #         eye_x + fov_radius * math.cos(left_angle),
    #         eye_y + fov_radius * math.sin(left_angle)
    #     )
    #     right_point = (
    #         eye_x + fov_radius * math.cos(right_angle),
    #         eye_y + fov_radius * math.sin(right_angle)
    #     )
 
    #     # Ensure top edge is horizontal (same Y for both corners)
    #     top_y = min(left_point[1], right_point[1])
    #     left_point = (left_point[0], top_y)
    #     right_point = (right_point[0], top_y)

    #     if blockers is None:
    #         return [(eye_x, eye_y), left_point, right_point]

    #     if not isinstance(blockers, list):
    #         print("[WARNING] Blockers not parsed as list. Converting now.")
    #         blockers = [blockers]

    #     for blocker in blockers:
    
    #         # Cast rays to left side of blocker (top-left and bottom-left corners)
    #         left_block_pts = [
    #             (blocker.rect.left, blocker.rect.top),
    #             (blocker.rect.left, blocker.rect.bottom)
    #         ]
    #         right_block_pts = [
    #             (blocker.rect.right, blocker.rect.top),
    #             (blocker.rect.right, blocker.rect.bottom)
    #         ]
    
    #         occluded = False
    #         closest_x = None
    
    #         for pt in right_block_pts:
    #             dx = pt[0] - eye_x
    #             dy = pt[1] - eye_y
    #             angle = math.atan2(dy, dx)
    #             angle_deg = math.degrees(angle)
    
    #             # Check if point lies within left half of 150 deg FOV
    #             if -180 < angle_deg < -90 + 75:  # Between -165 and -15 deg
    #                 # Project ray to reach top_y (horizontal cut)
    #                 if dy == 0:
    #                     continue  # Avoid division by zero
    #                 scale = (top_y - eye_y) / dy
    #                 x_at_top = eye_x + dx * scale
    
    #                 if closest_x is None or x_at_top < closest_x:
    #                     closest_x = x_at_top
    #                     occluded = True
 
    #     if occluded:
    #         clipped_left = (closest_x, top_y)
    #         return [(eye_x, eye_y), clipped_left, right_point]
    #     else:
    #         return [(eye_x, eye_y), left_point, right_point]
 
 
    def draw(self):
        color = config.YELLOW if self.collided else config.BLUE
        rotated_image = pygame.Surface((config.AV_WIDTH, config.AV_HEIGHT), pygame.SRCALPHA)
        pygame.draw.rect(rotated_image, color, (0, 0, config.AV_WIDTH, config.AV_HEIGHT))
 
        # Determine angle based on motion
        if self.intended_direction in ('left', 'right') and self.turn_angle is not None:
            # Tangent to the arc is perpendicular to the radius
            angle = math.degrees(self.turn_angle)
        elif self.vx == 0 and self.vy < 0:
            angle = 0  # Going straight up
        elif self.vx != 0:
            angle = math.degrees(self.turn_angle)
        else:
            angle = 0  # Default/fallback
 
        # Rotate the image and draw
        rotated_image = pygame.transform.rotate(rotated_image, angle)
        self.rect = rotated_image.get_rect(center=(self.rect.centerx, self.rect.centery))
        self.screen.blit(rotated_image, self.rect)
 
    def check_collision(self, other_cars):
        for car in other_cars:
            if self.rect.colliderect(car.rect):
                self.collided = True
                print("[COLLISION] Collision occurred! Resetting...")
                return True
        return False