import pygame
import math
import config

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
        self.collided = False
        self.manual_trigger = False
        self.intended_direction = 'straight'  # Options: 'left', 'straight', 'right'
        self.turn_angle = 0
        self.arc_cx = None
        self.arc_cy = None
        self.max_v_x = False
        self.max_v_y = False
        self.turn_stage = 0
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
 
   
    def get_fov_polygon(self, blocker=None):
        eye_x = self.rect.centerx
        eye_y = self.rect.top  # AV faces up
 
        total_fov_deg = 179
        fov_height = 140
        half_fov_rad = math.radians(total_fov_deg / 2)
        fov_radius = fov_height / math.cos(half_fov_rad)
        base_angle = -math.pi / 2  # Facing up
 
        # Compute original left and right points of the FOV triangle
        left_angle = base_angle - half_fov_rad
        right_angle = base_angle + half_fov_rad
 
        left_point = (
            eye_x + fov_radius * math.cos(left_angle),
            eye_y + fov_radius * math.sin(left_angle)
        )
        right_point = (
            eye_x + fov_radius * math.cos(right_angle),
            eye_y + fov_radius * math.sin(right_angle)
        )
 
        # Ensure top edge is horizontal (same Y for both corners)
        top_y = min(left_point[1], right_point[1])
        left_point = (left_point[0], top_y)
        right_point = (right_point[0], top_y)
 
        if blocker is None:
            return [(eye_x, eye_y), left_point, right_point]
 
        # Cast rays to left side of blocker (top-left and bottom-left corners)
        left_block_pts = [
            (blocker.rect.left, blocker.rect.top),
            (blocker.rect.left, blocker.rect.bottom)
        ]
        right_block_pts = [
            (blocker.rect.right, blocker.rect.top),
            (blocker.rect.right, blocker.rect.bottom)
        ]
 
        occluded = False
        closest_x = None
 
        for pt in right_block_pts:
            dx = pt[0] - eye_x
            dy = pt[1] - eye_y
            angle = math.atan2(dy, dx)
            angle_deg = math.degrees(angle)
 
            # Check if point lies within left half of 150 deg FOV
            if -180 < angle_deg < -90 + 75:  # Between -165 and -15 deg
                # Project ray to reach top_y (horizontal cut)
                if dy == 0:
                    continue  # Avoid division by zero
                scale = (top_y - eye_y) / dy
                x_at_top = eye_x + dx * scale
 
                if closest_x is None or x_at_top < closest_x:
                    closest_x = x_at_top
                    occluded = True
 
        if occluded:
            clipped_left = (closest_x, top_y)
            return [(eye_x, eye_y), clipped_left, right_point]
        else:
            return [(eye_x, eye_y), left_point, right_point]
 
 
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