import pygame
import math
import random
import config


class CrossTrafficCar:
    def __init__(self, direction, screen):
        self.direction_int = {'left' : -1 , 'right' : 1}
        self.direction = direction
        self.speed = config.CROSS_SPEED
        self.current_speed = self.speed  # Current actual speed (can be reduced for braking)
        self.max_speed = self.speed      # Maximum speed this car can go
        self.turn_angle = None
        self.turn_danger = False
        self.screen = screen
        self.turn_stage = 0
        self.e_stop = False
        
        # Braking parameters
        self.brake_distance = 60         # Distance to start braking
        self.min_following_distance = 35 # Minimum distance to maintain
        self.deceleration = 0.1         # How quickly to slow down
        self.acceleration = 0.1        # How quickly to speed up
        
        if direction == 'right':
            self.drive_path = config.DRIVE_PATH[random.choices([0, 1], weights=[0.7, 0.3], k=1)[0]]
            self.x = -config.CAR_WIDTH
            self.y = config.HEIGHT // 2 - 25
            self.vx = self.current_speed
            self.vy = 0
            if self.drive_path == 'straight':
                pass
            elif self.drive_path == 'left':
                self.start_turn_point = (config.WIDTH / 2) - (1.5*config.LANE_WIDTH) - config.CAR_WIDTH/2
            else:
                self.start_turn_point = (config.WIDTH / 2) + config.LANE_WIDTH - config.CAR_WIDTH

        else:
            self.drive_path = config.DRIVE_PATH[random.choices([0, 1, 2], weights=[0.6, 0.2, 0.2], k=1)[0]]
            self.x = config.WIDTH
            self.y = config.HEIGHT // 2 + 5
            self.vx = -self.current_speed
            self.vy = 0
            if self.drive_path == 'straight':
                pass
            elif self.drive_path == 'left':
                self.start_turn_point = (config.WIDTH / 2) + (1.5*config.LANE_WIDTH) - config.CAR_WIDTH/2
            else:
                self.start_turn_point = (config.WIDTH / 2) + (config.LANE_WIDTH/2)

        self.rect = pygame.Rect(self.x, self.y, config.CAR_WIDTH, config.CAR_HEIGHT)

    def _are_cars_in_collision_path(self, other_car):
        """Check if two cars are on a collision path"""
        collision = [0,]

############### Find Lead and Tail Edges ##############

        if self.direction == 'left':
            lead_edge = self.x
            tail_edge = self.x+config.CAR_WIDTH
            inside_edge = self.y
            outside_edge = self.y+config.CAR_HEIGHT
        else:
            lead_edge = self.x+config.CAR_WIDTH
            tail_edge = self.x
            inside_edge = self.y+config.CAR_HEIGHT
            outside_edge = self.y

        if other_car.direction == 'left':
            other_lead_edge = other_car.x
            other_tail_edge = other_car.x+config.CAR_WIDTH
            other_inside_edge = other_car.y
            other_outside_edge = other_car.y+config.CAR_HEIGHT
        else:
            other_lead_edge = other_car.x+config.CAR_WIDTH
            other_tail_edge = other_car.x
            other_inside_edge = other_car.y+config.CAR_HEIGHT
            other_outside_edge = other_car.y

        intersection_cx = config.WIDTH//2
        intersection_cy = config.HEIGHT//2
        direction_int = self.direction_int[self.direction]
        other_direction_int = other_car.direction_int[other_car.direction]

###################### Rule out obvious non-collisions ###############

        ##### Other car is passed intersection and completed turn #########
        if not other_car.is_in_intersection() and other_car.vx == 0 and not self.e_stop:
            return False
        
        ##### Car is passed intersection and completed turn ##############
        #####   Note: also sets speed of vehicle to max     ##############
        if not self.is_in_intersection() and self.vx == 0 and not self.e_stop:
            if self.direction == 'right':
                if self.drive_path == 'left':
                    self.vy = -self.max_speed
                else:
                    self.vy = self.max_speed
            else:
                if self.drive_path == 'left':
                    self.vy = self.max_speed
                else:
                    self.vy = -self.max_speed
            return False
        
        ##### Car is in intersection but turning without crossing traffic #########
        if self.is_in_intersection() and self.drive_path == 'left':
            return False
        
############ Work through other scenarios of potential collisions ##################

        #### If Other car is in front of car and travelling same direction ######
        if self.direction == other_car.direction:
            if direction_int*(other_tail_edge - lead_edge) > 0:
        #### Don't bother stopping if the other car has finished turning ########
                if other_car.turn_stage < 2:
                    collision.append(1)
        #### Set e stop to stop creeping ###############
            if 0 < direction_int*(other_tail_edge - lead_edge) < 1.5*config.CAR_WIDTH:
                # if not self.e_stop:
                #     print('Applying e stop')
                self.e_stop = True
            # elif self.e_stop:
            #     print('Turning off e-stop')
            #     self.current_speed = 0.25*self.max_speed
            #     self.e_stop = False
            else:
                self.e_stop = False

        #### If car is turning and other car is in opposing traffic within intersection
        if self.drive_path == 'right' and self.direction != other_car.direction:
            # if 0 < direction_int*(self.start_turn_point - lead_edge) < 2*config.LANE_WIDTH and -config.CAR_WIDTH < other_direction_int*(self.start_turn_point - other_lead_edge) < 3*config.LANE_WIDTH:
            #     # self.current_speed = 0
            #     collision.append(1)
            if self.is_in_intersection() and other_car.is_in_intersection():
                if not self.turn_stage > 0:
                    collision.append(1)

        #### If car other car is turning in front of car ##################    
        if other_car.drive_path == 'right' and abs(lead_edge - other_lead_edge) < config.LANE_WIDTH and other_car.turn_stage > 0:
            collision.append(1)



        # # Check for turning conflicts
        # if self.drive_path == 'right':
        #     # If either car is turning, use proximity-based collision detection
        #     # if distance < 3*config.LANE_WIDTH:
        #     if self.direction != other_car.direction:
        #         if abs(self.x - config.WIDTH//2) < 2*config.LANE_WIDTH and abs(other_car.x - config.WIDTH//2) < 2*config.LANE_WIDTH:
        #             self.turn_danger = True
        #             if other_car.current_speed == 0:
        #                 if abs(other_car.x-config.WIDTH//2)>config.CAR_WIDTH or self.y < config.HEIGHT//2:
        #                     # return False
        #                     collision.append(0)
        #                 else:
        #                     # return True
        #                     collision.append(1)
        #             else:
        #                 # other_car.current_speed = other_car.max_speed
        #                 # return True
        #                 collision.append(1)
        
        # if self.drive_path == 'right' and self.turn_stage > 0 and other_car.current_speed == 0:
        #     # return False
        #     collision.append(0)
        
        # # First check if cars are too far apart to matter
        # distance = math.sqrt((self.x - other_car.x)**2 + (self.y - other_car.y)**2)
        # if distance > 150:  # If cars are very far apart, no collision possible
        #     # return False
        #     collision.append(0)
        
        
        # # Check for same direction traffic (following behavior)
        # if self.direction == other_car.direction:
        #     if self.direction == 'right':
        #         # For right-moving traffic, check if other car is ahead and in same lane
        #         if (other_car.x > self.x and 
        #             abs(self.y - other_car.y) < config.CAR_HEIGHT + 10):
        #             # return True
        #             collision.append(1)
        #     else:  # direction == 'left'
        #         # For left-moving traffic, check if other car is ahead and in same lane
        #         if (other_car.x < self.x and 
        #             abs(self.y - other_car.y) < config.CAR_HEIGHT + 10):
        #             # return True
        #             collision.append(1)
        
        # if self.is_in_intersection() and other_car.drive_path == 'right' and other_car.turn_angle != None:
        #     # return True
        #     collision.append(1)
                
        # if self.drive_path != 'left' and (0 < self.direction_int[self.direction]*(config.WIDTH//2-self.x) < 2*config.LANE_WIDTH) and other_car.drive_path == 'right' and abs(other_car.x - config.WIDTH//2) < 2*config.LANE_WIDTH: #other_car.turn_stage > 0:
        #     if self.turn_stage == 0:
        #         # return False
        #         collision.append(0)
        #     else:
        #         # return True
        #         collision.append(1)
        
        # return False
        return any(collision)

    def _calculate_distance_ahead(self, other_car):
        """Calculate distance to a car ahead in the same direction"""
        
        # For same direction traffic
        if self.direction == other_car.direction:
            if self.direction == 'right':
                if other_car.x > self.x:  # Other car is ahead
                    return other_car.x - (self.x + config.CAR_WIDTH)
            else:  # direction == 'left'
                if other_car.x < self.x:  # Other car is ahead
                    return self.x - (other_car.x + config.CAR_WIDTH)
        
        # For intersection and turning scenarios, use Euclidean distance
        distance = math.sqrt((self.rect.centerx - other_car.rect.centerx)**2 + 
                            (self.rect.centery - other_car.rect.centery)**2)
        return max(0, distance - (config.CAR_WIDTH + config.CAR_HEIGHT))

    def adjust_speed(self, other_cars):
        """Adjust speed based on nearby cars"""
        closest_car, distance = self.check_collision_ahead(other_cars)

        if self.e_stop:
            self.current_speed = 0
            self.vx = 0
        else:        
            if closest_car and distance is not None:
                if distance < self.min_following_distance:
                    # Emergency braking - car too close
                    self.current_speed = max(0, self.current_speed - self.deceleration * 4)
                elif distance < self.brake_distance:
                    # Gradual braking - car within brake distance
                    brake_factor = (self.brake_distance - distance) / self.brake_distance
                    target_speed = self.max_speed * (1 - brake_factor * 0.7)
                    if self.current_speed > target_speed:
                        self.current_speed = max(target_speed, self.current_speed - self.deceleration * 2)
                else:
                    # Clear ahead - accelerate back to normal speed
                    if self.current_speed < self.max_speed:
                        self.current_speed = min(self.max_speed, self.current_speed + self.acceleration)
            else:
                # No car ahead - accelerate back to normal speed
                if self.current_speed < self.max_speed:
                    self.current_speed = min(self.max_speed, self.current_speed + self.acceleration)
            
            # Update velocity components based on current movement state
            if self.turn_stage == 0:  # Moving straight
                if self.direction == 'right':
                    self.vx = self.current_speed
                    self.vy = 0
                else:
                    self.vx = -self.current_speed
                    self.vy = 0
            elif self.turn_stage > 0 and hasattr(self, 'turn_angle'):
                # During turning, maintain speed but adjust for turn radius
                # The turning logic in update() will handle the actual velocity direction
                pass  # Let the turning logic in update() handle velocity direction
            else:
                # Moving vertically after turn
                if abs(self.vy) > 0:
                    if self.vy > 0:
                        self.vy = self.current_speed
                    else:
                        self.vy = -self.current_speed
                    self.vx = 0

    def check_collision_ahead(self, other_cars):
        """Check if there's a car ahead that we might collide with"""
        closest_car = None
        closest_distance = float('inf')
        
        for other_car in other_cars:
            if other_car == self:
                continue
                
            # Check if cars are in collision path
            if not self._are_cars_in_collision_path(other_car):
                continue
                
            distance = self._calculate_distance_ahead(other_car)
            if distance is not None and distance >= 0 and distance < closest_distance:
                closest_distance = distance
                closest_car = other_car
                    
        return closest_car, closest_distance if closest_car else None
 

    def update(self, other_cars=None):
        # Adjust speed based on other cars
        if other_cars:
            self.adjust_speed(other_cars)

        if not self.e_stop:
            
            if self.direction == 'left': # Left travelling traffic
                if self.drive_path == 'straight':
                    self.x += self.vx
                    self.rect.x = self.x
                elif self.drive_path == 'left':
                    # self.start_turn_point = (config.WIDTH / 2) + (1.5*config.LANE_WIDTH) - config.CAR_WIDTH/2

                    if not hasattr(self, 'turn_stage'):
                        self.turn_stage = 0

                    if self.turn_stage == 0:
                        self.x += self.vx
                        self.rect.x = self.x
                        if self.x <= self.start_turn_point:
                        # if self.can_start_turn(self.start_turn_point, other_cars): # and not self.turn_danger
                            self.turn_stage = 1
                            self.turn_angle = 0
                            self.radius = config.LANE_WIDTH/2 - config.CAR_HEIGHT/2 - 5
                            self.arc_cx = self.x
                            self.arc_cy = self.y + self.radius

                    elif self.turn_stage == 1:
                        angle_speed = self.current_speed / self.radius  # Use current_speed instead of CROSS_SPEED
                        self.turn_angle += angle_speed

                        self.x = self.arc_cx + self.radius * math.cos(self.turn_angle+math.pi/2)
                        self.y = self.arc_cy - self.radius * math.sin(self.turn_angle+math.pi/2)
                        self.rect.center = (self.x, self.y)

                        if self.turn_angle >= math.pi / 2:
                            self.turn_stage = 2
                            self.vx = 0
                            self.vy = self.current_speed
                            self.turn_angle = math.pi / 2

                    elif self.turn_stage == 2:
                        self.y += self.vy
                        self.rect.y = self.y

                elif self.drive_path == 'right':
                    # self.start_turn_point = (config.WIDTH / 2) + (config.LANE_WIDTH/2)

                    if not hasattr(self, 'turn_stage'):
                        self.turn_stage = 0

                    if self.turn_stage == 0:
                        self.x += self.vx
                        self.rect.x = self.x
                        if self.x <= self.start_turn_point:
                            self.turn_stage = 1
                            self.turn_angle = 0
                            self.radius = config.LANE_WIDTH/2
                            self.arc_cx = self.x
                            self.arc_cy = self.y - self.radius

                    elif self.turn_stage == 1:
                        angle_speed = self.current_speed / self.radius  # Use current_speed
                        self.turn_angle -= angle_speed

                        self.x = self.arc_cx + self.radius * math.cos(self.turn_angle-math.pi/2)
                        self.y = self.arc_cy - self.radius * math.sin(self.turn_angle-math.pi/2)
                        self.rect.center = (self.x, self.y)

                        if self.turn_angle <= -math.pi / 2:
                            self.turn_stage = 2
                            self.vx = 0
                            self.vy = -self.current_speed
                            self.turn_angle = -math.pi / 2

                    elif self.turn_stage == 2:
                        self.y += self.vy
                        self.rect.y = self.y

            else: # Right travelling traffic
                if self.drive_path == 'straight':
                    self.x += self.vx
                    self.rect.x = self.x
                elif self.drive_path == 'left':
                    # self.start_turn_point = (config.WIDTH / 2) - (1.5*config.LANE_WIDTH) - config.CAR_WIDTH/2

                    if not hasattr(self, 'turn_stage'):
                        self.turn_stage = 0

                    if self.turn_stage == 0:
                        self.x += self.vx
                        self.rect.x = self.x
                        if self.x >= self.start_turn_point:
                            self.turn_stage = 1
                            self.turn_angle = 0
                            self.radius = config.LANE_WIDTH/2 + 5
                            self.arc_cx = self.x + config.CAR_WIDTH/2
                            self.arc_cy = self.y - self.radius

                    elif self.turn_stage == 1:
                        angle_speed = self.current_speed / self.radius  # Use current_speed
                        self.turn_angle += angle_speed

                        self.x = self.arc_cx + self.radius * math.cos(self.turn_angle-math.pi/2)
                        self.y = self.arc_cy - self.radius * math.sin(self.turn_angle-math.pi/2)
                        self.rect.center = (self.x, self.y)

                        if self.turn_angle >= math.pi / 2:
                            self.turn_stage = 2
                            self.vx = 0
                            self.vy = -self.current_speed
                            self.turn_angle = math.pi / 2

                    elif self.turn_stage == 2:
                        self.y += self.vy
                        self.rect.y = self.y

                elif self.drive_path == 'right':
                    # self.start_turn_point = (config.WIDTH / 2) + config.LANE_WIDTH - config.CAR_WIDTH

                    if not hasattr(self, 'turn_stage'):
                        self.turn_stage = 0

                    if self.turn_stage == 0:
                        self.x += self.vx
                        self.rect.x = self.x
                        if self.x >= self.start_turn_point:
                            self.turn_stage = 1
                            self.turn_angle = 0
                            self.radius = config.LANE_WIDTH/2+config.CAR_WIDTH/2 - 5
                            self.arc_cx = self.x
                            self.arc_cy = self.y + self.radius

                    elif self.turn_stage == 1:
                        angle_speed = self.current_speed / self.radius  # Use current_speed
                        self.turn_angle -= angle_speed

                        self.x = self.arc_cx + self.radius * math.cos(self.turn_angle+math.pi/2)
                        self.y = self.arc_cy - self.radius * math.sin(self.turn_angle+math.pi/2)
                        self.rect.center = (self.x, self.y)

                        if self.turn_angle <= -math.pi / 2:
                            self.turn_stage = 2
                            self.vx = 0
                            self.vy = self.current_speed
                            self.turn_angle = -math.pi / 2

                    elif self.turn_stage == 2:
                        self.y += self.vy
                        self.rect.y = self.y

    def draw(self):
        # Color changes based on speed - red when braking, green when normal
        if self.current_speed < self.max_speed * 0.5:
            color = config.RED  # Braking hard
        elif self.current_speed < self.max_speed * 0.8:
            color = config.YELLOW  # Slowing down
        else:
            color = config.GREEN  # Normal speed
            
        rotated_image = pygame.Surface((config.CAR_WIDTH, config.CAR_HEIGHT), pygame.SRCALPHA)
        pygame.draw.rect(rotated_image, color, (0, 0, config.CAR_WIDTH, config.CAR_HEIGHT))

        if self.drive_path != 'straight' and self.turn_stage < 1:
            # Add a small orange square to indicate turning direction
            indicator_size = 6
            orange = (255, 165, 0)

            w = config.CAR_WIDTH
            h = config.CAR_HEIGHT
            s = indicator_size

            # Precompute valid corner positions (x, y) for the indicator
            corners = {
                'top_left':     (0, 0),
                'top_right':    (w - s, 0),
                'bottom_left':  (0, h - s),
                'bottom_right': (w - s, h - s),
            }

            # Choose corner according to drive_path and travel direction.
            # This preserves your original mapping but avoids arithmetic that can go negative.
            if self.drive_path == 'left':
                if self.direction == 'left':
                    chosen = 'bottom_left'   # moving left, turning left
                else:
                    chosen = 'top_right'     # moving other direction, turning left
            else:  # drive_path == 'right'
                if self.direction == 'left':
                    chosen = 'top_left'  # moving left, turning right
                else:
                    chosen = 'bottom_right'      # moving other direction, turning right

            indicator_x, indicator_y = corners[chosen]

            # Draw indicator
            pygame.draw.rect(rotated_image, orange, (indicator_x, indicator_y, s, s))
 
        # Determine angle based on motion
        if self.drive_path in ('left', 'right') and self.turn_angle is not None:
            angle = math.degrees(self.turn_angle) 
        elif hasattr(self, 'vy') and self.vx == 0 and self.vy > 0 and not self.e_stop:
            angle = math.degrees(math.pi/2)
        elif hasattr(self, 'vy') and self.vx == 0 and self.vy < 0 and not self.e_stop:
            angle = math.degrees(-math.pi/2)
        else:
            angle = 0
 
        # Rotate the image and draw
        rotated_image = pygame.transform.rotate(rotated_image, angle)
        self.rect = rotated_image.get_rect(center=(self.rect.centerx, self.rect.centery))
        self.screen.blit(rotated_image, self.rect)
 
    def is_in_intersection(self):
        return self.rect.colliderect(config.INTERSECTION_BOX)