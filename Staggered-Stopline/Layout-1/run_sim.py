import pygame
import random
import sys
import time
# import math
# from matplotlib.path import Path
import config
import av_class
import blocker_vehicle_class
import cross_traffic_class
import utils
 
pygame.init()
screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
pygame.display.set_caption("AV Intersection Simulator")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)
 
 
def reset_simulation():
    time.sleep(1.5)
    run_sim()
 
 
def run_sim(include_stationary_vehicle=False):
    av = av_class.AutonomousVehicle(screen)
    if include_stationary_vehicle:
        stationary_vehicle = blocker_vehicle_class.StationaryVehicle(screen)
    else:
        stationary_vehicle = None

    intersection_obstruction = None
    parked_vehicle = None
    cross_traffic = []
    running = True
    deciding = False
 
    while running:
        utils.draw_roads(screen)

        if av.intended_direction not in ['straight', 'right']:
            print(f"[WARNING] AV trying to go illegal direction! ({av.intended_direction})")
            print('[WARNING] Changing AV direction to straight.')
            av.intended_direction = 'straight'
 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                av.manual_trigger = True
                print("[INPUT] SPACE pressed: AV is now allowed to attempt to go.")
 
            if event.type == pygame.KEYDOWN and event.key == pygame.K_b:
                if stationary_vehicle is None:
                    stationary_vehicle = blocker_vehicle_class.StationaryVehicle(screen)
                    print("[TOGGLE] Stationary vehicle added.")
                else:
                    stationary_vehicle = None
                    print("[TOGGLE] Stationary vehicle removed.")

            if event.type == pygame.KEYDOWN and event.key == pygame.K_o:
                if intersection_obstruction is None:
                    intersection_obstruction = blocker_vehicle_class.IntersectionObstruction(screen)
                    print("[TOGGLE] Blocking object added (on right).")
                else:
                    intersection_obstruction = None
                    print("[TOGGLE] Blocking object removed (on right).")

            if event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                if parked_vehicle is None:
                    parked_vehicle = blocker_vehicle_class.ParkedVehicle(screen)
                    print("[TOGGLE] Parked vehicle added (on left).")
                else:
                    parked_vehicle = None
                    print("[TOGGLE] Parked vehicle removed (on left).")
 
            if event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                # av.intended_direction = 'left'
                # print("[INPUT] AV intends to turn LEFT.")
                print("[INPUT] AV can't turn LEFT from here!")
            if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                av.intended_direction = 'straight'
                print("[INPUT] AV intends to go STRAIGHT.")
            if event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                av.intended_direction = 'right'
                print("[INPUT] AV intends to turn RIGHT.")
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                print("[INPUT] Simulation reset.")
                reset_simulation()

 
        # Spawn cross traffic
        if random.random() < config.SPAWN_RATE:
            direction = random.choice(['left', 'right'])
            # cross_traffic.append(CrossTrafficCar(direction))
            new_car = cross_traffic_class.CrossTrafficCar(direction, screen)
            
            # Check if new car would collide with existing cars
            collision_detected = False
            for existing_car in cross_traffic:
                if new_car.rect.colliderect(existing_car.rect):
                    collision_detected = True
                    break
            
            # Only add the new car if no collision detected
            if not collision_detected:
                cross_traffic.append(new_car)
 
        for car in cross_traffic:
            car.update(cross_traffic)
        cross_traffic = [c for c in cross_traffic if (-config.CAR_WIDTH < c.x < config.WIDTH + config.CAR_WIDTH) and (-config.CAR_HEIGHT < c.y < config.HEIGHT + config.CAR_HEIGHT)]
 
        # AV decision to move
        if av.manual_trigger and not av.moving and not av.collided:
            if utils.should_av_go(cross_traffic, av, [stationary_vehicle, intersection_obstruction, parked_vehicle]):
            # if utils.should_av_go_probabilistic(cross_traffic, av, stationary_vehicle):
            # if utils.should_av_go_hmm(cross_traffic, av, stationary_vehicle):
                av.moving = True
                print("[DECISION] AV proceeds through intersection.")
            else:
                if not deciding:
                    print("[DECISION] AV waits: intersection not clear.")
                    deciding = True
 
        av.update()
 
        # Check for collision
        if av.check_collision(cross_traffic):
            av.draw()
            for car in cross_traffic:
                car.draw()
            pygame.display.flip()
            reset_simulation()
 
        # Check for success (AV fully exited top of screen)
        # if av.y + AV_HEIGHT < 250:
        #     print("[SUCCESS] AV successfully crossed the intersection.")
        #     reset_simulation()
        if av.y + config.AV_HEIGHT < 250 or av.x < 0 or av.x > config.WIDTH:
            print("[SUCCESS] AV successfully crossed the intersection.")
            reset_simulation()
 
        av.draw()
        if stationary_vehicle is not None:
            stationary_vehicle.draw()
        if intersection_obstruction is not None:
            intersection_obstruction.draw()
        if parked_vehicle is not None:
            parked_vehicle.draw()
        
        if abs(av.vx) != config.AV_SPEED and av.turn_stage < 1:
            fov_polygon = av.get_fov_polygon([stationary_vehicle, intersection_obstruction, parked_vehicle])
    
            fov_surface = pygame.Surface((config.WIDTH, config.HEIGHT), pygame.SRCALPHA)
            pygame.draw.polygon(fov_surface, (255, 255, 255, 80), fov_polygon)
            screen.blit(fov_surface, (0, 0))
        for car in cross_traffic:
            car.draw()
            if utils.is_car_in_fov(car, av,[stationary_vehicle, intersection_obstruction, parked_vehicle]):
                pygame.draw.circle(screen, (255, 0, 0), car.rect.center, 5)  # small red dot
 
        status_text = font.render(
            f"AV {'MOVING' if av.moving else 'WAITING'} | Press SPACE to GO", True, (0, 0, 0)
        )
        screen.blit(status_text, (10, 10))
        # Additional instructions
        instruction1 = font.render("Press 'B' to toggle blocking vehicle", True, (0, 0, 0))
        screen.blit(instruction1, (10, 35))

        instruction1 = font.render("Press 'O' to toggle blocking object", True, (0, 0, 0))
        screen.blit(instruction1, (10, 60))

        instruction1 = font.render("Press 'P' to toggle parked vehicle", True, (0, 0, 0))
        screen.blit(instruction1, (10, 85))

        instruction2 = font.render("Press Arrows to alter AV direction", True, (0, 0, 0))
        screen.blit(instruction2, (10, 110))

        instruction3 = font.render("Press 'R' to reset", True, (0, 0, 0))
        screen.blit(instruction3, (10, 135))
 
        pygame.display.flip()
        clock.tick(config.FPS)
 
 
if __name__ == "__main__":
    include_blocker = "--blocker" in sys.argv
    run_sim(include_stationary_vehicle=include_blocker)