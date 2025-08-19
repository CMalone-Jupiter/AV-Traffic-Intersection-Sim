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
import numpy as np
 
pygame.init()
screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
pygame.display.set_caption("AV Intersection Simulator")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)
# possible_directions = ['straight', 'left', 'right']
 
 
def reset_simulation(epoch, epochs, running, success, av_direction, include_stationary_vehicle):
    time.sleep(1.5)
    run_sim(epoch, epochs, running, success, av_direction, include_stationary_vehicle)

# Define a custom event
MY_EVENT = pygame.USEREVENT + 1

# Function to start a random timer
def set_decision_timer(min_ms, max_ms):
    delay = random.randint(min_ms, max_ms)
    pygame.time.set_timer(MY_EVENT, delay, loops=1)  # fire once
 
 
def run_sim(epoch, epochs, running, success, av_direction='straight', include_stationary_vehicle=False):
    save_dir = './'
    save_name = f"epochs-{epochs}_direction-{av_direction}_blocker-{include_stationary_vehicle}.csv"
    av = av_class.AutonomousVehicle(screen)
    if include_stationary_vehicle:
        stationary_vehicle = blocker_vehicle_class.StationaryVehicle(screen)
    else:
        stationary_vehicle = None
    cross_traffic = []
    # running = True
    deciding = False
    av.intended_direction = av_direction
    set_decision_timer(5000, 15000)

    if epoch >= epochs:
        running = False
        print(f"[STATUS] Finished running {epochs} experiments!")
    else:
        print(f"[STATUS] Running experiment {epoch+1}")

 
    while running:
        utils.draw_roads(screen)
 
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         pygame.quit()
        #         sys.exit()
        #     if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
        #         av.manual_trigger = True
        #         print("[INPUT] SPACE pressed: AV is now allowed to attempt to go.")
 
        #     if event.type == pygame.KEYDOWN and event.key == pygame.K_b:
        #         if stationary_vehicle is None:
        #             stationary_vehicle = blocker_vehicle_class.StationaryVehicle(screen)
        #             print("[TOGGLE] Stationary vehicle added.")
        #         else:
        #             stationary_vehicle = None
        #             print("[TOGGLE] Stationary vehicle removed.")
 
        #     if event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
        #         av.intended_direction = 'left'
        #         print("[INPUT] AV intends to turn LEFT.")
        #     if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
        #         av.intended_direction = 'straight'
        #         print("[INPUT] AV intends to go STRAIGHT.")
        #     if event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
        #         av.intended_direction = 'right'
        #         print("[INPUT] AV intends to turn RIGHT.")
        #     if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
        #         print("[INPUT] Simulation reset.")
        #         reset_simulation()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == MY_EVENT:
                av.manual_trigger = True
                print("[TRIGGER] AV is now allowed to attempt to go.")

 
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
        cross_traffic = [c for c in cross_traffic if (-config.CAR_WIDTH < c.x < config.WIDTH) and (-config.CAR_HEIGHT < c.y < config.HEIGHT)]
 
        # AV decision to move
        # print(f"[MOVE CHECK] manual trigger: {av.manual_trigger}, moving: {av.moving}, collided: {av.collided}")
        if av.manual_trigger and not av.moving and not av.collided:
            if utils.should_av_go(cross_traffic, av, stationary_vehicle):
                av.moving = True
                # print("[DECISION] AV proceeds through intersection.")
            else:
                if not deciding:
                    # print("[DECISION] AV waits: intersection not clear.")
                    deciding = True
 
        av.update()
 
        # Check for collision
        if av.check_collision(cross_traffic):
            av.draw()
            for car in cross_traffic:
                car.draw()
            pygame.display.flip()
            print('[OUTCOME] AV failed to navigate intersection')
            success.append(0)
            epoch += 1
            reset_simulation(epoch, epochs, running, success, av_direction, include_stationary_vehicle)
 
        # Check for success (AV fully exited top of screen)
        # if av.y + AV_HEIGHT < 250:
        #     print("[SUCCESS] AV successfully crossed the intersection.")
        #     reset_simulation()
        if av.y + config.AV_HEIGHT < 250 or av.x < 0 or av.x > config.WIDTH:
            # print("[SUCCESS] AV successfully crossed the intersection.")
            print('[OUTCOME] AV successfully navigated intersection!')
            success.append(1)
            epoch += 1
            running = reset_simulation(epoch, epochs, running, success, av_direction, include_stationary_vehicle)
 
        av.draw()
        if stationary_vehicle is not None:
            stationary_vehicle.draw()
        
        if abs(av.vx) != config.AV_SPEED and av.turn_stage < 1:
            fov_polygon = av.get_fov_polygon(stationary_vehicle)
    
            fov_surface = pygame.Surface((config.WIDTH, config.HEIGHT), pygame.SRCALPHA)
            pygame.draw.polygon(fov_surface, (255, 255, 255, 80), fov_polygon)
            screen.blit(fov_surface, (0, 0))
        for car in cross_traffic:
            car.draw()
            if utils.is_car_in_fov(car, av,stationary_vehicle):
                pygame.draw.circle(screen, (255, 0, 0), car.rect.center, 5)  # small red dot
 
        # status_text = font.render(
        #     f"AV {'MOVING' if av.moving else 'WAITING'} | Press SPACE to GO", True, (0, 0, 0)
        # )
        # screen.blit(status_text, (10, 10))
        # # Additional instructions
        # instruction1 = font.render("Press 'B' to toggle blocking vehicle", True, (0, 0, 0))
        # screen.blit(instruction1, (10, 35))

        # instruction2 = font.render("Press Arrows to alter AV direction", True, (0, 0, 0))
        # screen.blit(instruction2, (10, 60))

        # instruction3 = font.render("Press 'R' to reset", True, (0, 0, 0))
        # screen.blit(instruction3, (10, 85))
 
        pygame.display.flip()
        clock.tick(config.FPS)

    print(f"[RESULTS] AV had a {(np.array(success).sum()/epochs)*100}% success rate trying to cross the intersection")
    np.savetxt(save_dir+save_name, np.array(success), delimiter=',')
 
 
if __name__ == "__main__":
    include_blocker = "--blocker" in sys.argv
    epoch = 0
    epochs = 5
    running = True
    av_direction = 'straight'
    success = []
    run_sim(epoch, epochs, running, success, av_direction, include_stationary_vehicle=include_blocker)