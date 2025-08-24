import pygame
import random
import sys
import time
import argparse
import config
import av_class
import blocker_vehicle_class
import cross_traffic_class
import utils
import numpy as np
import os
import cv2

parser = argparse.ArgumentParser(description="Run pygame app with options")
parser.add_argument("--blocker", action="store_true",
                    help="Add blocking vehicle to stop line")
parser.add_argument("--epochs", type=int, default=10,
                    help="Number of experiment attempts")
parser.add_argument("--av_direction", type=str, default="straight", choices=["straight", "left", "right"],
                    help="Direction the AV is attempting to turn")
parser.add_argument("--save_fails", action="store_true",
                    help="Save any attempts that resulted in a collision")

if config.HEADERLESS:
    os.environ["SDL_VIDEODRIVER"] = "dummy"
 
pygame.init()
# if not HEADERLESS:
screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
pygame.display.set_caption("AV Intersection Simulator")
# else:
#     screen = pygame.Surface((config.WIDTH, config.HEIGHT))

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)
# possible_directions = ['straight', 'left', 'right']
 
 
def reset_simulation(epoch, epochs, running, success, av_direction, save_fails, include_stationary_vehicle):
    time.sleep(1.5)
    run_sim(epoch, epochs, running, success, av_direction, save_fails, include_stationary_vehicle)

# Define a custom event
MY_EVENT = pygame.USEREVENT + 1

# Function to start a random timer
def set_decision_timer(min_ms, max_ms):
    delay = random.randint(min_ms, max_ms)
    pygame.time.set_timer(MY_EVENT, delay, loops=1)  # fire once
 
 
def run_sim(epoch, epochs, running, success, av_direction='straight', save_fails=False, include_stationary_vehicle=False):
    save_dir = './results/'
    save_name = f"epochs-{epochs}_direction-{av_direction}_blocker-{include_stationary_vehicle}.csv"
    save_fails = save_fails
    frames = []
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
        print(f"[RESULTS] AV had a {(np.array(success).sum()/epochs)*100}% success rate trying to cross the intersection")
        np.savetxt(save_dir+save_name, np.array(success), delimiter=',')
        pygame.quit()
        sys.exit()
    else:
        print(f"[STATUS] Running experiment {epoch+1}")

 
    while running:
        utils.draw_roads(screen)
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

            if save_fails:
                ####### Draw final frame ##########################################
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
        
                pygame.display.flip()
                frame = pygame.surfarray.array3d(screen)        # (w, h, 3)
                frame = np.transpose(frame, (1, 0, 2))          # (h, w, 3)
                ####################################################################

                for idx in range(90):
                    frames.append(frame)

                print("[SAVING] Saving the screen recording from this collision")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # mp4 codec
                out = cv2.VideoWriter(save_dir+f"epoch-{epoch}_for_epochs-{epochs}_direction-{av_direction}_blocker-{include_stationary_vehicle}.mp4", fourcc, 60, (config.WIDTH, config.HEIGHT))
                for frame in frames:
                    bgr_frame = frame[:, :, ::-1]
                    out.write(bgr_frame)
                out.release()

            frames.clear()

            epoch += 1
            running = reset_simulation(epoch, epochs, running, success, av_direction, save_fails, include_stationary_vehicle)
 
        # Check for success (AV fully exited top of screen)
        if av.y + config.AV_HEIGHT < 250 or av.x < 0 or av.x > config.WIDTH:
            # print("[SUCCESS] AV successfully crossed the intersection.")
            print('[OUTCOME] AV successfully navigated intersection!')
            success.append(1)
            epoch += 1
            frames.clear()
            running = reset_simulation(epoch, epochs, running, success, av_direction, save_fails, include_stationary_vehicle)
 
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
 
        pygame.display.flip()
        frame = pygame.surfarray.array3d(screen)        # (w, h, 3)
        frame = np.transpose(frame, (1, 0, 2))          # (h, w, 3)
        frames.append(frame)
        clock.tick(config.FPS)
 
 
if __name__ == "__main__":
    # include_blocker = "--blocker" in sys.argv
    args = parser.parse_args()
    include_blocker = args.blocker
    save_fails = args.save_fails
    if include_blocker:
        block = 'with'
    else:
        block = 'without'
    epoch = 0
    epochs = args.epochs
    running = True
    av_direction = args.av_direction
    success = []
    print(f"AV attempting to go {av_direction}, {epochs} times, {block} blocker")
    run_sim(epoch, epochs, running, success, av_direction, save_fails, include_stationary_vehicle=include_blocker)