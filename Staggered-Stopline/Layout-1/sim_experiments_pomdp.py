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

# Import POMDP components
from pomdp_unseen_cars_blocked_area_5 import UnseenCarPOMDPAgent, should_av_go_pomdp

parser = argparse.ArgumentParser(description="Run pygame app with options")
parser.add_argument("--blocker", action="store_true",
                    help="Add blocking vehicle to stop line")
parser.add_argument("--epochs", type=int, default=10,
                    help="Number of experiment attempts")
parser.add_argument("--creep", action="store_true",
                    help="Include if AV should use creeping behaviour to improve FOV.")
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
 
 
def reset_simulation(epoch, epochs, running, success, av_direction, inch_behave, save_fails, include_stationary_vehicle):
    time.sleep(1.5)
    run_sim(epoch, epochs, running, success, av_direction, inch_behave, save_fails, include_stationary_vehicle)

# Define a custom event
MY_EVENT = pygame.USEREVENT + 1

# Function to start a random timer
def set_decision_timer(min_ms, max_ms):
    delay = random.randint(min_ms, max_ms)
    pygame.time.set_timer(MY_EVENT, delay, loops=1)  # fire once
 
 
def run_sim(epoch, epochs, running, success, av_direction='straight', inch_behave=True, save_fails=False, include_stationary_vehicle=False):
    save_dir = './results/'
    save_name = f"epoch-{epoch}_direction-{av_direction}_blocker-{include_stationary_vehicle}.csv"
    save_fails = save_fails
    frames = []
    start_record = False

    print(f'Saving Collisions: {save_fails}')
    print("[INIT] Initializing POMDP Agent...")
    # Initialize POMDP agent
    pomdp_agent = UnseenCarPOMDPAgent(model_unseen_cars=True, enable_visibility_check=True)
    pomdp_agent.verbose = False
    pomdp_agent.policy.verbose = False
    print(f"[INIT] POMDP Agent initialized")
    print(f"[CONFIG] p_exist = {pomdp_agent.config.p_exist}")
    
    av = av_class.AutonomousVehicle(screen)
    av.inch_behave = inch_behave

    if include_stationary_vehicle:
        stationary_vehicle = blocker_vehicle_class.StationaryVehicle(screen)
    else:
        stationary_vehicle = None

    intersection_obstruction = None
    parked_vehicle = None
    cross_traffic = []
    running = True
    deciding = False
    set_decision_timer(2000, 7000)

    if epoch >= epochs:
        running = False
        print(f"[STATUS] Finished running {epochs} experiments!")
        print(f"[RESULTS] AV had a {(np.array(success).sum()/epochs)*100}% success rate trying to cross the intersection")
        np.savetxt(save_dir+save_name, np.array(success), delimiter=',')
        pygame.quit()
        sys.exit()
    else:
        print(f"[STATUS] Running experiment {epoch+1}")
    print(f"[STATUS] Waiting for timer")
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
            if event.type == MY_EVENT:
                av.manual_trigger = True
                print("[TRIGGER] AV is now allowed to attempt to go.")

        pomdp_agent.transition_model.config.enable_creep_improvement = av.inch_behave
        pomdp_agent.policy.config.enable_creep_improvement = av.inch_behave
        pomdp_agent.observation_model.config.enable_creep_improvement = av.inch_behave
        pomdp_agent.reward_model.config.enable_creep_improvement = av.inch_behave
        pomdp_agent.config.enable_creep_improvement = av.inch_behave

        # Spawn cross traffic
        if random.random() < (config.TRAFFIC_FLOW/3600)/config.FPS:
            direction = random.choice(['left', 'right'])
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
 
        # ============ POMDP DECISION MAKING ============
        if av.manual_trigger and not av.moving and not av.collided:
            # Prepare blockers list
            blockers = []
            if stationary_vehicle is not None:
                blockers.append(stationary_vehicle)
            if intersection_obstruction is not None:
                blockers.append(intersection_obstruction)
            if parked_vehicle is not None:
                blockers.append(parked_vehicle)
            
            # Use POMDP decision function
            should_go = should_av_go_pomdp(cross_traffic, av, blockers, pomdp_agent)

            if should_go:
                print(f"[STATUS] Attempting to navigate intersection")
                start_record = True
                
            if av.inching:
                start_record = True
            
            # The POMDP function handles av.moving and av.inching internally
            if not deciding and not should_go:
                deciding = True
        # ============================================
 
        av.update()
 
        if av.check_collision(cross_traffic):
            av.draw()
            for car in cross_traffic:
                car.draw()
            pygame.display.flip()
            print("\n[COLLISION DETECTED]")
            
            success.append(0)

            if save_fails:
                print('[INFO] Attempting to save collision replay')
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
                frames.append(frame)
                ####################################################################

                # for idx in range(90):
                #     frames.append(frame)

                print("[SAVING] Saving the screen recording from this collision")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # mp4 codec
                out = cv2.VideoWriter(save_dir+f"epoch-{epoch}_for_epochs-{epochs}_direction-{av_direction}_blocker-{include_stationary_vehicle}.mp4", fourcc, 60, (config.WIDTH, config.HEIGHT))
                for frame in frames:
                    bgr_frame = frame[:, :, ::-1]
                    out.write(bgr_frame)
                out.release()

            frames.clear()

            epoch += 1

            reset_simulation(epoch, epochs, running, success, av_direction, inch_behave, save_fails, include_stationary_vehicle)
            # break
 
        # Check for success (AV fully exited top of screen)
        if av.y + config.AV_HEIGHT < 250 or av.x < 0 or av.x > config.WIDTH:
            print('[OUTCOME] AV successfully navigated intersection!')
            success.append(1)
            epoch += 1
            frames.clear()
            reset_simulation(epoch, epochs, running, success, av_direction, inch_behave, save_fails, include_stationary_vehicle)
            # break

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

        pygame.display.flip()
        if start_record:
            frame = pygame.surfarray.array3d(screen)        # (w, h, 3)
            frame = np.transpose(frame, (1, 0, 2))
            frames.append(frame)

        clock.tick(config.FPS)
 
 
if __name__ == "__main__":
    # include_blocker = "--blocker" in sys.argv
    args = parser.parse_args()
    include_blocker = args.blocker
    save_fails = args.save_fails
    inch_behave = args.creep
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

    run_sim(epoch, epochs, running, success, av_direction, inch_behave, save_fails, include_stationary_vehicle=include_blocker)