import pygame
import random
import sys
import time
import config
import av_class
import blocker_vehicle_class
import cross_traffic_class
import utils
import ctypes

# Import POMDP components
from pomdp_unseen_cars_blocked_area_5 import UnseenCarPOMDPAgent, should_av_go_pomdp

on_off = ['OFF', 'ON']
on_off_colour = [(255,0,0), (0,255,0)]
 
print("[INIT] Starting POMDP-based intersection simulator")

pygame.init()
screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
pygame.display.set_caption("AV Intersection Simulator with POMDP")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)
font_small = pygame.font.SysFont(None, 22)

user32 = ctypes.WinDLL('user32')
kernel32 = ctypes.WinDLL('kernel32')

def focus_console():
    hWnd = kernel32.GetConsoleWindow()
    if hWnd:
        user32.SetForegroundWindow(hWnd)

def focus_pygame():
    hWnd = pygame.display.get_wm_info()['window']
    if hWnd:
        user32.SetForegroundWindow(hWnd)

def flash_screen(duration_ms=100, colour=(255,0,0)):
    start = pygame.time.get_ticks()
    while pygame.time.get_ticks() - start < duration_ms:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        screen.fill(colour)
        pygame.display.flip()
 
 
def reset_simulation():
    time.sleep(1.5)
    run_sim()
 
 
def run_sim(include_stationary_vehicle=False):
    print("[INIT] Initializing POMDP Agent...")
    
    # Initialize POMDP agent
    pomdp_agent = UnseenCarPOMDPAgent()
    print(f"[INIT] POMDP Agent initialized")
    print(f"[CONFIG] p_exist = {pomdp_agent.config.p_exist}")
    # print(f"[CONFIG] unseen_car_danger_weight = {pomdp_agent.config.unseen_car_danger_weight}")
    # print(f"[CONFIG] confidence_threshold_go = {pomdp_agent.config.confidence_threshold_go}")
    
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
            if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                print("[INPUT] Quitting simulation.")
                # Print POMDP statistics before quitting
                # summary = pomdp_agent.get_unseen_car_summary()
                # if summary['active']:
                #     print("\n" + "="*60)
                #     print("POMDP STATISTICS")
                #     print("="*60)
                #     print(f"Unseen car model activations: {summary['num_activations']}")
                #     print(f"Total danger added: {summary['total_danger_added']:.1f}")
                #     print(f"Average danger per activation: {summary['avg_danger_per_activation']:.1f}")
                #     print("="*60)
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

            if event.type == pygame.KEYDOWN and event.key == pygame.K_i:
                if av.inch_behave:
                    print("[TOGGLE] AV will no longer inch.")
                    av.inch_behave = False
                else:
                    av.inch_behave = True
                    print("[TOGGLE] AV will engage inching behaviour.")
 
            if event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                print("[INPUT] AV can't turn LEFT from here!")
                flash_screen()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                print("[INPUT] AV can't go backwards!")
                flash_screen()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                av.intended_direction = 'straight'
                print("[INPUT] AV intends to go STRAIGHT.")
            if event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                av.intended_direction = 'right'
                print("[INPUT] AV intends to turn RIGHT.")
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                print("[INPUT] Simulation reset.")
                reset_simulation()

            if event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                focus_console()
                selection = input("[INPUT] Select parameter to change (input number): \n (1) Traffic Flow \n (2) AV Speed \n (3) Cross-Traffic Speed \n (4) Exit \n Selection: ")
                
                if int(selection) == 1:
                    new_rate = input("[INPUT] Enter desired traffic flow (vehicles/hour): ")
                    try:
                        config.TRAFFIC_FLOW = float(new_rate)
                        print(f"[INPUT] Traffic flow updated: {config.TRAFFIC_FLOW}")
                    except ValueError:
                        print("[WARNING] Invalid input")
                        flash_screen()
                elif int(selection) == 2:
                    new_speed = input("[INPUT] Enter desired AV top speed (kilometres/hour): ")
                    try:
                        config.AV_SPEED = int((100*float(new_speed)/3.6)/(config.FPS*8))
                        print(f"[INPUT] AV top speed updated: {(3.6*config.AV_SPEED*config.FPS*8)/100}")
                        print(f"Disclaimer: rounding due to pixel size may round set speed")
                    except ValueError:
                        print("[WARNING] Invalid input")
                        flash_screen()
                elif int(selection) == 3:
                    new_speed = input("[INPUT] Enter desired cross-traffic top speed (kilometres/hour): ")
                    try:
                        config.CROSS_SPEED = int((100*float(new_speed)/3.6)/(config.FPS*8))
                        print(f"[INPUT] Cross-traffic top speed updated: {(3.6*config.CROSS_SPEED*config.FPS*8)/100}")
                        print(f"Disclaimer: rounding due to pixel size may round set speed")
                    except ValueError:
                        print("[WARNING] Invalid input")
                        flash_screen()
                elif int(selection) == 4:
                    print("[INPUT] Back to simulator.")
                else:
                    print("[WARNING] Invalid input")
                    flash_screen()

                focus_pygame()

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
            # # Print POMDP statistics
            # summary = pomdp_agent.get_unseen_car_summary()
            # if summary['active']:
            #     print(f"  Unseen car model was active")
            #     print(f"  Activations: {summary['num_activations']}")
            #     print(f"  Total danger added: {summary['total_danger_added']:.1f}")
            reset_simulation()
 
        # Check for success (AV fully exited top of screen)
        if av.y + config.AV_HEIGHT < 250 or av.x < 0 or av.x > config.WIDTH:
            print("[SUCCESS] AV successfully crossed the intersection!")
            # Print POMDP statistics
            # summary = pomdp_agent.get_unseen_car_summary()
            # if summary['active']:
            #     print(f"[STATS] Unseen car model helped navigate occlusion")
            #     print(f"[STATS] Activations: {summary['num_activations']}")
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
 
        if av.moving:
            decision_status = 'MOVING'
        elif av.inching:
            decision_status = 'CREEPING'
        elif av.manual_trigger:
            decision_status = 'WAITING'
        else:
            decision_status = 'STOPPED'
        status_text = font.render(
            f"AV {decision_status} (POMDP) | Press SPACE to GO", True, (0, 0, 0)
        )
        screen.blit(status_text, (10, 10))
        
        # Additional instructions
        sample = font_small.render(f"Press 'I' to toggle inching behaviour ", True, (0, 0, 0))
        on_off_start = 10+sample.get_width()
        
        instruction1 = font_small.render(f"Press 'B' to toggle blocking vehicle ", True, (0, 0, 0))
        instruction1_start = font_small.render(f"[", True, (0, 0, 0))
        instruction1_on = font_small.render(f"{on_off[int(stationary_vehicle is not None)]}", True, on_off_colour[int(stationary_vehicle is not None)])
        instruction1_end = font_small.render(f"]", True, (0, 0, 0))
        screen.blit(instruction1, (10, 35))
        screen.blit(instruction1_start, (on_off_start, 35))
        screen.blit(instruction1_on, (on_off_start+instruction1_start.get_width(), 35))
        screen.blit(instruction1_end, (on_off_start+instruction1_start.get_width()+instruction1_on.get_width(), 35))

        instruction1 = font_small.render(f"Press 'O' to toggle blocking object ", True, (0, 0, 0))
        instruction1_start = font_small.render(f"[", True, (0, 0, 0))
        instruction1_on = font_small.render(f"{on_off[int(intersection_obstruction is not None)]}", True, on_off_colour[int(intersection_obstruction is not None)])
        instruction1_end = font_small.render(f"]", True, (0, 0, 0))
        screen.blit(instruction1, (10, 55))
        screen.blit(instruction1_start, (on_off_start, 55))
        screen.blit(instruction1_on, (on_off_start+instruction1_start.get_width(), 55))
        screen.blit(instruction1_end, (on_off_start+instruction1_start.get_width()+instruction1_on.get_width(), 55))

        instruction1 = font_small.render(f"Press 'P' to toggle parked vehicle ", True, (0, 0, 0))
        instruction1_start = font_small.render(f"[", True, (0, 0, 0))
        instruction1_on = font_small.render(f"{on_off[int(parked_vehicle is not None)]}", True, on_off_colour[int(parked_vehicle is not None)])
        instruction1_end = font_small.render(f"]", True, (0, 0, 0))
        screen.blit(instruction1, (10, 75))
        screen.blit(instruction1_start, (on_off_start, 75))
        screen.blit(instruction1_on, (on_off_start+instruction1_start.get_width(), 75))
        screen.blit(instruction1_end, (on_off_start+instruction1_start.get_width()+instruction1_on.get_width(), 75))

        instruction1 = font_small.render(f"Press 'I' to toggle inching behaviour ", True, (0, 0, 0))
        instruction1_start = font_small.render(f"[", True, (0, 0, 0))
        instruction1_on = font_small.render(f"{on_off[int(av.inch_behave)]}", True, on_off_colour[int(av.inch_behave)])
        instruction1_end = font_small.render(f"]", True, (0, 0, 0))
        screen.blit(instruction1, (10, 95))
        screen.blit(instruction1_start, (on_off_start, 95))
        screen.blit(instruction1_on, (on_off_start+instruction1_start.get_width(), 95))
        screen.blit(instruction1_end, (on_off_start+instruction1_start.get_width()+instruction1_on.get_width(), 95))

        instruction2 = font_small.render(f"Press Arrows to alter AV path ", True, (0, 0, 0))
        screen.blit(instruction2, (10, 115))
        instruction2_ = font_small.render(f"[{av.intended_direction.capitalize()}]", True, (0, 0, 0))
        screen.blit(instruction2_, (28+instruction2.get_width(), 115))

        instruction3 = font.render("Press 'R' to reset", True, (0, 0, 0))
        screen.blit(instruction3, (10+config.WIDTH//2+int(2*config.LANE_WIDTH), 35))

        instruction5 = font.render("Press 'Q' to quit", True, (0, 0, 0))
        screen.blit(instruction5, (10+config.WIDTH//2+int(2*config.LANE_WIDTH), 60))

        sample = font_small.render(f"(3) Cross-Traffic Speed ", True, (0, 0, 0))
        param_x = 30+config.WIDTH//2+int(2*config.LANE_WIDTH)+sample.get_width()

        instruction4 = font.render(f"Press 'S' to change:", True, (0, 0, 0))
        screen.blit(instruction4, (10+config.WIDTH//2+int(2*config.LANE_WIDTH), 85))
        instruction4_ = font_small.render(f"(1) Traffic Flow", True, (0, 0, 0))
        screen.blit(instruction4_, (30+config.WIDTH//2+int(2*config.LANE_WIDTH), 105))
        instruction4_ = font_small.render(f"[{config.TRAFFIC_FLOW} v/hr]", True, (0, 0, 0))
        screen.blit(instruction4_, (param_x, 105))
        instruction4_ = font_small.render(f"(2) AV Speed", True, (0, 0, 0))
        screen.blit(instruction4_, (30+config.WIDTH//2+int(2*config.LANE_WIDTH), 125))
        instruction4_ = font_small.render(f"[{(config.AV_SPEED*8*config.FPS*3.6)/100:.0f} km/hr]", True, (0, 0, 0))
        screen.blit(instruction4_, (param_x, 125))
        instruction4_ = font_small.render(f"(3) Cross-Traffic Speed", True, (0, 0, 0))
        screen.blit(instruction4_, (30+config.WIDTH//2+int(2*config.LANE_WIDTH), 145))
        instruction4_ = font_small.render(f"[{(config.CROSS_SPEED*8*config.FPS*3.6)/100:.0f} km/hr]", True, (0, 0, 0))
        screen.blit(instruction4_, (param_x, 145))
        
        # Display POMDP info
        # belief_text = font_small.render(f"POMDP Belief: {pomdp_agent.policy.get_belief_summary(pomdp_agent.belief)}", True, (0, 0, 0))
        # screen.blit(belief_text, (10, config.HEIGHT - 45))
        
        # step_text = font_small.render(f"POMDP Steps: {pomdp_agent.policy.step_count}", True, (0, 0, 0))
        # screen.blit(step_text, (10, config.HEIGHT - 25))

        pygame.display.flip()
        clock.tick(config.FPS)
 
 
if __name__ == "__main__":
    include_blocker = "--blocker" in sys.argv
    run_sim(include_stationary_vehicle=include_blocker)