import pomdp_py
import random
import math
import config as global_config
import numpy as np
from utils import travel_time, is_car_in_fov

# ============================================================================
# State Definition
# ============================================================================
class AVState(pomdp_py.State):
    """State of the autonomous vehicle and environment."""
    def __init__(self, av, cross_traffic_vehicles, blocking_objects):
        """
        Args:
            av: AutonomousVehicle object from simulation
            cross_traffic_vehicles: list of CrossTrafficCar objects (visible vehicles only)
            blocking_objects: list of blocking objects (ParkedVehicle, IntersectionObstruction, StationaryVehicle)
        """
        # Extract AV state
        self.av_pos = av.y  # Y position (lower = closer to stop line)
        self.av_vel = av.vy  # Y velocity (negative = moving forward/up)
        # try:
        self.av_col_zone_times = av.col_zone_times
        self.inch_behave = av.inch_behave
        # except Exception as e:
        #     print(e)
        #     self.av_col_zone_times = np.array([[float('inf'), float('inf')], [float('inf'), float('inf')]])


        
        # Extract cross traffic state - sort by x position and direction
        self.cross_traffic = tuple(sorted(
            [(car.x, car.direction, car.drive_path, car.max_speed, car.y) 
             for car in cross_traffic_vehicles],
            key=lambda v: (v[1], v[0])  # Sort by direction then position
        ))
        
        # Extract blocking objects state
        self.blocking_objects = tuple(sorted(
            [(obj.x, obj.y, type(obj).__name__) for obj in blocking_objects],
            key=lambda b: (b[0], b[1])
        ))
    
    def __str__(self):
        traffic_str = [(f'x={x:.0f}', dir, path, f'v={v:.1f}', f'y={y:.0f}') for x, dir, path, v, y in self.cross_traffic]
        block_str = [(f'x={x:.0f}', f'y={y:.0f}', name) for x, y, name in self.blocking_objects]
        return f"AVState(av_y={self.av_pos:.0f}, av_vy={self.av_vel:.1f}, creep={self.inch_behave:.1f}, " \
               f"col_times={self.av_col_zone_times}, traffic={traffic_str}, blocks={block_str})"
    
    def __repr__(self):
        return str(self)
    
    def __hash__(self):
        traffic_hash = tuple((round(x, 0), d, p, round(v, 1), round(y,0)) for x, d, p, v, y in self.cross_traffic)
        block_hash = tuple((round(x, 0), round(y, 0), n) for x, y, n in self.blocking_objects)
        col_times_hash = tuple((round(t1, 1), round(t2, 1)) for t1, t2 in self.av_col_zone_times)
        return hash((round(self.av_pos, 0), round(self.av_vel, 1), self.inch_behave, col_times_hash, traffic_hash, block_hash))
    
    def __eq__(self, other):
        if not isinstance(other, AVState):
            return False
        return (round(self.av_pos, 0) == round(other.av_pos, 0) and
                round(self.av_vel, 1) == round(other.av_vel, 1) and
                round(self.inch_behave, 0) == round(other.inch_behave, 0) and
                all(round(t1[0], 1) == round(t2[0], 1) and round(t1[1], 1) == round(t2[1], 1)
                    for t1, t2 in zip(self.av_col_zone_times, other.av_col_zone_times)) and
                len(self.cross_traffic) == len(other.cross_traffic) and
                all(round(x1, 0) == round(x2, 0) and d1 == d2 and p1 == p2 and round(v1, 1) == round(v2, 1) and round(y1, 1) == round(y2, 1)
                    for (x1, d1, p1, v1, y1), (x2, d2, p2, v2, y2) in zip(self.cross_traffic, other.cross_traffic)) and
                len(self.blocking_objects) == len(other.blocking_objects) and
                all(round(x1, 0) == round(x2, 0) and round(y1, 0) == round(y2, 0) and n1 == n2
                    for (x1, y1, n1), (x2, y2, n2) in zip(self.blocking_objects, other.blocking_objects)))

# ============================================================================
# Action Definition
# ============================================================================
class AVAction(pomdp_py.Action):
    """Actions available to the AV."""
    def __init__(self, name):
        """
        Args:
            name: str, one of ['go', 'stop', 'creep']
        """
        self.name = name
    
    def __str__(self):
        return self.name
    
    def __repr__(self):
        return self.name
    
    def __hash__(self):
        return hash(self.name)
    
    def __eq__(self, other):
        if not isinstance(other, AVAction):
            return False
        return self.name == other.name

# ============================================================================
# Observation Definition
# ============================================================================
class AVObservation(pomdp_py.Observation):
    """Observation of the environment."""
    def __init__(self, cross_traffic_obs, blocking_obs):
        """
        Args:
            cross_traffic_obs: list of (x, direction, path, speed) tuples or None
            blocking_obs: list of (x, y, type_name) tuples or None
        """
        self.cross_traffic_obs = tuple(sorted(cross_traffic_obs, key=lambda v: (v[1], v[0]))) if cross_traffic_obs else None
        self.blocking_obs = tuple(sorted(blocking_obs, key=lambda b: (b[0], b[1]))) if blocking_obs else None
    
    def __str__(self):
        traffic_str = [(f'x={x:.0f}', d, p, f'y={y:.0f}') for x, d, p, _, y in self.cross_traffic_obs] if self.cross_traffic_obs else None
        block_str = [(f'x={x:.0f}', f'y={y:.0f}', n) for x, y, n in self.blocking_obs] if self.blocking_obs else None
        return f"Obs(traffic={traffic_str}, blocks={block_str})"
    
    def __repr__(self):
        return str(self)
    
    def __hash__(self):
        traffic_hash = tuple((round(x, 0), d, p, round(v, 1), round(y, 0)) for x, d, p, v, y in self.cross_traffic_obs) if self.cross_traffic_obs else None
        block_hash = tuple((round(x, 0), round(y, 0), n) for x, y, n in self.blocking_obs) if self.blocking_obs else None
        return hash((traffic_hash, block_hash))
    
    def __eq__(self, other):
        if not isinstance(other, AVObservation):
            return False
        
        # Check cross traffic
        if self.cross_traffic_obs is None or other.cross_traffic_obs is None:
            if not (self.cross_traffic_obs is None and other.cross_traffic_obs is None):
                return False
        elif (len(self.cross_traffic_obs) != len(other.cross_traffic_obs) or
              not all(round(x1, 0) == round(x2, 0) and d1 == d2 and p1 == p2 and round(v1, 1) == round(v2, 1) and round(y1, 1) == round(y2, 1)
                     for (x1, d1, p1, v1, y1), (x2, d2, p2, v2, y2) in zip(self.cross_traffic_obs, other.cross_traffic_obs))):
            return False
        
        # Check blocking objects
        if self.blocking_obs is None or other.blocking_obs is None:
            if not (self.blocking_obs is None and other.blocking_obs is None):
                return False
        elif (len(self.blocking_obs) != len(other.blocking_obs) or
              not all(round(x1, 0) == round(x2, 0) and round(y1, 0) == round(y2, 0) and n1 == n2
                     for (x1, y1, n1), (x2, y2, n2) in zip(self.blocking_obs, other.blocking_obs))):
            return False
        
        return True

# ============================================================================
# Transition Model
# ============================================================================
class AVTransitionModel(pomdp_py.TransitionModel):
    """Defines state transitions based on actions."""
    
    DT = 1.0  # time step (seconds)
    
    def __init__(self, av_config):
        """
        Args:
            av_config: dict with keys like 'stop_line_y', 'creep_limit_y', 'intersection_exit_y', 
                      'go_accel', 'stop_accel', 'creep_speed', 'creep_accel'
        """
        self.stop_line_y = av_config.get('stop_line_y', 400)  # Y position of stop line
        self.creep_limit_y = av_config.get('creep_limit_y', 380)  # Y position of creep limit
        self.intersection_exit_y = av_config.get('intersection_exit_y', 300)  # Y position of exit
        
        self.go_accel = av_config.get('go_accel', 0.08)  # acceleration for 'go'
        self.stop_accel = av_config.get('stop_accel', -0.1)  # deceleration for 'stop'
        self.creep_speed = av_config.get('creep_speed', -0.5)  # target speed for 'creep'
        self.creep_accel = av_config.get('creep_accel', 0.05)  # acceleration for 'creep'
    
    def probability(self, next_state, state, action):
        """Returns probability of transitioning to next_state given state and action."""
        # Deterministic transitions
        return 1.0 if self._would_transition_to(state, action, next_state) else 0.0
    
    def _would_transition_to(self, state, action, next_state):
        """Check if state would transition to next_state under action."""
        expected = self.sample(state, action)
        return next_state == expected
    
    # def sample(self, state, action):
    #     """Sample next state given current state and action."""
    #     new_vel = state.av_vel
        
    #     if action.name == 'go':
    #         # Accelerate upward (more negative)
    #         new_vel = state.av_vel - self.go_accel
    #         new_vel = max(-5.0, new_vel)  # Cap at max speed
    #     elif action.name == 'stop':
    #         # Decelerate toward zero
    #         if state.av_vel < 0:
    #             new_vel = min(0.0, state.av_vel - self.stop_accel)
    #         else:
    #             new_vel = 0.0
    #     elif action.name == 'creep':
    #         # Accelerate slowly to creep speed (negative)
    #         if state.av_vel > self.creep_speed:
    #             new_vel = max(self.creep_speed, state.av_vel - self.creep_accel)
    #         else:
    #             new_vel = self.creep_speed
        
    #     # Update AV position (negative velocity decreases Y)
    #     new_pos = state.av_pos + new_vel
        
    #     # Creep action stops at creep limit
    #     if action.name == 'creep' and new_pos < self.creep_limit_y:
    #         new_pos = self.creep_limit_y
    #         new_vel = 0.0
        
    #     # Cap position at intersection exit
    #     new_pos = max(new_pos, self.intersection_exit_y)
        
    #     # Update cross traffic positions
    #     new_cross_traffic = []
    #     for x, direction, path, speed, y in state.cross_traffic:
    #         if direction == 'right':
    #             new_x = x + speed  # Moving right (positive)
    #         else:  # 'left'
    #             new_x = x - speed  # Moving left (negative)
    #         new_cross_traffic.append((new_x, direction, path, speed, y))
        
    #     # Blocking objects remain constant
    #     new_blocking = list(state.blocking_objects)

    #     new_col_zone_times = state.av_col_zone_times
        
    #     # Create mock objects for the new state (simplified - just tuples)
    #     # In practice, you'd reconstruct proper objects or pass references
    #     return AVState(
    #         type('AV', (), {'y': new_pos, 'vy': new_vel, 'col_zone_times': new_col_zone_times})(),
    #         [type('Car', (), {'x': x, 'direction': d, 'drive_path': p, 'max_speed': v, 'y': y})() 
    #          for x, d, p, v, y in new_cross_traffic],
    #         [type(n, (), {'x': x, 'y': y})() for x, y, n in new_blocking]
    #     )

    def sample(self, state, action):
        """Sample next state with action preconditions enforced."""
        new_vel = state.av_vel
        
        # ACTION PRECONDITIONS
        is_stopped = abs(state.av_vel) < 0.1
        
        if action.name == 'go':
            # Go always accelerates (valid from any state)
            new_vel = state.av_vel - self.go_accel
            new_vel = max(-5.0, new_vel)
            
        elif action.name == 'stop':
            # Can only stop if currently stopped or very slow
            if is_stopped:
                new_vel = 0.0
            else:
                # Invalid action - maintain current velocity (or keep going)
                new_vel = state.av_vel - self.go_accel  # Continue accelerating
                new_vel = max(-5.0, new_vel)
                
        elif action.name == 'creep':
            # Can only creep from stopped position
            if is_stopped or state.av_vel > -1.0:  # Stopped or already creeping
                if state.av_vel > self.creep_speed:
                    new_vel = max(self.creep_speed, state.av_vel - self.creep_accel)
                else:
                    new_vel = self.creep_speed
            else:
                # Invalid - continue current motion
                new_vel = state.av_vel - self.go_accel
                new_vel = max(-5.0, new_vel)
        
        # Update AV position (negative velocity decreases Y)
        new_pos = state.av_pos + new_vel
        
        # Creep action stops at creep limit
        if action.name == 'creep' and new_pos < self.creep_limit_y:
            new_pos = self.creep_limit_y
            new_vel = 0.0
        
        # Cap position at intersection exit
        new_pos = max(new_pos, self.intersection_exit_y)
        
        # Update cross traffic positions
        new_cross_traffic = []
        for x, direction, path, speed, y in state.cross_traffic:
            if direction == 'right':
                new_x = x + speed  # Moving right (positive)
            else:  # 'left'
                new_x = x - speed  # Moving left (negative)
            new_cross_traffic.append((new_x, direction, path, speed, y))
        
        # Blocking objects remain constant
        new_blocking = list(state.blocking_objects)

        # new_col_zone_times = state.av_col_zone_times
        new_col_zone_times = np.array([[travel_time(new_pos-global_config.LOWER_CONFLICT_ZONE[3], abs(new_vel), abs(self.go_accel), global_config.AV_SPEED),
                               travel_time((new_pos+global_config.AV_HEIGHT)-global_config.LOWER_CONFLICT_ZONE[2], abs(new_vel), abs(self.go_accel), global_config.AV_SPEED)],
                              [travel_time(new_pos-global_config.UPPER_CONFLICT_ZONE[3], abs(new_vel), abs(self.go_accel), global_config.AV_SPEED),
                               travel_time((new_pos+global_config.AV_HEIGHT)-global_config.UPPER_CONFLICT_ZONE[2], abs(new_vel), abs(self.go_accel), global_config.AV_SPEED)]])
        # self.col_zone_times[0,0] = travel_time(self.y-config.LOWER_CONFLICT_ZONE[3], abs(self.vy), abs(self.acceleration), config.AV_SPEED)
        # self.col_zone_times[0,1] = travel_time((self.y+config.AV_HEIGHT)-config.LOWER_CONFLICT_ZONE[2], abs(self.vy), abs(self.acceleration), config.AV_SPEED)
        # self.col_zone_times[1,0] = travel_time(self.y-config.UPPER_CONFLICT_ZONE[3], abs(self.vy), abs(self.acceleration), config.AV_SPEED)
        # self.col_zone_times[1,1] = travel_time((self.y+config.AV_HEIGHT)-config.UPPER_CONFLICT_ZONE[2], abs(self.vy), abs(self.acceleration), config.AV_SPEED)
        
        # Create mock objects for the new state (simplified - just tuples)
        # In practice, you'd reconstruct proper objects or pass references
        return AVState(
            type('AV', (), {'y': new_pos, 'vy': new_vel, 'col_zone_times': new_col_zone_times, 'inch_behave': state.inch_behave})(),
            [type('Car', (), {'x': x, 'direction': d, 'drive_path': p, 'max_speed': v, 'y': y})() 
             for x, d, p, v, y in new_cross_traffic],
            [type(n, (), {'x': x, 'y': y})() for x, y, n in new_blocking]
        )
    
    def get_all_states(self):
        """Get discretized state space (simplified)."""
        # This would need to be customized based on your specific discretization needs
        return []

# ============================================================================
# Observation Model
# ============================================================================
class AVObservationModel(pomdp_py.ObservationModel):
    """Defines observation probabilities."""
    
    def __init__(self, obs_noise=5.0):
        """
        Args:
            obs_noise: standard deviation of observation noise (pixels)
        """
        self.obs_noise = obs_noise
    
    def probability(self, observation, next_state, action):
        """Returns probability of observation given next_state."""
        # Blocking objects observed perfectly
        if observation.blocking_obs != next_state.blocking_objects:
            return 0.0
        
        if observation.cross_traffic_obs is None:
            return 0.01
        
        # Check we observe the right number of vehicles
        if len(observation.cross_traffic_obs) != len(next_state.cross_traffic):
            return 0.0
        
        # Gaussian noise on each vehicle's observation
        prob = 1.0
        for (obs_x, obs_d, obs_p, obs_v, obs_y), (true_x, true_d, true_p, true_v, true_y) in \
                zip(observation.cross_traffic_obs, next_state.cross_traffic):
            if obs_d != true_d or obs_p != true_p:
                return 0.0
            diff = abs(obs_x - true_x)
            prob *= math.exp(-0.5 * (diff / self.obs_noise) ** 2)
            
            speed_diff = abs(obs_v - true_v)
            prob *= math.exp(-0.5 * (speed_diff / 0.5) ** 2)  # Speed noise
        
        return max(0.001, prob)
    
    def sample(self, next_state, action):
        """Sample observation given next_state."""
        # Blocking objects observed perfectly
        blocking_obs = list(next_state.blocking_objects)
        
        # Cross traffic observed with Gaussian noise on position
        cross_obs = [(x + random.gauss(0, self.obs_noise), direction, path, 
                     speed + random.gauss(0, 0.3), y)
                    for x, direction, path, speed, y in next_state.cross_traffic]
        
        return AVObservation(cross_obs, blocking_obs)

# ============================================================================
# Reward Model
# ============================================================================
class AVRewardModel(pomdp_py.RewardModel):
    """Defines rewards for state-action pairs."""
    
    def __init__(self, config):
        """
        Args:
            config: dict with intersection geometry and reward parameters
        """
        self.stop_line_y = config.get('stop_line_y', 400)
        self.creep_limit_y = config.get('creep_limit_y', 380)
        self.conflict_zone_start_y = config.get('conflict_zone_start_y', 370)
        self.conflict_zone_end_y = config.get('conflict_zone_end_y', 320)
        self.intersection_exit_y = config.get('intersection_exit_y', 300)
        self.intersection_center_x = config.get('intersection_center_x', 400)
        
        self.collision_penalty = config.get('collision_penalty', -1000)
        self.goal_reward = config.get('goal_reward', 100) 
        self.time_penalty = config.get('time_penalty', -1)
        self.safe_time_gap = config.get('safe_time_gap', 2.5)
        self.current_reward = 0
    
    def _check_collision(self, state):
        """Check if AV collides with any cross traffic."""
        # AV is in conflict zone
        in_conflict = self.conflict_zone_end_y <= state.av_pos <= self.conflict_zone_start_y
        
        if not in_conflict:
            return False
        
        # Define collision zone around intersection center (±50 pixels)
        collision_x_min = self.intersection_center_x - 25
        collision_x_max = self.intersection_center_x + 25
        
        # Check collision with any vehicle in the conflict zone
        for x, direction, path, speed, y in state.cross_traffic:
            if (collision_x_min <= x <= collision_x_max) and ((global_config.HEIGHT//2 - global_config.LANE_WIDTH) <= y <= (global_config.HEIGHT//2 + global_config.LANE_WIDTH)):
                return True
        
        return False
    
    def _get_conflict_window(self, vehicle_x, direction, speed, vehicle_y):
        """Calculate entry and exit times for cross-traffic vehicle in conflict zone."""
        # # Handle zero speed case
        # if speed <= 0:
        #     # If stationary, check if already in conflict zone
        #     conflict_min = self.intersection_center_x - 25
        #     conflict_max = self.intersection_center_x + 25
        #     if conflict_min <= vehicle_x <= conflict_max:
        #         return (0.0, float('inf'))  # Always in conflict
        #     else:
        #         return (float('inf'), float('inf'))  # Never in conflict
        
        conflict_min = self.intersection_center_x - 25
        conflict_max = self.intersection_center_x + 25
        
        if direction == 'right':
            # Moving right (positive direction)
            if ((vehicle_x+global_config.CAR_WIDTH) > conflict_max) or global_config.HEIGHT//2 <= vehicle_y or vehicle_y <=(global_config.HEIGHT//2 - 50):
                return (float('inf'), float('inf'))
            
            if vehicle_x+global_config.CAR_WIDTH < conflict_min:
                # Approaching from left
                entry_time = (conflict_min - (vehicle_x+global_config.CAR_WIDTH)) / speed
                exit_time = (conflict_max - vehicle_x) / speed
                return (entry_time, exit_time)
            elif vehicle_x <= conflict_max:
                # Already in conflict zone
                exit_time = (conflict_max - vehicle_x) / speed
                return (0.0, exit_time)
            else:
                # Past conflict zone
                return (float('inf'), float('inf'))
        else:  # 'left'
            # Moving left (negative direction)
            if ((vehicle_x+global_config.CAR_WIDTH) < conflict_min) or (global_config.HEIGHT//2 + 50) <= vehicle_y or vehicle_y<= global_config.HEIGHT//2:
                return (float('inf'), float('inf'))
            
            if vehicle_x > conflict_max:
                # Approaching from right
                entry_time = (vehicle_x - conflict_max) / speed
                exit_time = ((vehicle_x+global_config.CAR_WIDTH) - conflict_min) / speed
                return (entry_time, exit_time)
            elif (vehicle_x+global_config.CAR_WIDTH) >= conflict_min:
                # Already in conflict zone
                exit_time = ((vehicle_x+global_config.CAR_WIDTH) - conflict_min) / speed
                return (0.0, exit_time)
            else:
                # Past conflict zone
                return (float('inf'), float('inf'))
    
    def _get_av_conflict_window(self, state):
        """Calculate entry and exit times for AV in conflict zone."""
        # AV conflict zone boundaries
        entry_y = self.conflict_zone_start_y
        exit_y = self.conflict_zone_end_y
        
        current_y = state.av_pos
        current_vy = state.av_vel
        
        # If AV is stationary or moving wrong direction
        if current_vy >= 0:
            if self.conflict_zone_end_y <= current_y <= self.conflict_zone_start_y:
                return (0.0, float('inf'))  # Stuck in conflict zone
            else:
                return (float('inf'), float('inf'))  # Will never enter
        
        # Calculate time to reach entry and exit points
        # Assume constant acceleration from 'go' action
        go_accel = abs(self.stop_line_y - self.creep_limit_y) * 0.001  # Rough estimate
        
        # Simple approximation: use average velocity
        # More accurate would use kinematic equations with acceleration
        if current_y > entry_y:
            # Before conflict zone
            distance_to_entry = current_y - entry_y
            distance_to_exit = current_y - exit_y
            
            # Estimate assuming acceleration to max speed
            # t = (-v0 + sqrt(v0^2 + 2*a*d)) / a for constant acceleration
            # Simplified: use current velocity and assume it increases
            avg_speed = max(abs(current_vy), 2.0)  # Minimum assumed speed
            entry_time = distance_to_entry / avg_speed
            exit_time = distance_to_exit / avg_speed
            return (entry_time, exit_time)
        elif current_y > exit_y:
            # In conflict zone
            distance_to_exit = current_y - exit_y
            avg_speed = max(abs(current_vy), 2.0)
            exit_time = distance_to_exit / avg_speed
            return (0.0, exit_time)
        else:
            # Past conflict zone
            return (float('inf'), float('inf'))
    
    def _is_safe_to_go(self, state):
        """Check if there's no overlap between AV and cross-traffic conflict windows."""
        # Get AV's time window in conflict zone
        # av_entry, av_exit = self._get_av_conflict_window(state)
        # av_entry = [0][0]
        # av_exit = state.av.col_zone_times[1,1]
        
        # If AV can't enter conflict zone, not safe
        if (state.av_col_zone_times == float('inf')).any():
            print('Problem with AV collision zone times')
            # print(state.av_col_zone_times)
            return False
        
        # Check each cross-traffic vehicle
        for x, direction, path, speed, y in state.cross_traffic:

            if state.av_vel == 0 and (global_config.WIDTH//2-global_config.LANE_WIDTH <= x <=global_config.WIDTH//2+global_config.LANE_WIDTH):
                return False

            ct_entry, ct_exit = self._get_conflict_window(x, direction, speed, y)

            if ct_entry == float('inf') and ct_exit == float('inf'):
                continue
            # Check for temporal overlap
            # Two intervals [a1, a2] and [b1, b2] overlap if: a1 < b2 AND b1 < a2
            if direction == 'right':
                if state.av_col_zone_times[1][0] < ct_exit and ct_entry < state.av_col_zone_times[1][1]:
                    return False  # Conflict windows overlap
            else:
                if state.av_col_zone_times[0][0] < ct_exit and ct_entry < state.av_col_zone_times[0][1]:
                    return False  # Conflict windows overlap
        
        # print('POMDP thinks it is safe')
        return True
    
    def sample(self, state, action, next_state):
        """Sample reward for transition."""

        # Check for collision
        if self._check_collision(next_state):
            # print('[REWARD] Collision check triggered')
            return self.collision_penalty
        
        # Reached goal
        if next_state.av_pos <= self.intersection_exit_y:
            # print('[REWARD] End goal triggered')
            return self.goal_reward
        
        # Base time penalty
        reward = self.time_penalty
        
        # Large penalty for creeping when already at intersection edge
        if action.name == 'creep' and state.av_pos <= self.creep_limit_y:
            reward -= 50  # Large penalty to discourage creeping past the edge
        
        # Reward strategic creeping with blocking objects (only if not at edge yet)
        if action.name == 'creep' and len(next_state.blocking_objects) > 0 and state.av_pos > self.creep_limit_y:
            reward += 5
            # Extra reward for reaching good observation position
            if next_state.av_pos <= self.creep_limit_y + 5:
                reward += 3

        is_safe_to_go = self._is_safe_to_go(next_state)
        # print(f'[REWARD] Safe to go: {is_safe_to_go}')
        
        # Reward safe execution
        if action.name == 'go' and is_safe_to_go:
            reward += 100
        
        # Reward waiting when unsafe
        if action.name == 'stop':
            if is_safe_to_go:
                reward -= 10
            else:
                reward += 5
        
        # Penalize unnecessary waiting when safe
        if (action.name == 'stop' and is_safe_to_go and 
            next_state.av_pos > self.creep_limit_y):
            reward -= 2

        self.current_reward = reward
        
        return reward

# ============================================================================
# Policy Model
# ============================================================================
# class AVPolicyModel(pomdp_py.RolloutPolicy):
#     """Simple random policy for rollouts."""
    
#     def __init__(self):
#         self.actions = [AVAction('go'), AVAction('stop'), AVAction('creep')]
    
#     def sample(self, state):
#         return random.choice(self.actions)
    
#     def rollout(self, state, history=None):
#         return random.choice(self.actions)
    
#     def get_all_actions(self, state=None, history=None):
#         return self.actions
class AVPolicyModel(pomdp_py.RolloutPolicy):
    """Policy that respects action preconditions."""
    
    def __init__(self):
        self.all_actions = [AVAction('go'), AVAction('stop'), AVAction('creep')]
    
    def get_all_actions(self, state=None, history=None):
        """Return only valid actions for current state."""
        if state is None:
            return self.all_actions
        
        valid_actions = []
        
        # Can only creep from a stop (velocity ~ 0)
        if abs(state.av_vel) < 0.1:  # Essentially stopped
            if state.inch_behave and state.av_pos > global_config.HEIGHT//2+global_config.LANE_WIDTH+1:
                valid_actions.append(AVAction('creep'))
            valid_actions.append(AVAction('go'))
            valid_actions.append(AVAction('stop'))
        # Once moving (go or creep), cannot stop
        else:
            valid_actions.append(AVAction('go'))
            # If creeping, can continue creeping
            if state.av_vel > -1.0:  # Creep speed range
                if state.inch_behave and state.av_pos > global_config.HEIGHT//2+global_config.LANE_WIDTH+1:
                    valid_actions.append(AVAction('creep'))
        
        return valid_actions if valid_actions else [AVAction('go')]
    
    def sample(self, state):
        valid = self.get_all_actions(state)
        return random.choice(valid)
    
    def rollout(self, state, history=None):
        valid = self.get_all_actions(state)
        # print(valid)
        return random.choice(valid)

# ============================================================================
# POMDP Problem
# ============================================================================
class AVIntersectionProblem(pomdp_py.POMDP):
    """Complete POMDP problem definition."""
    
    def __init__(self, av, cross_traffic_vehicles, blocking_objects, config):
        """
        Args:
            av: AutonomousVehicle object
            cross_traffic_vehicles: list of CrossTrafficCar objects
            blocking_objects: list of blocking objects
            config: dict with configuration parameters
        """
        init_state = AVState(av, cross_traffic_vehicles, blocking_objects)
        
        agent = pomdp_py.Agent(
            pomdp_py.Histogram({init_state: 1.0}),
            AVPolicyModel(),
            AVTransitionModel(config),
            AVObservationModel(config.get('obs_noise', 5.0)),
            AVRewardModel(config)
        )
        env = pomdp_py.Environment(
            init_state,
            AVTransitionModel(config),
            AVRewardModel(config)
        )
        super().__init__(agent, env, name="AVIntersection")

# ============================================================================
# Helper Class for Easy Integration
# ============================================================================
class AVIntersectionPlanner:
    """Wrapper class for easy integration with simulation."""
    
    def __init__(self, config, num_sims=100, max_depth=10, exploration_const=50, num_particles=100):
        """
        Args:
            config: configuration dictionary
            num_sims: number of POMCP simulations per planning step (lower = faster)
            max_depth: maximum planning depth (lower = faster)
            exploration_const: exploration constant for POMCP
            num_particles: number of particles for belief representation (lower = faster)
        """
        self.config = config
        self.num_particles = num_particles
        # Create POMCP with custom rollout policy
        self.pomcp = pomdp_py.POMCP(
            max_depth=max_depth,
            discount_factor=0.95,
            num_sims=num_sims,
            exploration_const=exploration_const,
            rollout_policy=AVPolicyModel()
        )
        self.problem = None
        self.count = 0
        self.exploration_const = exploration_const
        self.max_depth = max_depth
        self.num_sims = num_sims
    
    def get_action(self, av, cross_traffic, blocking_objects):
        """
        Get action for current time step.
        
        Args:
            av: AutonomousVehicle object
            cross_traffic_vehicles: list of visible CrossTrafficCar objects
            blocking_objects: list of blocking objects
            
        Returns:
            str: action name ('go', 'stop', or 'creep')
        """
        cross_traffic_vehicles = []
        for car in cross_traffic:
            if is_car_in_fov(car, av, blocking_objects):
                cross_traffic_vehicles.append(car)
        # Create new state
        new_state = AVState(av, cross_traffic_vehicles, blocking_objects)
        
        # Create or update problem
        if self.problem is None or self.count == 0:
            print('Creating new pomdp')
            self.problem = AVIntersectionProblem(av, cross_traffic_vehicles, 
                                                blocking_objects, self.config)
            # Convert initial belief to particles
            particles = [new_state for _ in range(self.num_particles)]
            self.problem.agent.set_belief(pomdp_py.Particles(particles))
        else:
            # Recreate environment with new state
            self.problem.env = pomdp_py.Environment(
                new_state,
                self.problem.env.transition_model,
                self.problem.env.reward_model
            )
            # Create particle belief centered on current state
            particles = [new_state for _ in range(self.num_particles)]
            self.problem.agent.set_belief(pomdp_py.Particles(particles))

        # print(f"Step debug - AV y:{av.y:.1f} vy:{av.vy:.2f}")
        # print(f"Col zone times: {av.col_zone_times}")
        # print(f"Cross traffic count: {len(cross_traffic_vehicles)}")
        # print(f"Is safe: {self.problem.env.reward_model._is_safe_to_go(new_state)}")
        self.count += 1
        if self.count == 300:
            # self.count = 0
            # self.pomcp = pomdp_py.POMCP(
            #     max_depth=self.max_depth,
            #     discount_factor=0.95,
            #     num_sims=self.num_sims,
            #     exploration_const=self.exploration_const,
            #     rollout_policy=AVPolicyModel()
            # )
            self.count = 0
        
        # Plan and return action
        action = self.pomcp.plan(self.problem.agent)
        # if action.name == 'stop':
        #     self.problem.agent.reward_model.time_penalty -= 1
            # self.pomcp.exploration_const += 10
            # print(f'Current time penalty: {self.problem.agent.reward_model.time_penalty}')
        # print(f"POMCP chose: {action.name}, Current reward from last step: {self.problem.agent.reward_model.current_reward}")
        return action.name

# ============================================================================
# Example Usage
# ============================================================================
if __name__ == "__main__":
    # Mock objects for demonstration
    class MockAV:
        def __init__(self):
            self.x = 350
            self.y = 400
            self.vx = 0
            self.vy = 0
    
    class MockCrossTraffic:
        def __init__(self, x, direction, path, speed, y):
            self.x = x
            self.y = y
            self.direction = direction
            self.drive_path = path
            self.max_speed = speed
    
    class MockParkedVehicle:
        def __init__(self):
            self.x = 250
            self.y = 430
    
    # Configuration
    config = {
        'stop_line_y': 400,
        'creep_limit_y': 380,
        'conflict_zone_start_y': 370,
        'conflict_zone_end_y': 320,
        'intersection_exit_y': 300,
        'intersection_center_x': 400,
        'go_accel': 0.05,
        'stop_accel': -0.1,
        'creep_speed': -0.5,
        'creep_accel': 0.05,
        'obs_noise': 0.0,
        'collision_penalty': -1000,
        'goal_reward': 100,
        'time_penalty': -1,
        'safe_time_gap': 2.5
    }
    
    # Create planner with faster settings
    # Adjust these parameters to balance speed vs quality:
    # - num_sims: 50-200 for fast, 500+ for high quality
    # - max_depth: 5-10 for fast, 15+ for high quality
    # - num_particles: 50-100 for fast, 500+ for high quality
    planner = AVIntersectionPlanner(config, num_sims=100, max_depth=10, num_particles=100)
    
    print("Demonstration: Getting actions for each time step")
    print("=" * 70)
    
    # Simulate several time steps
    av = MockAV()
    
    for step in range(10):
        # Get current traffic situation (in your sim, this would be your actual objects)
        cross_traffic = [
            MockCrossTraffic(200 + step*3, 'right', 'straight', 3.0, 200),
            MockCrossTraffic(280 + step*3, 'right', 'left', 3.0, 200),
            MockCrossTraffic(600 - step*3, 'left', 'straight', 3.0, 200)
        ]
        blocking = [MockParkedVehicle()]
        
        # Get action from POMDP planner
        action = planner.get_action(av, cross_traffic, blocking)
        
        print(f"Step {step}: Action = {action}")
        
        # In your simulation, you would apply this action to your AV
        # For example:
        # if action == 'go':
        #     av.moving = True
        #     av.inching = False
        # elif action == 'creep':
        #     av.inching = True
        #     av.moving = False
        # elif action == 'stop':
        #     av.moving = False
        #     av.inching = False
        
        # Update AV position for next iteration (mock update)
        if action == 'go':
            av.vy = -2.0
        elif action == 'creep':
            av.vy = -0.5
        else:
            av.vy = 0.0
        av.y += av.vy
    
    print("=" * 70)
    print("\nIntegration into your simulation:")
    print("1. Create planner once: planner = AVIntersectionPlanner(config)")
    print("2. Each time step: action = planner.get_action(av, cross_traffic, blocking)")
    print("3. Apply action to your AV based on action string ('go', 'stop', 'creep')")

# import pomdp_py
# import random
# import math

# # ============================================================================
# # State Definition
# # ============================================================================
# class AVState(pomdp_py.State):
#     """State of the autonomous vehicle and environment."""
#     def __init__(self, av, cross_traffic_vehicles, blocking_objects):
#         """
#         Args:
#             av: AutonomousVehicle object from simulation
#             cross_traffic_vehicles: list of CrossTrafficCar objects (visible vehicles only)
#             blocking_objects: list of blocking objects (ParkedVehicle, IntersectionObstruction, StationaryVehicle)
#         """
#         # Extract AV state
#         self.av_pos = av.y  # Y position (lower = closer to stop line)
#         self.av_vel = av.vy  # Y velocity (negative = moving forward/up)
        
#         # Extract cross traffic state - sort by x position and direction
#         self.cross_traffic = tuple(sorted(
#             [(car.x, car.direction, car.drive_path, car.max_speed) 
#              for car in cross_traffic_vehicles],
#             key=lambda v: (v[1], v[0])  # Sort by direction then position
#         ))
        
#         # Extract blocking objects state
#         self.blocking_objects = tuple(sorted(
#             [(obj.x, obj.y, type(obj).__name__) for obj in blocking_objects],
#             key=lambda b: (b[0], b[1])
#         ))
    
#     def __str__(self):
#         traffic_str = [(f'x={x:.0f}', dir, path, f'v={v:.1f}') for x, dir, path, v in self.cross_traffic]
#         block_str = [(f'x={x:.0f}', f'y={y:.0f}', name) for x, y, name in self.blocking_objects]
#         return f"AVState(av_y={self.av_pos:.0f}, av_vy={self.av_vel:.1f}, " \
#                f"traffic={traffic_str}, blocks={block_str})"
    
#     def __repr__(self):
#         return str(self)
    
#     def __hash__(self):
#         traffic_hash = tuple((round(x, 0), d, p, round(v, 1)) for x, d, p, v in self.cross_traffic)
#         block_hash = tuple((round(x, 0), round(y, 0), n) for x, y, n in self.blocking_objects)
#         return hash((round(self.av_pos, 0), round(self.av_vel, 1), traffic_hash, block_hash))
    
#     def __eq__(self, other):
#         if not isinstance(other, AVState):
#             return False
#         return (round(self.av_pos, 0) == round(other.av_pos, 0) and
#                 round(self.av_vel, 1) == round(other.av_vel, 1) and
#                 len(self.cross_traffic) == len(other.cross_traffic) and
#                 all(round(x1, 0) == round(x2, 0) and d1 == d2 and p1 == p2 and round(v1, 1) == round(v2, 1)
#                     for (x1, d1, p1, v1), (x2, d2, p2, v2) in zip(self.cross_traffic, other.cross_traffic)) and
#                 len(self.blocking_objects) == len(other.blocking_objects) and
#                 all(round(x1, 0) == round(x2, 0) and round(y1, 0) == round(y2, 0) and n1 == n2
#                     for (x1, y1, n1), (x2, y2, n2) in zip(self.blocking_objects, other.blocking_objects)))

# # ============================================================================
# # Action Definition
# # ============================================================================
# class AVAction(pomdp_py.Action):
#     """Actions available to the AV."""
#     def __init__(self, name):
#         """
#         Args:
#             name: str, one of ['go', 'stop', 'creep']
#         """
#         self.name = name
    
#     def __str__(self):
#         return self.name
    
#     def __repr__(self):
#         return self.name
    
#     def __hash__(self):
#         return hash(self.name)
    
#     def __eq__(self, other):
#         if not isinstance(other, AVAction):
#             return False
#         return self.name == other.name

# # ============================================================================
# # Observation Definition
# # ============================================================================
# class AVObservation(pomdp_py.Observation):
#     """Observation of the environment."""
#     def __init__(self, cross_traffic_obs, blocking_obs):
#         """
#         Args:
#             cross_traffic_obs: list of (x, direction, path, speed) tuples or None
#             blocking_obs: list of (x, y, type_name) tuples or None
#         """
#         self.cross_traffic_obs = tuple(sorted(cross_traffic_obs, key=lambda v: (v[1], v[0]))) if cross_traffic_obs else None
#         self.blocking_obs = tuple(sorted(blocking_obs, key=lambda b: (b[0], b[1]))) if blocking_obs else None
    
#     def __str__(self):
#         traffic_str = [(f'x={x:.0f}', d, p) for x, d, p, _ in self.cross_traffic_obs] if self.cross_traffic_obs else None
#         block_str = [(f'x={x:.0f}', f'y={y:.0f}', n) for x, y, n in self.blocking_obs] if self.blocking_obs else None
#         return f"Obs(traffic={traffic_str}, blocks={block_str})"
    
#     def __repr__(self):
#         return str(self)
    
#     def __hash__(self):
#         traffic_hash = tuple((round(x, 0), d, p, round(v, 1)) for x, d, p, v in self.cross_traffic_obs) if self.cross_traffic_obs else None
#         block_hash = tuple((round(x, 0), round(y, 0), n) for x, y, n in self.blocking_obs) if self.blocking_obs else None
#         return hash((traffic_hash, block_hash))
    
#     def __eq__(self, other):
#         if not isinstance(other, AVObservation):
#             return False
        
#         # Check cross traffic
#         if self.cross_traffic_obs is None or other.cross_traffic_obs is None:
#             if not (self.cross_traffic_obs is None and other.cross_traffic_obs is None):
#                 return False
#         elif (len(self.cross_traffic_obs) != len(other.cross_traffic_obs) or
#               not all(round(x1, 0) == round(x2, 0) and d1 == d2 and p1 == p2 and round(v1, 1) == round(v2, 1)
#                      for (x1, d1, p1, v1), (x2, d2, p2, v2) in zip(self.cross_traffic_obs, other.cross_traffic_obs))):
#             return False
        
#         # Check blocking objects
#         if self.blocking_obs is None or other.blocking_obs is None:
#             if not (self.blocking_obs is None and other.blocking_obs is None):
#                 return False
#         elif (len(self.blocking_obs) != len(other.blocking_obs) or
#               not all(round(x1, 0) == round(x2, 0) and round(y1, 0) == round(y2, 0) and n1 == n2
#                      for (x1, y1, n1), (x2, y2, n2) in zip(self.blocking_obs, other.blocking_obs))):
#             return False
        
#         return True

# # ============================================================================
# # Transition Model
# # ============================================================================
# class AVTransitionModel(pomdp_py.TransitionModel):
#     """Defines state transitions based on actions."""
    
#     DT = 1.0  # time step (seconds)
    
#     def __init__(self, av_config):
#         """
#         Args:
#             av_config: dict with keys like 'stop_line_y', 'creep_limit_y', 'intersection_exit_y', 
#                       'go_accel', 'stop_accel', 'creep_speed', 'creep_accel'
#         """
#         self.stop_line_y = av_config.get('stop_line_y', 400)  # Y position of stop line
#         self.creep_limit_y = av_config.get('creep_limit_y', 380)  # Y position of creep limit
#         self.intersection_exit_y = av_config.get('intersection_exit_y', 300)  # Y position of exit
        
#         self.go_accel = av_config.get('go_accel', 0.08)  # acceleration for 'go'
#         self.stop_accel = av_config.get('stop_accel', -0.1)  # deceleration for 'stop'
#         self.creep_speed = av_config.get('creep_speed', -0.5)  # target speed for 'creep'
#         self.creep_accel = av_config.get('creep_accel', 0.05)  # acceleration for 'creep'
    
#     def probability(self, next_state, state, action):
#         """Returns probability of transitioning to next_state given state and action."""
#         # Deterministic transitions
#         return 1.0 if self._would_transition_to(state, action, next_state) else 0.0
    
#     def _would_transition_to(self, state, action, next_state):
#         """Check if state would transition to next_state under action."""
#         expected = self.sample(state, action)
#         return next_state == expected
    
#     def sample(self, state, action):
#         """Sample next state given current state and action."""
#         new_vel = state.av_vel
        
#         if action.name == 'go':
#             # Accelerate upward (more negative)
#             new_vel = state.av_vel - self.go_accel
#             new_vel = max(-5.0, new_vel)  # Cap at max speed
#         elif action.name == 'stop':
#             # Decelerate toward zero
#             if state.av_vel < 0:
#                 new_vel = min(0.0, state.av_vel - self.stop_accel)
#             else:
#                 new_vel = 0.0
#         elif action.name == 'creep':
#             # Accelerate slowly to creep speed (negative)
#             if state.av_vel > self.creep_speed:
#                 new_vel = max(self.creep_speed, state.av_vel - self.creep_accel)
#             else:
#                 new_vel = self.creep_speed
        
#         # Update AV position (negative velocity decreases Y)
#         new_pos = state.av_pos + new_vel
        
#         # Creep action stops at creep limit
#         if action.name == 'creep' and new_pos < self.creep_limit_y:
#             new_pos = self.creep_limit_y
#             new_vel = 0.0
        
#         # Cap position at intersection exit
#         new_pos = max(new_pos, self.intersection_exit_y)
        
#         # Update cross traffic positions
#         new_cross_traffic = []
#         for x, direction, path, speed in state.cross_traffic:
#             if direction == 'right':
#                 new_x = x + speed  # Moving right (positive)
#             else:  # 'left'
#                 new_x = x - speed  # Moving left (negative)
#             new_cross_traffic.append((new_x, direction, path, speed))
        
#         # Blocking objects remain constant
#         new_blocking = list(state.blocking_objects)
        
#         # Create mock objects for the new state (simplified - just tuples)
#         # In practice, you'd reconstruct proper objects or pass references
#         return AVState(
#             type('AV', (), {'y': new_pos, 'vy': new_vel})(),
#             [type('Car', (), {'x': x, 'direction': d, 'drive_path': p, 'max_speed': v})() 
#              for x, d, p, v in new_cross_traffic],
#             [type(n, (), {'x': x, 'y': y})() for x, y, n in new_blocking]
#         )
    
#     def get_all_states(self):
#         """Get discretized state space (simplified)."""
#         # This would need to be customized based on your specific discretization needs
#         return []

# # ============================================================================
# # Observation Model
# # ============================================================================
# class AVObservationModel(pomdp_py.ObservationModel):
#     """Defines observation probabilities."""
    
#     def __init__(self, obs_noise=5.0):
#         """
#         Args:
#             obs_noise: standard deviation of observation noise (pixels)
#         """
#         self.obs_noise = obs_noise
    
#     def probability(self, observation, next_state, action):
#         """Returns probability of observation given next_state."""
#         # Blocking objects observed perfectly
#         if observation.blocking_obs != next_state.blocking_objects:
#             return 0.0
        
#         if observation.cross_traffic_obs is None:
#             return 0.01
        
#         # Check we observe the right number of vehicles
#         if len(observation.cross_traffic_obs) != len(next_state.cross_traffic):
#             return 0.0
        
#         # Gaussian noise on each vehicle's observation
#         prob = 1.0
#         for (obs_x, obs_d, obs_p, obs_v), (true_x, true_d, true_p, true_v) in \
#                 zip(observation.cross_traffic_obs, next_state.cross_traffic):
#             if obs_d != true_d or obs_p != true_p:
#                 return 0.0
#             diff = abs(obs_x - true_x)
#             prob *= math.exp(-0.5 * (diff / self.obs_noise) ** 2)
            
#             speed_diff = abs(obs_v - true_v)
#             prob *= math.exp(-0.5 * (speed_diff / 0.5) ** 2)  # Speed noise
        
#         return max(0.001, prob)
    
#     def sample(self, next_state, action):
#         """Sample observation given next_state."""
#         # Blocking objects observed perfectly
#         blocking_obs = list(next_state.blocking_objects)
        
#         # Cross traffic observed with Gaussian noise on position
#         cross_obs = [(x + random.gauss(0, self.obs_noise), direction, path, 
#                      speed + random.gauss(0, 0.3))
#                     for x, direction, path, speed in next_state.cross_traffic]
        
#         return AVObservation(cross_obs, blocking_obs)

# # ============================================================================
# # Reward Model
# # ============================================================================
# class AVRewardModel(pomdp_py.RewardModel):
#     """Defines rewards for state-action pairs."""
    
#     def __init__(self, config):
#         """
#         Args:
#             config: dict with intersection geometry and reward parameters
#         """
#         self.stop_line_y = config.get('stop_line_y', 400)
#         self.creep_limit_y = config.get('creep_limit_y', 380)
#         self.conflict_zone_start_y = config.get('conflict_zone_start_y', 370)
#         self.conflict_zone_end_y = config.get('conflict_zone_end_y', 320)
#         self.intersection_exit_y = config.get('intersection_exit_y', 300)
#         self.intersection_center_x = config.get('intersection_center_x', 400)
        
#         self.collision_penalty = config.get('collision_penalty', -1000)
#         self.goal_reward = config.get('goal_reward', 100)
#         self.time_penalty = config.get('time_penalty', -1)
#         self.safe_time_gap = config.get('safe_time_gap', 2.5)
    
#     def _check_collision(self, state):
#         """Check if AV collides with any cross traffic."""
#         # AV is in conflict zone
#         in_conflict = self.conflict_zone_end_y <= state.av_pos <= self.conflict_zone_start_y
        
#         if not in_conflict:
#             return False
        
#         # Define collision zone around intersection center (±50 pixels)
#         collision_x_min = self.intersection_center_x - 50
#         collision_x_max = self.intersection_center_x + 50
        
#         # Check collision with any vehicle in the conflict zone
#         for x, direction, path, speed in state.cross_traffic:
#             if collision_x_min <= x <= collision_x_max:
#                 return True
        
#         return False
    
#     def _time_to_conflict(self, vehicle_x, direction, speed):
#         """Calculate time until vehicle reaches conflict zone."""
#         # Handle zero speed case
#         if speed <= 0:
#             return float('inf')
        
#         conflict_min = self.intersection_center_x - 25
#         conflict_max = self.intersection_center_x + 25
        
#         if direction == 'right':
#             # Moving right (positive direction)
#             if vehicle_x < conflict_min:
#                 return (conflict_min - vehicle_x) / speed
#             elif vehicle_x <= conflict_max:
#                 return 0.0  # Already in conflict
#             else:
#                 return float('inf')  # Past conflict zone
#         else:  # 'left'
#             # Moving left (negative direction)
#             if vehicle_x > conflict_max:
#                 return (vehicle_x - conflict_max) / speed
#             elif vehicle_x >= conflict_min:
#                 return 0.0  # Already in conflict
#             else:
#                 return float('inf')  # Past conflict zone
    
#     def _is_safe_to_go(self, state):
#         """Check if there's sufficient gap in cross traffic."""


#         for x, direction, path, speed in state.cross_traffic:
#             ttc = self._time_to_conflict(x, direction, speed)
#             if ttc < self.safe_time_gap:
#                 return False
#         return True
    
#     def sample(self, state, action, next_state):
#         """Sample reward for transition."""
#         # Check for collision
#         if self._check_collision(next_state):
#             return self.collision_penalty
        
#         # Reached goal
#         if next_state.av_pos <= self.intersection_exit_y:
#             return self.goal_reward
        
#         # Base time penalty
#         reward = self.time_penalty
        
#         # Large penalty for creeping when already at intersection edge
#         if action.name == 'creep' and state.av_pos <= self.creep_limit_y:
#             reward -= 20  # Large penalty to discourage creeping past the edge
        
#         # Reward strategic creeping with blocking objects (only if not at edge yet)
#         if action.name == 'creep' and len(next_state.blocking_objects) > 0 and state.av_pos > self.creep_limit_y:
#             reward += 5
#             # Extra reward for reaching good observation position
#             if next_state.av_pos <= self.creep_limit_y + 5:
#                 reward += 3
        
#         # Reward safe execution
#         if action.name == 'go' and self._is_safe_to_go(next_state):
#             reward += 4
        
#         # Reward waiting when unsafe
#         if action.name == 'stop' and not self._is_safe_to_go(next_state):
#             reward += 2
        
#         # Penalize unnecessary waiting when safe
#         if (action.name == 'stop' and self._is_safe_to_go(next_state) and 
#             next_state.av_pos > self.creep_limit_y):
#             reward -= 2
        
#         return reward

# # ============================================================================
# # Policy Model
# # ============================================================================
# class AVPolicyModel(pomdp_py.RolloutPolicy):
#     """Simple random policy for rollouts."""
    
#     def __init__(self):
#         self.actions = [AVAction('go'), AVAction('stop'), AVAction('creep')]
    
#     def sample(self, state):
#         return random.choice(self.actions)
    
#     def rollout(self, state, history=None):
#         return random.choice(self.actions)
    
#     def get_all_actions(self, state=None, history=None):
#         return self.actions

# # ============================================================================
# # POMDP Problem
# # ============================================================================
# class AVIntersectionProblem(pomdp_py.POMDP):
#     """Complete POMDP problem definition."""
    
#     def __init__(self, av, cross_traffic_vehicles, blocking_objects, config):
#         """
#         Args:
#             av: AutonomousVehicle object
#             cross_traffic_vehicles: list of CrossTrafficCar objects
#             blocking_objects: list of blocking objects
#             config: dict with configuration parameters
#         """
#         init_state = AVState(av, cross_traffic_vehicles, blocking_objects)
        
#         agent = pomdp_py.Agent(
#             pomdp_py.Histogram({init_state: 1.0}),
#             AVPolicyModel(),
#             AVTransitionModel(config),
#             AVObservationModel(config.get('obs_noise', 5.0)),
#             AVRewardModel(config)
#         )
#         env = pomdp_py.Environment(
#             init_state,
#             AVTransitionModel(config),
#             AVRewardModel(config)
#         )
#         super().__init__(agent, env, name="AVIntersection")

# # ============================================================================
# # Helper Class for Easy Integration
# # ============================================================================
# class AVIntersectionPlanner:
#     """Wrapper class for easy integration with simulation."""
    
#     def __init__(self, config, num_sims=100, max_depth=10, exploration_const=50, num_particles=100):
#         """
#         Args:
#             config: configuration dictionary
#             num_sims: number of POMCP simulations per planning step (lower = faster)
#             max_depth: maximum planning depth (lower = faster)
#             exploration_const: exploration constant for POMCP
#             num_particles: number of particles for belief representation (lower = faster)
#         """
#         self.config = config
#         self.num_particles = num_particles
#         # Create POMCP with custom rollout policy
#         self.pomcp = pomdp_py.POMCP(
#             max_depth=max_depth,
#             discount_factor=0.95,
#             num_sims=num_sims,
#             exploration_const=exploration_const,
#             rollout_policy=AVPolicyModel()
#         )
#         self.problem = None
    
#     def get_action(self, av, cross_traffic_vehicles, blocking_objects):
#         """
#         Get action for current time step.
        
#         Args:
#             av: AutonomousVehicle object
#             cross_traffic_vehicles: list of visible CrossTrafficCar objects
#             blocking_objects: list of blocking objects
            
#         Returns:
#             str: action name ('go', 'stop', or 'creep')
#         """
#         # Create new state
#         new_state = AVState(av, cross_traffic_vehicles, blocking_objects)
        
#         # Create or update problem
#         if self.problem is None:
#             self.problem = AVIntersectionProblem(av, cross_traffic_vehicles, 
#                                                 blocking_objects, self.config)
#             # Convert initial belief to particles
#             particles = [new_state for _ in range(self.num_particles)]
#             self.problem.agent.set_belief(pomdp_py.Particles(particles))
#         else:
#             # Recreate environment with new state
#             self.problem.env = pomdp_py.Environment(
#                 new_state,
#                 self.problem.env.transition_model,
#                 self.problem.env.reward_model
#             )
#             # Create particle belief centered on current state
#             particles = [new_state for _ in range(self.num_particles)]
#             self.problem.agent.set_belief(pomdp_py.Particles(particles))
        
#         # Plan and return action
#         action = self.pomcp.plan(self.problem.agent)
#         return action.name

# # ============================================================================
# # Example Usage
# # ============================================================================
# if __name__ == "__main__":
#     # Mock objects for demonstration
#     class MockAV:
#         def __init__(self):
#             self.x = 350
#             self.y = 400
#             self.vx = 0
#             self.vy = 0
    
#     class MockCrossTraffic:
#         def __init__(self, x, direction, path, speed):
#             self.x = x
#             self.direction = direction
#             self.drive_path = path
#             self.max_speed = speed
    
#     class MockParkedVehicle:
#         def __init__(self):
#             self.x = 250
#             self.y = 430
    
#     # Configuration
#     config = {
#         'stop_line_y': 400,
#         'creep_limit_y': 380,
#         'conflict_zone_start_y': 370,
#         'conflict_zone_end_y': 320,
#         'intersection_exit_y': 300,
#         'intersection_center_x': 400,
#         'go_accel': 0.05,
#         'stop_accel': -0.1,
#         'creep_speed': -0.5,
#         'creep_accel': 0.05,
#         'obs_noise': 0.0,
#         'collision_penalty': -1000,
#         'goal_reward': 100,
#         'time_penalty': -1,
#         'safe_time_gap': 2.5
#     }
    
#     # Create planner with faster settings
#     # Adjust these parameters to balance speed vs quality:
#     # - num_sims: 50-200 for fast, 500+ for high quality
#     # - max_depth: 5-10 for fast, 15+ for high quality
#     # - num_particles: 50-100 for fast, 500+ for high quality
#     planner = AVIntersectionPlanner(config, num_sims=100, max_depth=10, num_particles=100)
    
#     print("Demonstration: Getting actions for each time step")
#     print("=" * 70)
    
#     # Simulate several time steps
#     av = MockAV()
    
#     for step in range(10):
#         # Get current traffic situation (in your sim, this would be your actual objects)
#         cross_traffic = [
#             MockCrossTraffic(200 + step*3, 'right', 'straight', 3.0),
#             MockCrossTraffic(280 + step*3, 'right', 'left', 3.0),
#             MockCrossTraffic(600 - step*3, 'left', 'straight', 3.0)
#         ]
#         blocking = [MockParkedVehicle()]
        
#         # Get action from POMDP planner
#         action = planner.get_action(av, cross_traffic, blocking)
        
#         print(f"Step {step}: Action = {action}")
        
#         # In your simulation, you would apply this action to your AV
#         # For example:
#         # if action == 'go':
#         #     av.moving = True
#         #     av.inching = False
#         # elif action == 'creep':
#         #     av.inching = True
#         #     av.moving = False
#         # elif action == 'stop':
#         #     av.moving = False
#         #     av.inching = False
        
#         # Update AV position for next iteration (mock update)
#         if action == 'go':
#             av.vy = -2.0
#         elif action == 'creep':
#             av.vy = -0.5
#         else:
#             av.vy = 0.0
#         av.y += av.vy
    
#     print("=" * 70)
#     print("\nIntegration into your simulation:")
#     print("1. Create planner once: planner = AVIntersectionPlanner(config)")
#     print("2. Each time step: action = planner.get_action(av, cross_traffic, blocking)")
#     print("3. Apply action to your AV based on action string ('go', 'stop', 'creep')")