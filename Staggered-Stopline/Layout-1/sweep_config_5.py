"""
Parameter Sweep -
"""

import pygame
import random
import sys
import json
from typing import Dict, List
from dataclasses import dataclass, asdict
import itertools

# Import your simulation components
import config
import av_class
import blocker_vehicle_class
import cross_traffic_class
import utils
from pomdp_unseen_cars_blocked_area_5 import UnseenCarPOMDPAgent, UnseenCarPOMDPConfig, should_av_go_pomdp


# ============================================================================
# Configuration - ONLY parameters from UnseenCarPOMDPConfig
# ============================================================================

@dataclass
class SweepConfig:
    """ONLY parameters that exist in UnseenCarPOMDPConfig"""
    
    # Planning (usually not swept)
    planning_horizon: int = 10
    discount_factor: float = 0.95
    
    # Reward/cost structure
    cost_stop: float = -5.0
    cost_creep: float = -2.5
    reward_go_low: float = 200.0
    reward_go_medium: float = -40.0
    reward_go_high: float = -300.0
    
    # Sensor accuracy (stored as separate values, converted to dict)
    sensor_accuracy_low: float = 0.92
    sensor_accuracy_medium: float = 0.75
    sensor_accuracy_high: float = 0.5
    
    # Creep behavior
    enable_creep_improvement: bool = True
    creep_improvement_rate: float = 0.4
    natural_change_rate: float = 0.08
    
    # Decision thresholds
    min_steps_before_go: int = 4
    max_info_gathering_steps: int = 30
    
    # FOV thresholds
    fov_blocked_threshold: float = 0.9
    
    # Feature flags
    enable_unseen_car_model: bool = True
    enable_visibility_check: bool = True
    
    # Prior probability that an unseen car exists at FOV edge
    p_exist: float = 0.2
    
    # Danger weight per expected unseen car
    unseen_car_danger_weight: float = 35.0
    
    # Spatial parameters
    edge_detection_threshold: float = 50.0
    min_hidden_width_for_concern: float = 80.0
    
    # Distance-based probability adjustment
    use_distance_weighting: bool = True
    distance_weight_factor: float = 0.5
    
    # Multiple edge modeling
    model_multiple_edges: bool = True
    
    # Collision time consideration
    consider_collision_timing: bool = True
    critical_collision_time: float = 3.0


def config_to_pomdp_config(sweep_config: SweepConfig) -> UnseenCarPOMDPConfig:
    """Convert SweepConfig to UnseenCarPOMDPConfig - 1-to-1 mapping"""
    return UnseenCarPOMDPConfig(
        planning_horizon=sweep_config.planning_horizon,
        discount_factor=sweep_config.discount_factor,
        cost_stop=sweep_config.cost_stop,
        cost_creep=sweep_config.cost_creep,
        reward_go_low=sweep_config.reward_go_low,
        reward_go_medium=sweep_config.reward_go_medium,
        reward_go_high=sweep_config.reward_go_high,
        sensor_accuracy={
            "low": sweep_config.sensor_accuracy_low,
            "medium": sweep_config.sensor_accuracy_medium,
            "high": sweep_config.sensor_accuracy_high
        },
        enable_creep_improvement=sweep_config.enable_creep_improvement,
        creep_improvement_rate=sweep_config.creep_improvement_rate,
        natural_change_rate=sweep_config.natural_change_rate,
        min_steps_before_go=sweep_config.min_steps_before_go,
        max_info_gathering_steps=sweep_config.max_info_gathering_steps,
        fov_blocked_threshold=sweep_config.fov_blocked_threshold,
        enable_unseen_car_model=sweep_config.enable_unseen_car_model,
        enable_visibility_check=sweep_config.enable_visibility_check,
        p_exist=sweep_config.p_exist,
        unseen_car_danger_weight=sweep_config.unseen_car_danger_weight,
        edge_detection_threshold=sweep_config.edge_detection_threshold,
        min_hidden_width_for_concern=sweep_config.min_hidden_width_for_concern,
        use_distance_weighting=sweep_config.use_distance_weighting,
        distance_weight_factor=sweep_config.distance_weight_factor,
        model_multiple_edges=sweep_config.model_multiple_edges,
        consider_collision_timing=sweep_config.consider_collision_timing,
        critical_collision_time=sweep_config.critical_collision_time
    )


# ============================================================================
# Headless Simulator
# ============================================================================

class HeadlessSimulator:
    """Runs simulation without GUI"""
    
    def __init__(self):
        if not pygame.get_init():
            pygame.init()
        self.screen = pygame.Surface((config.WIDTH, config.HEIGHT))
        
    def run_episode(self, pomdp_config: UnseenCarPOMDPConfig, 
                   max_steps: int = 3000,
                   include_blockers: bool = True,
                   verbose: bool = False) -> Dict[str, float]:
        """Run one episode"""
        pomdp_agent = UnseenCarPOMDPAgent(config=pomdp_config)
        
        av = av_class.AutonomousVehicle(self.screen)
        av.manual_trigger = True
        av.inch_behave = True
        
        blockers = []
        if include_blockers:
            blockers.append(blocker_vehicle_class.StationaryVehicle(self.screen))
        
        cross_traffic = []
        collision = False
        success = False
        creep_count = 0
        stop_count = 0
        go_count = 0
        
        for steps in range(1, max_steps + 1):
            # Spawn cross traffic
            if random.random() < config.TRAFFIC_FLOW / (3600 * config.FPS):
                direction = random.choice(['left', 'right'])
                new_car = cross_traffic_class.CrossTrafficCar(direction, self.screen)
                cross_traffic.append(new_car)
            
            # Update cross traffic
            for car in cross_traffic:
                car.update(cross_traffic)
            cross_traffic = [c for c in cross_traffic 
                           if (-config.CAR_WIDTH < c.x < config.WIDTH + config.CAR_WIDTH)]
            
            # POMDP decision
            if av.manual_trigger and not av.moving and not av.collided:
                should_go = should_av_go_pomdp(cross_traffic, av, blockers, pomdp_agent)
                
                if av.inching:
                    creep_count += 1
                elif av.moving:
                    go_count += 1
                else:
                    stop_count += 1
            
            av.update()
            
            # Check collision
            collision, collision_car = av.check_collision(cross_traffic)
            if collision:
                if verbose:
                    print(f"  [COLLISION at step {steps}]")
                break
            
            # Check success
            if av.y + config.AV_HEIGHT < 250 or av.x < 0 or av.x > config.WIDTH:
                success = True
                if verbose:
                    print(f"  [SUCCESS at step {steps}]")
                break
        
        time_seconds = steps / config.FPS
        
        return {
            'success': float(success),
            'collision': float(collision),
            'timeout': float(not success and not collision),
            'time': time_seconds,
            'steps': steps,
            'creep_actions': creep_count,
            'stop_actions': stop_count,
            'go_actions': go_count,
        }


def simulator_func(sweep_config: SweepConfig, verbose: bool = False) -> Dict[str, float]:
    """Wrapper for parameter sweep"""
    pomdp_config = config_to_pomdp_config(sweep_config)
    sim = HeadlessSimulator()
    result = sim.run_episode(pomdp_config, verbose=verbose)
    
    return {
        'success_rate': result['success'],
        'collision_rate': result['collision'],
        'timeout_rate': result['timeout'],
        'avg_time': result['time'],
        'avg_steps': result['steps'],
        'creep_actions': result['creep_actions'],
        'stop_actions': result['stop_actions'],
        'go_actions': result['go_actions'],
    }


# ============================================================================
# Parameter Sweep Functions
# ============================================================================

def grid_sweep(param_ranges: Dict[str, List]) -> List[SweepConfig]:
    """Create all combinations (grid search)"""
    param_names = list(param_ranges.keys())
    param_values = [param_ranges[name] for name in param_names]
    
    configs = []
    for values in itertools.product(*param_values):
        config_dict = {name: value for name, value in zip(param_names, values)}
        configs.append(SweepConfig(**config_dict))
    
    return configs


class SweepRunner:
    """Runs the parameter sweep"""
    
    def __init__(self):
        self.results = []
    
    def run_sweep(self, configs: List[SweepConfig], n_episodes: int = 10):
        """Run sweep over all configs"""
        total = len(configs)
        
        print(f"\n{'='*70}")
        print(f"Starting Parameter Sweep")
        print(f"{'='*70}")
        print(f"Total configs to test: {total}")
        print(f"Episodes per config: {n_episodes}")
        print(f"Total episodes: {total * n_episodes}\n")
        
        for i, config in enumerate(configs, 1):
            # print(f"[{i}/{total}] Testing config...")
            
            episode_metrics = []
            for ep in range(n_episodes):
                metrics = simulator_func(config, verbose=False)
                episode_metrics.append(metrics)
                
                if (ep + 1) % 5 == 0:
                    print(f"  ... episode {ep + 1}/{n_episodes}")
            
            avg_metrics = self._average_metrics(episode_metrics)
            
            self.results.append({
                'config': asdict(config),
                'metrics': avg_metrics
            })
            
            print(f"  → success={avg_metrics['success_rate']:.2%}, "
                  f"collision={avg_metrics['collision_rate']:.2%}, "
                  f"time={avg_metrics['avg_time']:.1f}s\n")
    
    def _average_metrics(self, episode_metrics: List[Dict]) -> Dict:
        """Average metrics across episodes"""
        avg = {}
        for key in episode_metrics[0].keys():
            avg[key] = sum(m[key] for m in episode_metrics) / len(episode_metrics)
        return avg
    
    def save(self, filename: str = "sweep_results_one_config_per_time.json"):
        """Save results"""
        with open(filename, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"\nResults saved to {filename}")
    
    def print_summary(self):
        """Print summary"""
        print("\n" + "="*70)
        print("SWEEP SUMMARY")
        print("="*70)
        print(f"Tested {len(self.results)} configurations\n")
        
        best = max(self.results, key=lambda r: r['metrics']['success_rate'])
        print(f"Best success rate: {best['metrics']['success_rate']:.1%}")
        print(f"  p_exist: {best['config']['p_exist']}")
        print(f"  unseen_car_danger_weight: {best['config']['unseen_car_danger_weight']}")


# ============================================================================
# SWEEP DEFINITIONS (ONLY VALID PARAMETERS!)
# ============================================================================
def run_sweep_configs():

    configs_values = {
        'discount_factor':[0.90, 0.95, 1],

        'cost_stop': [-1, -5, -10],
        'cost_creep': [-2, -2.5, -3],

        'reward_go_low': [150, 200, 250],
        'reward_go_medium': [-50, -40, -30],
        'reward_go_high': [-250, -300, -350],

        'sensor_accuracy_low': [0.88, 0.92, 0.94],
        'sensor_accuracy_medium': [0.71, 0.75, 0.79],
        'sensor_accuracy_high': [0.45, 0.5, 0.55],

        'creep_improvement_rate': [0.30, 0.4, 0.50],
        'natural_change_rate': [0.04, 0.08, 0.12],

        'min_steps_before_go': [1, 4, 8],
        'max_info_gathering_steps': [20, 30, 40],

        'fov_blocked_threshold': [0.7, 0.8, 0.9],

        'p_exist': [0.1, 0.2, 0.3],  

        'unseen_car_danger_weight': [30, 35, 40], 

        'edge_detection_threshold': [40, 50, 60],
        'min_hidden_width_for_concern': [70, 80, 90], 

        'distance_weight_factor': [0.4, 0.5, 0.6], 

        'critical_collision_time': [2.5, 3.0, 3.5], 
    }

    # Loop through each parameter one at a time
    for param_name, param_values in configs_values.items():
        print(f"\n{'='*70}")
        print(f"Sweeping: {param_name}")
        print(f"{'='*70}")
        
        # Create grid_sweep with just this one parameter
        configs = grid_sweep({
            param_name: param_values
        })
        
        print(f"Testing {len(configs)} configurations for {param_name}")
 
        runner = SweepRunner()
        runner.run_sweep(configs, n_episodes=10)
        runner.save(f"sweep_{param_name}.json")
        runner.print_summary()


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":

    run_sweep_configs()
    