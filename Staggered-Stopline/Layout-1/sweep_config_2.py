#

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
# STEP 1: Configuration with all magic numbers
# ============================================================================

@dataclass
class SweepConfig:
    """All tunable parameters from your POMDP"""
    

    p_exist: float = 0.3
    unseen_car_danger_weight: float = 35.0
    edge_detection_threshold: float = 50.0
    min_hidden_width_for_concern: float = 80.0
    

    danger_high: float = 60.0
    danger_medium: float = 30.0
    visibility_high: float = 0.3
    visibility_medium: float = 0.6
    

    fov_penalty_low: float = 30.0
    fov_penalty_medium: float = 15.0
    fov_threshold_low: float = 0.4
    fov_threshold_medium: float = 0.6
    max_possible_fov_ratio: float = 0.7
    

    safety_x_min: float = -50.0
    safety_x_max: float = 200.0
    

    creep_improvement_rate: float = 0.4
    natural_change_rate: float = 0.08
    

    cost_stop: float = -1.0
    cost_creep: float = -2.5
    reward_go_low: float = 100.0
    reward_go_medium: float = -40.0
    reward_go_high: float = -350.0
    
   
    sensor_accuracy_low: float = 0.92
    sensor_accuracy_medium: float = 0.75
    sensor_accuracy_high: float = 0.50
    
    
    min_steps_before_go: int = 4
    max_info_gathering_steps: int = 40


def config_to_pomdp_config(sweep_config: SweepConfig) -> UnseenCarPOMDPConfig:
    """Convert SweepConfig to UnseenCarPOMDPConfig"""
    return UnseenCarPOMDPConfig(
        # Unseen car model
        p_exist=sweep_config.p_exist,
        unseen_car_danger_weight=sweep_config.unseen_car_danger_weight,
        edge_detection_threshold=sweep_config.edge_detection_threshold,
        min_hidden_width_for_concern=sweep_config.min_hidden_width_for_concern,
        
        # Sensor accuracy
        sensor_accuracy={
            "low": sweep_config.sensor_accuracy_low,
            "medium": sweep_config.sensor_accuracy_medium,
            "high": sweep_config.sensor_accuracy_high
        },
        
        # Transition rates
        creep_improvement_rate=sweep_config.creep_improvement_rate,
        natural_change_rate=sweep_config.natural_change_rate,
        
        # Rewards
        cost_stop=sweep_config.cost_stop,
        cost_creep=sweep_config.cost_creep,
        reward_go_low=sweep_config.reward_go_low,
        reward_go_medium=sweep_config.reward_go_medium,
        reward_go_high=sweep_config.reward_go_high,
        
        # Decision thresholds
        min_steps_before_go=sweep_config.min_steps_before_go,
        max_info_gathering_steps=sweep_config.max_info_gathering_steps,
        
        # Feature flags
        enable_unseen_car_model=True,
        enable_creep_improvement=True,
    )


# ============================================================================
# STEP 2: Headless Simulator (no GUI, just metrics)
# ============================================================================

class HeadlessSimulator:
    """Runs simulation without GUI"""
    
    def __init__(self):
        # Initialize pygame in headless mode
        if not pygame.get_init():
            pygame.init()
        self.screen = pygame.Surface((config.WIDTH, config.HEIGHT))
        
    def run_episode(self, pomdp_config: UnseenCarPOMDPConfig, 
                   max_steps: int = 3000,
                   include_blockers: bool = True,
                   verbose: bool = False) -> Dict[str, float]:
        """
        Run one episode of the simulation.
        
        Returns:
            dict with metrics: success, collision, time, steps, etc.
        """
      
        pomdp_agent = UnseenCarPOMDPAgent(config=pomdp_config)
        
 
        av = av_class.AutonomousVehicle(self.screen)
        av.manual_trigger = True  
        av.inch_behave = True  
        
        # Add blockers
        blockers = []
        if include_blockers:
            blockers.append(blocker_vehicle_class.StationaryVehicle(self.screen))
        
        cross_traffic = []
        
 
        collision = False
        success = False
        steps = 0
        creep_count = 0
        stop_count = 0
        go_count = 0
        
       
        for steps in range(1, max_steps + 1):
         
            
           
            if random.random() < config.TRAFFIC_FLOW / (3600 * config.FPS):
                direction = random.choice(['left', 'right'])
                new_car = cross_traffic_class.CrossTrafficCar(direction, 
                    self.screen
                )
                cross_traffic.append(new_car)
            
            
            for car in cross_traffic:
                car.update(cross_traffic)  
            cross_traffic = [c for c in cross_traffic 
                           if (-config.CAR_WIDTH < c.x < config.WIDTH + config.CAR_WIDTH)]
            
            
            if av.manual_trigger and not av.moving and not av.collided:
                
                should_go = should_av_go_pomdp(cross_traffic, av, blockers, pomdp_agent)
                
                
                if av.inching:
                    creep_count += 1
                elif av.moving:
                    go_count += 1
                else:
                    stop_count += 1
            
        
            av.update()
            
           
            collision, collision_car = av.check_collision(cross_traffic)
            
            if collision:
                break
            
           
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


# ============================================================================
# STEP 3: Simulator Function for Parameter Sweep
# ============================================================================

def simulator_func(sweep_config: SweepConfig, verbose: bool = False) -> Dict[str, float]:
    """
    This is the function that the parameter sweep calls.
    It takes a SweepConfig and returns metrics.
    """
   
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
# STEP 4: Parameter Sweep Functions
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
            print(f"[{i}/{total}] Testing: p_exist={config.p_exist:.2f}, "
                  f"danger_weight={config.unseen_car_danger_weight:.0f}, "
                  f"danger_high={config.danger_high:.0f}")
     
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
    
    def save(self, filename: str = "sweep_results.json"):
        """Save results"""
        with open(filename, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"\nResults saved to {filename}")
    
    def get_best(self, metric: str = 'success_rate', minimize: bool = False):
        """Get best config"""
        if minimize:
            best = min(self.results, key=lambda r: r['metrics'][metric])
        else:
            best = max(self.results, key=lambda r: r['metrics'][metric])
        return best
    
    def print_summary(self):
        """Print summary"""
        print("\n" + "="*70)
        print("SWEEP SUMMARY")
        print("="*70)
        print(f"Tested {len(self.results)} configurations\n")
        
        # Best configs
        metrics = ['success_rate', 'collision_rate', 'avg_time']
        for metric in metrics:
            minimize = (metric in ['collision_rate', 'avg_time'])
            best = self.get_best(metric, minimize=minimize)
            value = best['metrics'][metric]
            
            print(f"Best {metric}: {value:.3f}")
            print(f"  p_exist={best['config']['p_exist']:.2f}")
            print(f"  unseen_car_danger_weight={best['config']['unseen_car_danger_weight']:.0f}")
            print(f"  danger_high={best['config']['danger_high']:.0f}\n")


# ============================================================================
# STEP 5: Analysis Functions
# ============================================================================

def analyze_results(json_file: str = "sweep_results.json"):
    """Analyze results"""
    import pandas as pd
    
    with open(json_file, 'r') as f:
        results = json.load(f)
    
    rows = []
    for r in results:
        row = {**r['config'], **r['metrics']}
        rows.append(row)
    
    df = pd.DataFrame(rows)
    
    print("\n" + "="*70)
    print("PARAMETER SENSITIVITY (correlation with success_rate)")
    print("="*70)
    
    params = ['p_exist', 'unseen_car_danger_weight', 'danger_high', 'danger_medium',
              'fov_penalty_low', 'creep_improvement_rate']
    
    for param in params:
        if param in df.columns:
            corr = df[param].corr(df['success_rate'])
            impact = "★★★" if abs(corr) > 0.5 else "★★" if abs(corr) > 0.3 else "★"
            print(f"{param:30s}: {corr:+.3f}  {impact}")
    
    print("\n" + "="*70)
    print("METRIC RANGES")
    print("="*70)
    for metric in ['success_rate', 'collision_rate', 'avg_time']:
        if metric in df.columns:
            print(f"{metric:20s}: [{df[metric].min():.3f}, {df[metric].max():.3f}]  "
                  f"mean={df[metric].mean():.3f}")
    
    return df


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

def run_quick_test():
    """Quick test with a few configs"""
    print("\n" + "="*70)
    print("QUICK TEST - Testing 6 configurations")
    print("="*70)
    
    configs = grid_sweep({
        'p_exist': [0.2, 0.3, 0.4],
        'danger_high': [60, 70]
    })
    
    runner = SweepRunner()
    runner.run_sweep(configs, n_episodes=5)
    runner.save("quick_test_results.json")
    runner.print_summary()
    
    df = analyze_results("quick_test_results.json")
    
    # try:
    #     plot_results(df)
    # except Exception as e:
    #     print(f"Plotting skipped: {e}")


def run_baseline_sweep():
    """Baseline sweep - moderate number of configs"""
    print("\n" + "="*70)
    print("BASELINE SWEEP")
    print("="*70)
    
    configs = grid_sweep({
    # Unseen car model (MOST IMPORTANT - affects danger calculation)
    'p_exist': [0.2, 0.3, 0.4],                          
    'unseen_car_danger_weight': [25, 30, 35, 40],        
    'edge_detection_threshold': [40, 50, 60],            
    
    # Danger thresholds (CRITICAL - determines when it's "safe" to GO)
    'danger_high': [50, 55, 60, 65, 70],                 
    'danger_medium': [25, 30, 35],                       
    
    # Visibility thresholds
    'visibility_high': [0.2, 0.3, 0.4],                  
    'visibility_medium': [0.5, 0.6, 0.7],                
    
    # FOV penalties (affects observation model)
    'fov_penalty_low': [25, 30, 35],                     
    'fov_penalty_medium': [10, 15, 20],                  
    'fov_threshold_low': [0.3, 0.4, 0.5],              
    'fov_threshold_medium': [0.5, 0.6, 0.7],            
    
    # Transition model
    'creep_improvement_rate': [0.3, 0.4, 0.5],          
    'natural_change_rate': [0.05, 0.08, 0.12],          
    
    # Rewards (affects action selection)
    'cost_stop': [-1, -5, -10],                         
    'cost_creep': [-2, -2.5, -3],                        
    'reward_go_low': [100, 150, 200],                    
    'reward_go_medium': [-40, -20, 0],                   
    'reward_go_high': [-350, -250, -150],               
    
    # Decision timing
    'min_steps_before_go': [2, 4, 6],                    
    'max_info_gathering_steps': [30, 40, 50, 60],        
    })
    
    print(f"This will test {len(configs)} configurations")
    proceed = input("Continue? (y/n): ")
    
    if proceed.lower() == 'y':
        runner = SweepRunner()
        runner.run_sweep(configs, n_episodes=10)
        runner.save("baseline_sweep_results.json")
        runner.print_summary()
        
        df = analyze_results("baseline_sweep_results.json")
        # plot_results(df)


def run_sweep_1_unseen_car():
    """SWEEP 1: Unseen Car Model (36 configs, ~30 min)"""
    print("\n" + "="*70)
    print("SWEEP 1: UNSEEN CAR MODEL")
    print("="*70)
    
    configs = grid_sweep({
        'p_exist': [0.2, 0.3, 0.4],
        'unseen_car_danger_weight': [25, 30, 35, 40],
        'edge_detection_threshold': [40, 50, 60]
    })
    
    print(f"Testing {len(configs)} configurations (~30 minutes)")
    proceed = input("Continue? (y/n): ")
    
    if proceed.lower() == 'y':
        runner = SweepRunner()
        runner.run_sweep(configs, n_episodes=10)
        runner.save("sweep1_unseen_car.json")
        runner.print_summary()
        df = analyze_results("sweep1_unseen_car.json")


def run_sweep_2_danger_thresholds():
    """SWEEP 2: Danger Thresholds (45 configs, ~40 min)"""
    print("\n" + "="*70)
    print("SWEEP 2: DANGER THRESHOLDS")
    print("="*70)
    
    configs = grid_sweep({
        'danger_high': [50, 55, 60, 65, 70],
        'danger_medium': [25, 30, 35],
        'visibility_high': [0.2, 0.3, 0.4]
    })
    
    print(f"Testing {len(configs)} configurations (~40 minutes)")
    proceed = input("Continue? (y/n): ")
    
    if proceed.lower() == 'y':
        runner = SweepRunner()
        runner.run_sweep(configs, n_episodes=10)
        runner.save("sweep2_danger_thresholds.json")
        runner.print_summary()
        df = analyze_results("sweep2_danger_thresholds.json")


def run_sweep_3_fov():
    """SWEEP 3: FOV & Perception (81 configs, ~60 min)"""
    print("\n" + "="*70)
    print("SWEEP 3: FOV & PERCEPTION")
    print("="*70)
    
    configs = grid_sweep({
        'fov_penalty_low': [25, 30, 35],
        'fov_penalty_medium': [10, 15, 20],
        'fov_threshold_low': [0.3, 0.4, 0.5],
        'fov_threshold_medium': [0.5, 0.6, 0.7]
    })
    
    print(f"Testing {len(configs)} configurations (~60 minutes)")
    proceed = input("Continue? (y/n): ")
    
    if proceed.lower() == 'y':
        runner = SweepRunner()
        runner.run_sweep(configs, n_episodes=10)
        runner.save("sweep3_fov.json")
        runner.print_summary()
        df = analyze_results("sweep3_fov.json")


def run_sweep_4_rewards():
    """SWEEP 4: Rewards (54 configs, ~45 min)"""
    print("\n" + "="*70)
    print("SWEEP 4: REWARDS")
    print("="*70)
    
    configs = grid_sweep({
        'cost_stop': [-1, -5, -10],
        'cost_creep': [-2, -2.5, -3],
        'reward_go_low': [100, 150, 200],
        'p_exist': [0.3, 0.4]
    })
    
    print(f"Testing {len(configs)} configurations (~45 minutes)")
    proceed = input("Continue? (y/n): ")
    
    if proceed.lower() == 'y':
        runner = SweepRunner()
        runner.run_sweep(configs, n_episodes=10)
        runner.save("sweep4_rewards.json")
        runner.print_summary()
        df = analyze_results("sweep4_rewards.json")


def run_sweep_5_timing():
    """SWEEP 5: Decision Timing (36 configs, ~30 min)"""
    print("\n" + "="*70)
    print("SWEEP 5: DECISION TIMING")
    print("="*70)
    
    configs = grid_sweep({
        'min_steps_before_go': [2, 4, 6],
        'max_info_gathering_steps': [30, 40, 50, 60],
        'danger_high': [55, 60, 65]
    })
    
    print(f"Testing {len(configs)} configurations (~30 minutes)")
    proceed = input("Continue? (y/n): ")
    
    if proceed.lower() == 'y':
        runner = SweepRunner()
        runner.run_sweep(configs, n_episodes=10)
        runner.save("sweep5_timing.json")
        runner.print_summary()
        df = analyze_results("sweep5_timing.json")

if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║         POMDP PARAMETER SWEEP - YOUR SIMULATION                      ║
╚══════════════════════════════════════════════════════════════════════╝

Choose a sweep to run:

1. Quick Test                    (6 configs)
2. Baseline Sweep                (36 configs)
3. Sweep 1: Unseen Car Model     (36 configs) ⭐ RECOMMENDED
4. Sweep 2: Danger Thresholds    (45 configs) ⭐ RECOMMENDED
5. Sweep 3: FOV & Perception     (81 configs)
6. Sweep 4: Rewards              (54 configs)
7. Sweep 5: Decision Timing      (36 configs)

""")
    
    choice = input("Enter choice (1-7): ").strip()
    
    if choice == '1':
        run_quick_test()
    elif choice == '2':
        run_baseline_sweep()
    elif choice == '3':
        run_sweep_1_unseen_car()
    elif choice == '4':
        run_sweep_2_danger_thresholds()
    elif choice == '5':
        run_sweep_3_fov()
    elif choice == '6':
        run_sweep_4_rewards()
    elif choice == '7':
        run_sweep_5_timing()
    else:
        print("Invalid choice. Run script again.")