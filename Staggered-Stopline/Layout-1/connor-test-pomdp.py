import random
import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field
from enum import Enum
import numpy as np
import pomdp_py

# test 1
import config
import utils
from utils import (
    travel_time,
    is_car_in_fov,
    time_to_cover_distance,
    should_av_go_col_zone,
    poly_find_x
)


class VehiclePosition(Enum):
    """Enum for cleaner position tracking"""
    BEFORE_EDGE = "before_edge"
    AT_EDGE = "at_edge"
    IN_INTERSECTION = "in_intersection"


@dataclass
class UnseenCarPOMDPConfig:
    """Configuration with better organization and defaults"""
    # Reward structure
    cost_stop: float = -1.0
    cost_creep: float = -2.5
    reward_go_low: float = 100.0
    reward_go_medium: float = -40.0
    reward_go_high: float = -350.0
    
    # Sensor model
    sensor_accuracy: Dict[str, float] = field(default_factory=lambda: {
        "low": 0.92,
        "medium": 0.75,
        "high": 0.50
    })
    
    # Dynamics
    enable_creep_improvement: bool = True
    creep_improvement_rate: float = 0.40
    natural_change_rate: float = 0.08
    
    # Decision thresholds
    confidence_threshold_go: float = 0.92
    danger_threshold_stop: float = 0.40
    min_steps_before_go: int = 4
    max_info_gathering_steps: int = 40
    
    # **NEW: Creep delay mechanism**
    creep_delay_steps: int = 256  # Wait 8 steps (~0.25s at 30fps) before creeping
    min_fov_blockage_for_creep: float = 60  # Only creep if FOV < 40%
    
    # FOV thresholds
    fov_blocked_threshold: float = 0.5
    fov_severely_blocked_threshold: float = 0.3
    
    # Unseen car modeling
    enable_unseen_car_model: bool = True
    p_exist: float = 0.3
    unseen_car_danger_weight: float = 35.0
    edge_detection_threshold: float = 50.0
    min_hidden_width_for_concern: float = 80.0
    
    # Distance weighting
    use_distance_weighting: bool = True
    distance_weight_factor: float = 0.5
    model_multiple_edges: bool = True
    
    # Timing
    consider_collision_timing: bool = True
    critical_collision_time: float = 3.0
    
    discount_factor: float = 0.95


POMDP_CONFIG = UnseenCarPOMDPConfig()


# ============================================================================
# STATE / ACTION / OBSERVATION (unchanged structure, better __repr__)
# ============================================================================

class OcclusionState(pomdp_py.State):
    """Occlusion danger level"""
    LEVELS = ["high", "medium", "low"]
    
    def __init__(self, level: str):
        if level not in self.LEVELS:
            raise ValueError(f"Invalid level: {level}")
        self.level = level
    
    def __hash__(self):
        return hash(self.level)
    
    def __eq__(self, other):
        return isinstance(other, OcclusionState) and self.level == other.level
    
    def __repr__(self):
        return f"Occ({self.level[0].upper()})"


class AVAction(pomdp_py.Action):
    """AV action"""
    TYPES = ["stop", "creep", "go"]
    
    def __init__(self, name: str):
        if name not in self.TYPES:
            raise ValueError(f"Invalid action: {name}")
        self.name = name
    
    def __hash__(self):
        return hash(self.name)
    
    def __eq__(self, other):
        return isinstance(other, AVAction) and self.name == other.name
    
    def __repr__(self):
        return self.name.upper()


ACT_STOP = AVAction("stop")
ACT_CREEP = AVAction("creep")
ACT_GO = AVAction("go")
ALL_ACTIONS = {ACT_STOP, ACT_CREEP, ACT_GO}


class OcclusionObservation(pomdp_py.Observation):
    def __init__(self, reading: str):
        if reading not in OcclusionState.LEVELS:
            raise ValueError(f"Invalid observation: {reading}")
        self.reading = reading
    
    def __hash__(self):
        return hash(self.reading)
    
    def __eq__(self, other):
        return isinstance(other, OcclusionObservation) and self.reading == other.reading
    
    def __repr__(self):
        return f"Obs({self.reading[0].upper()})"


# ============================================================================
# EFFICIENT CACHING FOR FOV COMPUTATION
# ============================================================================

class FOVCache:
    """Cache FOV computations to avoid redundant polygon calculations"""
    def __init__(self):
        self._cache = {}
        self._last_av_pos = None
        
    def get_fov_info(self, av, blockers) -> Dict:
        """Get or compute FOV information with caching"""
        # Cache key based on AV position (round to nearest 5 pixels)
        av_key = (round(av.x / 5) * 5, round(av.rect.top / 5) * 5)
        blocker_key = tuple(id(b) for b in blockers) if blockers else ()
        cache_key = (av_key, blocker_key)
        
        if cache_key in self._cache:
            return self._cache[cache_key]
        
        # Compute FOV info
        # fov_polygon = av.get_fov_polygon(blockers)
        visible_range_lower, fov_polygon = poly_find_x(av, blockers, config.HEIGHT/2+config.LANE_WIDTH/2, side='right')
        visible_range_upper, fov_polygon = poly_find_x(av, blockers, config.HEIGHT/2-config.LANE_WIDTH/2, side='left')
        
        if len(fov_polygon) < 3:
            info = {
                "valid": False,
                "fov_width": 0,
                "visibility_ratio": 0,
                "left_edge_x": 0,
                "right_edge_x": 0,
                "left_blocked": True,
                "right_blocked": True
            }
        else:
            # fov_points = fov_polygon[1:]
            # left_x = min(pt[0] for pt in fov_points)
            # right_x = max(pt[0] for pt in fov_points)
            left_x = float(max(0, visible_range_upper))
            right_x = float(min(config.WIDTH, visible_range_lower))
            fov_width = right_x - left_x
            
            expected_fov_width = config.WIDTH * 0.5
            visibility_ratio = max(left_x/(0.25*config.WIDTH), (config.WIDTH-right_x)/(config.WIDTH-0.75*config.WIDTH))
            # print(f"left: {left_x}, right: {right_x}, visibility: {visibility_ratio}")
            
            # expected_left = config.WIDTH * 0.15
            # expected_right = config.WIDTH * 0.85
            
            info = {
                "valid": True,
                "fov_width": fov_width,
                "visibility_ratio": visibility_ratio,
                "left_edge_x": left_x,
                "right_edge_x": right_x,
                "left_blocked": left_x > 0.25*config.WIDTH,
                "right_blocked": right_x < 0.75*config.WIDTH
                # "left_hidden_width": max(0, left_x - expected_left),
                # "right_hidden_width": max(0, expected_right - right_x)
            }
        
        # Cache with size limit
        if len(self._cache) > 50:
            self._cache.pop(next(iter(self._cache)))
        
        self._cache[cache_key] = info
        return info
    
    def clear(self):
        """Clear cache (call when scenario resets)"""
        self._cache.clear()


# ============================================================================
# EFFICIENT BELIEF REPRESENTATION
# ============================================================================

class CompactBelief:
    """More efficient belief representation using numpy"""
    LEVEL_TO_IDX = {"high": 0, "medium": 1, "low": 2}
    IDX_TO_LEVEL = ["high", "medium", "low"]
    
    def __init__(self, probs: Optional[np.ndarray] = None):
        if probs is None:
            self.probs = np.ones(3) / 3.0  # Uniform prior
        else:
            self.probs = probs / probs.sum()  # Normalize
    
    def __getitem__(self, level: str) -> float:
        return self.probs[self.LEVEL_TO_IDX[level]]
    
    def update(self, obs_level: str, action: AVAction, 
               trans_model: 'TransitionModel', obs_model: 'ObservationModel'):
        """Efficient vectorized belief update"""
        # Get observation probabilities for each state
        obs_probs = np.array([
            obs_model.probability_fast(obs_level, level, action)
            for level in self.IDX_TO_LEVEL
        ])
        
        # Get transition matrix for this action
        trans_matrix = trans_model.get_matrix(action.name)
        
        # Belief update: P(s'|o,a) ∝ P(o|s',a) * Σ_s P(s'|s,a) * P(s)
        predicted_belief = trans_matrix.T @ self.probs
        new_probs = obs_probs * predicted_belief
        
        # Normalize
        total = new_probs.sum()
        if total > 1e-10:
            self.probs = new_probs / total
        else:
            self.probs = np.ones(3) / 3.0
    
    def summary(self) -> str:
        return f"P(H:{self.probs[0]:.2f} M:{self.probs[1]:.2f} L:{self.probs[2]:.2f})"


# ============================================================================
# MODELS WITH PRECOMPUTED MATRICES
# ============================================================================

class TransitionModel(pomdp_py.TransitionModel):
    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config
        self._build_matrices()
    
    def _build_matrices(self):
        """Precompute transition matrices for efficiency"""
        cfg = self.config
        
        # Stop matrix
        stop_matrix = np.array([
            [1.0 - cfg.natural_change_rate, cfg.natural_change_rate, 0.0],
            [cfg.natural_change_rate/2, 1.0 - cfg.natural_change_rate, cfg.natural_change_rate/2],
            [0.0, cfg.natural_change_rate, 1.0 - cfg.natural_change_rate]
        ])
        
        # Creep matrix
        if cfg.enable_creep_improvement:
            imp = cfg.creep_improvement_rate
            creep_matrix = np.array([
                [1.0 - imp, imp * 0.7, imp * 0.3],
                [0.0, 1.0 - imp, imp],
                [0.0, 0.0, 1.0]
            ])
        else:
            creep_matrix = stop_matrix.copy()
        
        # Go matrix (absorbing)
        go_matrix = np.eye(3)
        
        self.matrices = {
            "stop": stop_matrix,
            "creep": creep_matrix,
            "go": go_matrix
        }
    
    def get_matrix(self, action_name: str) -> np.ndarray:
        return self.matrices[action_name]
    
    def probability(self, next_state: OcclusionState, 
                   state: OcclusionState, action: AVAction) -> float:
        s_idx = CompactBelief.LEVEL_TO_IDX[state.level]
        ns_idx = CompactBelief.LEVEL_TO_IDX[next_state.level]
        return self.matrices[action.name][s_idx, ns_idx]


class ObservationModel(pomdp_py.ObservationModel):
    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config
        # Precompute observation matrix
        self._build_obs_matrix()
    
    def _build_obs_matrix(self):
        """Precompute observation probabilities"""
        self.obs_matrix = {}
        for true_level in OcclusionState.LEVELS:
            acc = self.config.sensor_accuracy[true_level]
            noise = (1.0 - acc) / 2.0
            self.obs_matrix[true_level] = {
                true_level: acc,
                **{level: noise for level in OcclusionState.LEVELS if level != true_level}
            }
    
    def probability_fast(self, obs_level: str, true_level: str, action: AVAction) -> float:
        """Fast lookup without object instantiation"""
        return self.obs_matrix[true_level][obs_level]
    
    def probability(self, observation: OcclusionObservation, 
                   next_state: OcclusionState, action: AVAction) -> float:
        return self.probability_fast(observation.reading, next_state.level, action)


class RewardModel(pomdp_py.RewardModel):
    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config
    
    def _reward(self, state: OcclusionState, action: AVAction, 
                next_state: OcclusionState) -> float:
        cfg = self.config
        if action.name == "stop":
            return cfg.cost_stop
        elif action.name == "creep":
            return cfg.cost_creep
        elif action.name == "go":
            return {
                "low": cfg.reward_go_low,
                "medium": cfg.reward_go_medium,
                "high": cfg.reward_go_high
            }[next_state.level]
        return 0.0
    
    def sample(self, state: OcclusionState, action: AVAction, 
               next_state: OcclusionState) -> float:
        return self._reward(state, action, next_state)


# ============================================================================
# IMPROVED POLICY WITH CREEP DELAY
# ============================================================================

class POMDPPolicy(pomdp_py.RolloutPolicy):
    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config
        self.step_count = 0
        
        # **NEW: Creep delay tracking**
        self.steps_blocked = 0  # How many steps FOV has been blocked
        self.fov_was_blocked_last_step = False

    def get_belief_summary(self, belief: pomdp_py.Histogram) -> str:
        # hist = belief.get_histogram()
        hist = {
                "high": belief["high"],
                "medium": belief["medium"],
                "low": belief["low"]
                }
        probs = {state.level: hist.get(state, 0.0) for state in hist.keys()}
        prob_high = probs.get("high", 0.0)
        prob_med = probs.get("medium", 0.0)
        prob_low = probs.get("low", 0.0)
        return f"P(H:{prob_high:.2f} M:{prob_med:.2f} L:{prob_low:.2f})"
        
    def get_all_actions(self, state=None, history=None):
        return ALL_ACTIONS
    
    def sample(self, belief: CompactBelief, position: VehiclePosition,
               fov_visibility: float) -> AVAction:
        """
        Improved action selection with creep delay
        
        Args:
            belief: Current belief state
            position: Vehicle position enum
            fov_visibility: Current FOV visibility ratio (0-1)
        """
        self.step_count += 1
        
        prob_high = belief["high"]
        prob_med = belief["medium"]
        prob_low = belief["low"]
        cfg = self.config
        
        # IN INTERSECTION: Always go
        if position == VehiclePosition.IN_INTERSECTION:
            return ACT_GO
        
        # AT EDGE: Choose between STOP or GO
        if position == VehiclePosition.AT_EDGE:
            # Minimum steps check
            if self.step_count < cfg.min_steps_before_go:
                return ACT_STOP
            
            # High confidence to go
            if prob_low > 0.85 and prob_high < 0.10:
                return ACT_GO
            
            # Timeout check
            if self.step_count > cfg.max_info_gathering_steps:
                if prob_high < 0.3 and prob_low > 0.50:
                    return ACT_GO
            
            return ACT_STOP
        
        # BEFORE EDGE: Decide between STOP and CREEP
        if cfg.enable_creep_improvement:
            # **NEW LOGIC: Only creep after delay AND if FOV is sufficiently blocked*
            
            # Check if FOV is blocked enough to warrant creeping
            fov_blocked = fov_visibility > 1
            
            # Track consecutive blocked steps
            if fov_blocked:
                if self.fov_was_blocked_last_step:
                    self.steps_blocked += 1
                else:
                    self.steps_blocked = 1  # Reset counter
            else:
                self.steps_blocked = 0
            
            self.fov_was_blocked_last_step = fov_blocked

            # print(f"CREEP DEBUG: fov_visibility={fov_visibility:.2f}, "
            #       f"fov_blocked={fov_blocked}, "
            #       f"prob_low={prob_low:.2f}, prob_high={prob_high:.2f}, "
            #       f"steps_blocked={self.steps_blocked}")
            
            # Only creep if:
            # 1. FOV is blocked enough
            # 2. We've been blocked for enough steps
            # 3. We're not too confident it's already safe
            if (fov_blocked):
                if (self.steps_blocked >= cfg.creep_delay_steps and prob_low < 0.80):
                    return ACT_CREEP
                else:
                    return ACT_STOP
            else:
                if self.step_count < cfg.min_steps_before_go:
                    return ACT_STOP
                
                # High confidence to go
                if prob_low > 0.85 and prob_high < 0.10:
                    return ACT_GO
                
                # Timeout check
                if self.step_count > cfg.max_info_gathering_steps:
                    if prob_high < 0.3 and prob_low > 0.50:
                        return ACT_GO
                
                return ACT_STOP
        else:
            # No creeping - use same logic as at edge
            print(f"prob_low: {prob_low}, prob_high: {prob_high}")
            if self.step_count < cfg.min_steps_before_go:
                return ACT_STOP
            if prob_low > 0.85 and prob_high < 0.10:
                return ACT_GO
            if self.step_count > cfg.max_info_gathering_steps:
                if prob_high < 0.3 and prob_low > 0.50:
                    return ACT_GO
            return ACT_STOP
    
    def rollout(self, state, history=None):
        return random.choice(list(ALL_ACTIONS))


# ============================================================================
# POSITION DETECTION (centralized)
# ============================================================================

def detect_vehicle_position(av) -> VehiclePosition:
    """Centralized position detection to avoid duplication"""
    intersection_line = config.HEIGHT // 2 + config.LANE_WIDTH
    edge_start = intersection_line - 5
    av_top = av.rect.top
    
    if av_top < edge_start:
        return VehiclePosition.IN_INTERSECTION
    elif edge_start <= av_top <= intersection_line:
        return VehiclePosition.AT_EDGE
    else:
        return VehiclePosition.BEFORE_EDGE


# ============================================================================
# MAIN AGENT (using improved components)
# ============================================================================

class UnseenCarPOMDPAgent:
    """Improved POMDP agent with efficiency optimizations"""
    
    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config
        
        # Models
        self.transition_model = TransitionModel(config)
        self.observation_model = ObservationModel(config)
        self.reward_model = RewardModel(config)
        self.policy = POMDPPolicy(config)
        
        # Efficient belief
        self.belief = CompactBelief()
        
        # Caching
        self.fov_cache = FOVCache()
        
        # State
        self.last_action = ACT_STOP
        self.unseen_car_history = []
    
    def observe_traffic_state(self, cross_traffic, av, blockers,
                             fov_info: Dict) -> str:
        """Generate observation using cached FOV info"""
        visibility_ratio = fov_info["visibility_ratio"]
        
        # Base danger from visibility
        danger_score = 0.0
        if visibility_ratio > 1:
            danger_score += 30
        elif visibility_ratio > 0.85:
            danger_score += 15
        
        # Collision zone danger
        col_zone_safe = should_av_go_col_zone(cross_traffic, av, blockers)
        if not col_zone_safe:
            danger_score += 50
        
        # Unseen car modeling (simplified for brevity - keep your logic)
        if self.config.enable_unseen_car_model and fov_info["valid"]:
            if fov_info.get("left_blocked") or fov_info.get("right_blocked"):
                # Your unseen car danger calculation here
                unseen_danger = self.config.p_exist * self.config.unseen_car_danger_weight
                danger_score += unseen_danger
        
        # Determine observation
        if danger_score > 60 or visibility_ratio > 1.2:
            return "high"
        elif danger_score > 30 or visibility_ratio > 1:
            return "medium"
        else:
            return "low"
    
    def decide_action(self, cross_traffic, av, blockers) -> Tuple[str, Dict]:
        """Main decision function with improvements"""
        
        # Get position once
        position = detect_vehicle_position(av)
        
        # Get cached FOV info
        blockers_list = blockers if isinstance(blockers, list) else [blockers]
        fov_info = self.fov_cache.get_fov_info(av, blockers_list)
        
        # Observe
        obs_str = self.observe_traffic_state(cross_traffic, av, blockers_list, fov_info)
        
        # Update belief efficiently
        self.belief.update(obs_str, self.last_action, 
                          self.transition_model, self.observation_model)
        
        # Decide action with position and visibility info
        action = self.policy.sample(self.belief, position, fov_info["visibility_ratio"])
        self.last_action = action
        
        # Build info dict
        time_sec = self.policy.step_count / config.FPS
        info = {
            "observation": obs_str,
            "belief": self.belief.summary(),
            "action": action.name,
            "step": self.policy.step_count,
            "time_sec": time_sec,
            "position": position.value,
            "fov_visibility": fov_info["visibility_ratio"],
            "steps_blocked": self.policy.steps_blocked,
            "creep_active": action.name == "creep",
            "at_edge": position == VehiclePosition.AT_EDGE,
            "in_intersection": position == VehiclePosition.IN_INTERSECTION,
            "blocked_steps": self.policy.step_count
        }
        
        # Logging
        if self.policy.step_count % 5 == 0 or action.name == "go":
            pos_str = position.name
            belief_str = self.belief.summary()
            print(f"[POMDP t={time_sec:.2f}s] {belief_str} → {action.name.upper()} "
                  f"({pos_str}, FOV={1-fov_info['visibility_ratio']:.0%}, "
                  f"blocked={self.policy.steps_blocked})")
        
        return action.name, info
    
    def reset(self):
        """Reset agent state for new scenario"""
        self.belief = CompactBelief()
        self.policy.step_count = 0
        self.policy.steps_blocked = 0
        self.policy.fov_was_blocked_last_step = False
        self.last_action = ACT_STOP
        self.fov_cache.clear()
        self.unseen_car_history.clear()


# ============================================================================
# INTEGRATION FUNCTION
# ============================================================================

def should_av_go_pomdp(cross_traffic, av, blockers, pomdp_agent=None):
    """Main integration function with safety override"""
    
    if pomdp_agent is None:
        pomdp_agent = UnseenCarPOMDPAgent()
    
    blockers_list = blockers if isinstance(blockers, list) else ([blockers] if blockers else [])
    
    # Get POMDP decision
    action, info = pomdp_agent.decide_action(cross_traffic, av, blockers_list)
    # print(f"DEBUG: enable_creep={pomdp_agent.config.enable_creep_improvement}, "
    #   f"action={action}, at_edge={info['at_edge']}, in_intersection={info['in_intersection']}, "
    #   f"av.inch_behave={av.inch_behave}, av.inching={av.inching}, blocked steps={info['blocked_steps']}")
    
    # Safety override for GO action
    if action == "go":
        # Double-check with visible cars
        # for car in cross_traffic:
        #     if utils.is_car_in_fov(car, av, blockers_list):
        #         if car.direction == 'left':
        #             x_diff = (car.x + config.CAR_WIDTH) - av.x
        #             if -50 < x_diff < 200:
        #                 print(f"[OVERRIDE] GO blocked by visible car safety check")
        #                 av.moving = False
        #                 av.inching = False
        #                 return False
        #         else:
        #             x_diff = av.x - car.x
        #             if -50 < x_diff < 200:
        #                 print(f"[OVERRIDE] GO blocked by visible car safety check")
        #                 av.moving = False
        #                 av.inching = False
        #                 return False
        
        # Safe to go
        av.moving = True
        av.inching = False
        # av.inch_behave = False
        print(f"[POMDP→SIM] ✓ GO (t={info['time_sec']:.2f}s)")
        return True
    
    elif action == "stop":
        av.moving = False
        av.inching = False
        # av.inch_behave = False
        return False
    
    elif action == "creep":
        av.moving = False
        # av.inch_behave = True
        av.inching = True
        if info['steps_blocked'] == info['step']:  # First creep
            print(f"[POMDP→SIM] ⚠ Starting CREEP after {info['steps_blocked']} blocked steps")
        return False
    
    return False


# if __name__ == "__main__":
    # print("="*70)
    # print("IMPROVED POMDP WITH DELAYED CREEP BEHAVIOR")
    # print("="*70)
    # print("\nImprovements:")
    # print("  ✓ Creep delay: waits 8 steps (~0.25s) before creeping")
    # print("  ✓ FOV blockage check: only creeps if visibility < 40%")
    # print("  ✓ Efficient caching: FOV polygon computed once per position")
    # print("  ✓ Vectorized belief updates using numpy")
    # print("  ✓ Precomputed transition/observation matrices")
    # print("  ✓ Centralized position detection")
    # print("  ✓ Better logging with time tracking")
    # print(f"\nConfig: creep_delay_steps = {POMDP_CONFIG.creep_delay_steps}")
    # print(f"Config: min_fov_blockage_for_creep = {POMDP_CONFIG.min_fov_blockage_for_creep:.0%}")
    # print("="*70)