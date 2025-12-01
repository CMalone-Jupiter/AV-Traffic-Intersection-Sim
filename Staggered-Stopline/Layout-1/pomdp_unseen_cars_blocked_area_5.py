import random
import math
from typing import List, Tuple, Dict, Optional
import config
import pomdp_py
from dataclasses import dataclass
import numpy as np
import pygame

import utils
from utils import (
    travel_time,
    is_car_in_fov,
    time_to_cover_distance,
    visible_x_range_at_y,
    should_av_go_col_zone
)


@dataclass
class UnseenCarPOMDPConfig:
    """Configuration for POMDP with unseen car modeling"""

    planning_horizon: int = 10
    discount_factor: float = 0.95
    # Reward/cost structure
    cost_stop: float = -1.0
    cost_creep: float = -2.5
    reward_go_low: float = 100.0
    reward_go_medium: float = -40.0
    reward_go_high: float = -350.0

    # Sensor accuracy
    sensor_accuracy: Dict[str, float] = None

    # Creep behavior
    enable_creep_improvement: bool = True
    creep_improvement_rate: float = 0.40
    natural_change_rate: float = 0.08

    # Decision thresholds
    min_steps_before_go: int = 4
    max_info_gathering_steps: int = 40

    # FOV thresholds
    fov_blocked_threshold: float = 0.5

    enable_unseen_car_model: bool = True

    # Prior probability that an unseen car exists at FOV edge
    p_exist: float = 0.3

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

    def __post_init__(self):
        if self.sensor_accuracy is None:
            self.sensor_accuracy = {
                "low": 0.92,
                "medium": 0.75,
                "high": 0.50
            }


POMDP_CONFIG = UnseenCarPOMDPConfig()


class OcclusionState(pomdp_py.State):
    """Represents the true occlusion/danger state"""
    LEVELS = ["high", "medium", "low"]

    def __init__(self, level: str):
        if level not in self.LEVELS:
            raise ValueError(f"Invalid occlusion level: {level}")
        self.level = level

    def __hash__(self):
        return hash(self.level)

    def __eq__(self, other):
        return isinstance(other, OcclusionState) and self.level == other.level

    def __str__(self):
        return f"Occ({self.level})"


class AVAction(pomdp_py.Action):
    """AV action: stop, creep, or go"""
    TYPES = ["stop", "creep", "go"]

    def __init__(self, name: str):
        if name not in self.TYPES:
            raise ValueError(f"Invalid action: {name}")
        self.name = name

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        return isinstance(other, AVAction) and self.name == other.name

    def __str__(self):
        return self.name


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

    def __str__(self):
        return f"Obs({self.reading})"


class TransitionModel(pomdp_py.TransitionModel):

    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config
        self._build_transition_matrix()

    def _build_transition_matrix(self):
        cfg = self.config

        # STOP
        stop_trans = {
            "high": {"high": 1.0 - cfg.natural_change_rate, "medium": cfg.natural_change_rate, "low": 0.0},
            "medium": {"high": cfg.natural_change_rate / 2, "medium": 1.0 - cfg.natural_change_rate,
                       "low": cfg.natural_change_rate / 2},
            "low": {"high": 0.0, "medium": cfg.natural_change_rate, "low": 1.0 - cfg.natural_change_rate}
        }

        # CREEP
        if cfg.enable_creep_improvement:
            improvement = cfg.creep_improvement_rate
            creep_trans = {
                "high": {"high": 1.0 - improvement, "medium": improvement * 0.7, "low": improvement * 0.3},
                "medium": {"high": 0.0, "medium": 1.0 - improvement, "low": improvement},
                "low": {"high": 0.0, "medium": 0.0, "low": 1.0}
            }
        else:
            creep_trans = stop_trans

        # GO
        go_trans = {
            "high": {"high": 1.0, "medium": 0.0, "low": 0.0},
            "medium": {"high": 0.0, "medium": 1.0, "low": 0.0},
            "low": {"high": 0.0, "medium": 0.0, "low": 1.0}
        }

        self.trans = {"stop": stop_trans, "creep": creep_trans, "go": go_trans}

    def probability(self, next_state: OcclusionState, state: OcclusionState, action: AVAction) -> float:
        return self.trans[action.name][state.level][next_state.level]

    def sample(self, state: OcclusionState, action: AVAction) -> OcclusionState:
        probs = self.trans[action.name][state.level]
        r = random.random()
        cumsum = 0.0
        for level, prob in probs.items():
            cumsum += prob
            if r <= cumsum:
                return OcclusionState(level)
        return OcclusionState(state.level)


class ObservationModel(pomdp_py.ObservationModel):

    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config

    def probability(self, observation: OcclusionObservation, next_state: OcclusionState, action: AVAction) -> float:
        true_level = next_state.level
        obs_level = observation.reading
        accuracy = self.config.sensor_accuracy[true_level]

        if obs_level == true_level:
            return accuracy
        return (1.0 - accuracy) / 2.0

    def sample(self, next_state: OcclusionState, action: AVAction) -> OcclusionObservation:
        probs = [self.probability(OcclusionObservation(level), next_state, action)
                 for level in OcclusionState.LEVELS]
        r = random.random()
        cumsum = 0.0
        for level, prob in zip(OcclusionState.LEVELS, probs):
            cumsum += prob
            if r <= cumsum:
                return OcclusionObservation(level)
        return OcclusionObservation(next_state.level)


class RewardModel(pomdp_py.RewardModel):

    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config

    def _reward(self, state: OcclusionState, action: AVAction, next_state: OcclusionState) -> float:
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

    def sample(self, state: OcclusionState, action: AVAction, next_state: OcclusionState) -> float:
        return self._reward(state, action, next_state)


class AVPolicy(pomdp_py.RolloutPolicy):

    def __init__(self, config: UnseenCarPOMDPConfig, 
                 transition_model: TransitionModel,
                 reward_model: RewardModel):
        self.config = config
        self.transition_model = transition_model
        self.reward_model = reward_model
        self.step_count = 0

    def get_all_actions(self, state=None, history=None):
        return ALL_ACTIONS

    def get_belief_summary(self, belief: pomdp_py.Histogram) -> str:
        hist = belief.get_histogram()
        probs = {state.level: hist.get(state, 0.0) for state in hist.keys()}
        prob_high = probs.get("high", 0.0)
        prob_med = probs.get("medium", 0.0)
        prob_low = probs.get("low", 0.0)
        return f"P(H:{prob_high:.2f} M:{prob_med:.2f} L:{prob_low:.2f})"

    def compute_expected_reward(self, belief: pomdp_py.Histogram, action: AVAction) -> float:
      
        hist = belief.get_histogram()
        expected_reward = 0.0
        
        for state, belief_prob in hist.items():
            for next_level in OcclusionState.LEVELS:
                next_state = OcclusionState(next_level)
                trans_prob = self.transition_model.probability(next_state, state, action)
                reward = self.reward_model._reward(state, action, next_state)
                expected_reward += belief_prob * trans_prob * reward
        
        return expected_reward

    def sample(self, belief: pomdp_py.Histogram, 
               in_intersection: bool = False,
               at_edge: bool = False) -> AVAction:
        """
        Choose action with highest expected reward.
        """
        self.step_count += 1

        # IN INTERSECTION → always GO
        if in_intersection:
            return ACT_GO

        # Compute expected rewards
        value_stop = self.compute_expected_reward(belief, ACT_STOP)
        value_creep = self.compute_expected_reward(belief, ACT_CREEP)
        value_go = self.compute_expected_reward(belief, ACT_GO)

        # Print values every 5 steps
        if self.step_count % 5 == 0:
            print(f"[VALUE] stop={value_stop:.1f}, creep={value_creep:.1f}, go={value_go:.1f}")

        # AT EDGE → choose between STOP and GO
        if at_edge:
            if self.step_count < self.config.min_steps_before_go:
                return ACT_STOP
            
            if self.step_count > self.config.max_info_gathering_steps:
                return ACT_GO if value_go > value_stop else ACT_STOP
            
            # GO if expected value is positive and better than STOP
            if value_go > 0 and value_go > value_stop:
                return ACT_GO
            return ACT_STOP

        # BEFORE EDGE → choose between CREEP and GO
        # GO only if much better than CREEP
        if value_go > 0 and value_go > value_creep + 20.0:
            return ACT_GO
        
        return ACT_CREEP


class PhantomCar:
    """unseen vehicle modeling"""

    def __init__(self, direction: str, x: float, y: float):
        self.direction = direction
        self.drive_path = 'straight'
        self.turn_stage = 0
        self.acceleration = 0.1
        self.x = x
        self.y = y
        self.vx = config.CROSS_SPEED if direction == 'right' else -config.CROSS_SPEED
        self.rect = pygame.Rect(int(self.x), int(self.y), config.CAR_WIDTH, config.CAR_HEIGHT)

    def is_in_intersection(self) -> bool:
        return False


def analyze_blocked_areas(av, blockers, pomdp_config: UnseenCarPOMDPConfig = POMDP_CONFIG) -> Dict:
    """Analyze blocked areas using FOV polygon"""
    if not isinstance(blockers, list):
        blockers = [blockers] if blockers is not None else []

    fov_polygon = av.get_fov_polygon(blockers)

    if len(fov_polygon) < 3:
        return {"valid": False, "reason": "no_fov_polygon"}

    fov_points = fov_polygon[1:]
    left_x = min(pt[0] for pt in fov_points)
    right_x = max(pt[0] for pt in fov_points)
    fov_width = right_x - left_x

    expected_fov_width = config.WIDTH * 0.7
    expected_left = config.WIDTH * 0.15
    expected_right = config.WIDTH * 0.85

    left_blocked = left_x > (expected_left + pomdp_config.edge_detection_threshold)
    left_hidden_width = left_x - expected_left if left_blocked else 0

    right_blocked = right_x < (expected_right - pomdp_config.edge_detection_threshold)
    right_hidden_width = expected_right - right_x if right_blocked else 0

    intersection_center_x = config.WIDTH // 2
    left_distance_to_intersection = abs(left_x - intersection_center_x)
    right_distance_to_intersection = abs(right_x - intersection_center_x)

    total_hidden = left_hidden_width + right_hidden_width

    return {
        "valid": True,
        "fov_width": fov_width,
        "fov_ratio": fov_width / expected_fov_width,
        "left_edge_x": left_x,
        "left_blocked": left_blocked,
        "left_hidden_width": left_hidden_width,
        "left_distance_to_intersection": left_distance_to_intersection,
        "right_edge_x": right_x,
        "right_blocked": right_blocked,
        "right_hidden_width": right_hidden_width,
        "right_distance_to_intersection": right_distance_to_intersection,
        "total_hidden_width": total_hidden,
        "any_blocked": left_blocked or right_blocked,
        "both_blocked": left_blocked and right_blocked
    }


def create_phantom_cars_from_blocked_info(blocked_info: Dict) -> List[PhantomCar]:
    """Create phantom cars at blocked FOV edges"""
    phantoms: List[PhantomCar] = []

    right_moving_y = config.HEIGHT // 2 - 25
    left_moving_y = config.HEIGHT // 2 + 5

    if blocked_info.get("left_blocked", False):
        x = blocked_info["left_edge_x"] - config.CAR_WIDTH
        phantoms.append(PhantomCar(direction="right", x=x, y=right_moving_y))

    if blocked_info.get("right_blocked", False):
        x = blocked_info["right_edge_x"]
        phantoms.append(PhantomCar(direction="left", x=x, y=left_moving_y))

    return phantoms


def assess_collision_zone_danger(cross_traffic, av, blockers,
                                 pomdp_config: UnseenCarPOMDPConfig = POMDP_CONFIG) -> Tuple[float, bool]:
    """Assess danger using collision zone check with phantom cars"""
    if not isinstance(blockers, list):
        blockers = [blockers] if blockers is not None else []

    blocked_info = analyze_blocked_areas(av, blockers, pomdp_config)

    phantom_cars = []
    if blocked_info.get("valid", False) and blocked_info.get("any_blocked", False):
        phantom_cars = create_phantom_cars_from_blocked_info(blocked_info)

    cross_for_check = list(cross_traffic) + phantom_cars

    if not cross_for_check:
        return 0.0, True

    safe_to_go = should_av_go_col_zone(cross_for_check, av, blockers)

    danger = 50.0 if not safe_to_go else 0.0

    visible_cars = [car for car in cross_traffic if is_car_in_fov(car, av, blockers)]

    for car in visible_cars:
        if hasattr(car, 'is_in_intersection') and car.is_in_intersection() and getattr(car, "turn_stage", 0) < 2:
            danger += 15

    turning_cars = [car for car in visible_cars
                    if getattr(car, "drive_path", "straight") == 'right' and getattr(car, "turn_stage", 0) > 0]
    danger += len(turning_cars) * 10

    return danger, safe_to_go


def model_unseen_cars_in_blocked_areas(av, blockers,
                                       pomdp_config: UnseenCarPOMDPConfig = POMDP_CONFIG) -> Tuple[float, Dict]:
    """Model unseen cars in blocked areas"""
    if not pomdp_config.enable_unseen_car_model:
        return 0.0, {"enabled": False}

    blocked_info = analyze_blocked_areas(av, blockers, pomdp_config)

    if not blocked_info["valid"]:
        return 0.0, {"valid": False, **blocked_info}

    if blocked_info["fov_ratio"] >= pomdp_config.fov_blocked_threshold:
        return 0.0, {"fov_sufficient": True, "fov_ratio": blocked_info["fov_ratio"], **blocked_info}

    if blocked_info["total_hidden_width"] < pomdp_config.min_hidden_width_for_concern:
        return 0.0, {"hidden_width_too_small": True, "total_hidden": blocked_info["total_hidden_width"], **blocked_info}

    danger_contribution = 0.0
    unseen_cars_info = []

    req_dist_upper = 2 * config.LANE_WIDTH + 40 + config.AV_HEIGHT
    req_time_upper = time_to_cover_distance(req_dist_upper, av.acceleration, config.AV_SPEED)

    req_dist_lower = 2 * config.LANE_WIDTH + 40 + config.AV_HEIGHT
    if av.intended_direction == 'right':
        req_dist_lower += config.AV_HEIGHT // 2
    req_time_lower = time_to_cover_distance(req_dist_lower, av.acceleration, config.AV_SPEED)

    req_space_lower = req_time_lower * config.CROSS_SPEED
    req_space_upper = req_time_upper * config.CROSS_SPEED

    # Left edge
    if blocked_info["left_blocked"]:
        p_exist_left = pomdp_config.p_exist

        if pomdp_config.use_distance_weighting:
            norm_distance = blocked_info["left_distance_to_intersection"] / (config.WIDTH / 2)
            distance_multiplier = 2.0 - norm_distance
            p_exist_left *= distance_multiplier * pomdp_config.distance_weight_factor

        if pomdp_config.consider_collision_timing:
            distance_to_intersection = abs(blocked_info["left_edge_x"] - config.WIDTH // 2)
            collision_time = travel_time(distance_to_intersection, config.CROSS_SPEED, 0, config.CROSS_SPEED)

            hypothetical_x = blocked_info["left_edge_x"]
            future_x_lower = hypothetical_x + config.CROSS_SPEED * req_time_lower
            x_diff = av.x - future_x_lower

            if collision_time < pomdp_config.critical_collision_time and 0 <= x_diff < req_space_lower:
                p_exist_left *= 1.5
                p_exist_left = min(p_exist_left, 0.9)

        left_danger = p_exist_left * pomdp_config.unseen_car_danger_weight
        danger_contribution += left_danger

        unseen_cars_info.append({
            "edge": "left",
            "position_x": blocked_info["left_edge_x"],
            "hidden_width": blocked_info["left_hidden_width"],
            "p_exist": p_exist_left,
            "danger": left_danger,
            "distance_to_intersection": blocked_info["left_distance_to_intersection"]
        })

    # Right edge
    if blocked_info["right_blocked"] and pomdp_config.model_multiple_edges:
        p_exist_right = pomdp_config.p_exist

        if pomdp_config.use_distance_weighting:
            norm_distance = blocked_info["right_distance_to_intersection"] / (config.WIDTH / 2)
            distance_multiplier = 2.0 - norm_distance
            p_exist_right *= distance_multiplier * pomdp_config.distance_weight_factor

        if pomdp_config.consider_collision_timing:
            distance_to_intersection = abs(blocked_info["right_edge_x"] - config.WIDTH // 2)
            collision_time = travel_time(distance_to_intersection, config.CROSS_SPEED, 0, config.CROSS_SPEED)

            hypothetical_x = blocked_info["right_edge_x"]
            future_x_upper = hypothetical_x + config.CROSS_SPEED * req_time_upper
            x_diff = future_x_upper - av.x

            if (collision_time < pomdp_config.critical_collision_time
                    and av.intended_direction != 'left' and 0 <= x_diff < req_space_upper):
                p_exist_right *= 1.5
                p_exist_right = min(p_exist_right, 0.9)

        right_danger = p_exist_right * pomdp_config.unseen_car_danger_weight
        danger_contribution += right_danger

        unseen_cars_info.append({
            "edge": "right",
            "position_x": blocked_info["right_edge_x"],
            "hidden_width": blocked_info["right_hidden_width"],
            "p_exist": p_exist_right,
            "danger": right_danger,
            "distance_to_intersection": blocked_info["right_distance_to_intersection"]
        })

    detailed_info = {
        "enabled": True,
        "danger_contribution": danger_contribution,
        "unseen_cars": unseen_cars_info,
        "num_edges_blocked": len(unseen_cars_info),
        **blocked_info
    }
    return danger_contribution, detailed_info


class UnseenCarPOMDPAgent:
    """POMDP Agent with simple reward-based policy"""
    def __init__(self, config: UnseenCarPOMDPConfig = POMDP_CONFIG):
        self.config = config

        self.transition_model = TransitionModel(config)
        self.observation_model = ObservationModel(config)
        self.reward_model = RewardModel(config)
        
        # Simple policy that uses rewards
        self.policy = AVPolicy(config, self.transition_model, self.reward_model)

        # Initialize belief
        initial_belief = {OcclusionState(level): 1.0 / 3.0 for level in OcclusionState.LEVELS}
        self.belief = pomdp_py.Histogram(initial_belief)

        self.last_action = ACT_STOP
        self.unseen_car_history = []

    def observe_traffic_state(self, cross_traffic, av, blockers) -> str:
        """Generate observation"""
        if not isinstance(blockers, list):
            blockers = [blockers] if blockers is not None else []

        fov_polygon = av.get_fov_polygon(blockers)
        if len(fov_polygon) >= 3:
            left_x = min(pt[0] for pt in fov_polygon[1:])
            right_x = max(pt[0] for pt in fov_polygon[1:])
            fov_width = right_x - left_x
        else:
            fov_width = 0

        max_possible_fov = config.WIDTH * 0.7
        visibility_ratio = fov_width / max_possible_fov if max_possible_fov > 0 else 0

        danger_score = 0.0

        if visibility_ratio < 0.4:
            danger_score += 30
        elif visibility_ratio < 0.6:
            danger_score += 15

        collision_danger, col_zone_safe = assess_collision_zone_danger(
            cross_traffic, av, blockers, self.config
        )
        danger_score += collision_danger

        unseen_danger, unseen_info = model_unseen_cars_in_blocked_areas(
            av, blockers, self.config
        )
        danger_score += unseen_danger

        if unseen_info.get('enabled') and unseen_danger > 0:
            self.unseen_car_history.append({
                'step': self.policy.step_count,
                'danger': unseen_danger,
                'collision_zone_safe': col_zone_safe,
                'collision_danger': collision_danger,
                'info': unseen_info
            })

        if danger_score > 60 or visibility_ratio < 0.3:
            obs = "high"
        elif danger_score > 30 or visibility_ratio < 0.6:
            obs = "medium"
        else:
            obs = "low"

        if self.policy.step_count % 5 == 0:
            print(f"[OBS] danger={danger_score:.1f}, vis={visibility_ratio:.2f}, obs={obs}")

        return obs

    def update_belief(self, observation_str: str):
        """Bayesian belief update"""
        observation = OcclusionObservation(observation_str)
        current_belief = self.belief.get_histogram()
        new_belief = {}

        for next_level in OcclusionState.LEVELS:
            next_state = OcclusionState(next_level)
            obs_prob = self.observation_model.probability(observation, next_state, self.last_action)

            if obs_prob == 0:
                continue

            transition_sum = 0.0
            for state, belief_weight in current_belief.items():
                trans_prob = self.transition_model.probability(next_state, state, self.last_action)
                transition_sum += trans_prob * belief_weight

            new_belief[next_state] = obs_prob * transition_sum

        total = sum(new_belief.values())
        if total > 0:
            for state in new_belief:
                new_belief[state] /= total
        else:
            new_belief = {OcclusionState(level): 1.0 / 3.0 for level in OcclusionState.LEVELS}

        self.belief = pomdp_py.Histogram(new_belief)

    def decide_action(self, cross_traffic, av, blockers, verbose=False) -> Tuple[str, Dict]:
        """Main decision function"""
        intersection_line = config.HEIGHT // 2 + config.LANE_WIDTH
        edge_start = intersection_line - 5

        av_top = av.rect.top

        in_intersection = av_top < edge_start
        at_edge = (edge_start <= av_top <= intersection_line) and not in_intersection

        observation_str = self.observe_traffic_state(cross_traffic, av, blockers)
        self.update_belief(observation_str)

        belief_summary = self.policy.get_belief_summary(self.belief)

        
        action = self.policy.sample(
            belief=self.belief,
            in_intersection=in_intersection,
            at_edge=at_edge
        )
        self.last_action = action

        blocked_info = analyze_blocked_areas(av, blockers, self.config)

        info = {
            "observation": observation_str,
            "belief": belief_summary,
            "action": action.name,
            "step": self.policy.step_count,
            "in_intersection": in_intersection,
            "at_edge": at_edge,
            "av_position": av_top,
            "fov_ratio": blocked_info.get("fov_ratio", 0),
            "blocked_areas": blocked_info,
            "unseen_car_model_active": blocked_info.get("any_blocked", False)
            and self.config.enable_unseen_car_model
        }

        position_state = "IN_INTERSECTION" if in_intersection else ("AT_EDGE" if at_edge else "BEFORE_EDGE")
        time_sec = self.policy.step_count / config.FPS
        print(f"[POMDP t={time_sec:.2f}s] {belief_summary} → {action.name.upper()} "
              f"({position_state}, pos={av_top:.1f})")

        return action.name, info

    def get_unseen_car_summary(self) -> Dict:
        """Get summary of unseen car modeling history"""
        if not self.unseen_car_history:
            return {"active": False}

        total_danger = sum(h['danger'] for h in self.unseen_car_history)
        avg_danger = total_danger / len(self.unseen_car_history)

        return {
            "active": True,
            "num_activations": len(self.unseen_car_history),
            "total_danger_added": total_danger,
            "avg_danger_per_activation": avg_danger,
            "last_activation_step": self.unseen_car_history[-1]['step']
        }


def should_av_go_pomdp(cross_traffic, av, blockers, pomdp_agent=None):
    if pomdp_agent is None:
        pomdp_agent = UnseenCarPOMDPAgent()

    if not isinstance(blockers, list):
        blockers = [blockers] if blockers is not None else []

    action, info = pomdp_agent.decide_action(cross_traffic, av, blockers)

    at_edge = info.get('at_edge', False)
    av_pos = info.get('av_position', av.rect.top)

    if action == "go":
        visible_danger = False
        for car in cross_traffic:
            if utils.is_car_in_fov(car, av, blockers):
                if car.direction == 'left':
                    x_diff = (car.x + config.CAR_WIDTH) - av.x
                    if -50 < x_diff < 200:
                        visible_danger = True
                        break
                else:
                    x_diff = av.x - car.x
                    if -50 < x_diff < 200:
                        visible_danger = True
                        break

        if visible_danger:
            if pomdp_agent.policy.step_count % 5 == 0:
                print(f"[OVERRIDE] GO blocked by safety check")
            av.inching = False
            av.inch_behave = False
            av.moving = False
            return False

        av.inching = False
        av.inch_behave = False
        av.moving = True
        print(f"[POMDP→SIM] ✓ GO (pos={av_pos:.1f})")
        return True

    elif action == "stop":
        av.inching = False
        av.inch_behave = False
        av.moving = False

    elif action == "creep":
        av.inch_behave = True
        av.inching = True
        av.moving = False
        if pomdp_agent.policy.step_count % 10 == 0:
            print(f"[POMDP→SIM] ⚠ CREEP (pos={av_pos:.1f})")
        return False

    return False


if __name__ == "__main__":
    print(f"\n=== SIMPLIFIED REWARD-BASED POMDP ===")
    print(f"Rewards: GO_low={POMDP_CONFIG.reward_go_low}, GO_med={POMDP_CONFIG.reward_go_medium}, GO_high={POMDP_CONFIG.reward_go_high}")
    print(f"Costs: STOP={POMDP_CONFIG.cost_stop}, CREEP={POMDP_CONFIG.cost_creep}")