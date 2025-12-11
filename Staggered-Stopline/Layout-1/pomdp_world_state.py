"""
Improved POMDP Algorithm for Intersection Navigation
Fixed and completed version — runnable module.

Features:
- Proper state representation (CarState, IntersectionState)
- Physics-based transition model
- Realistic sensor observation model with occlusion handling
- Particle-based belief (particle filter)
- QMDP planning (sample-based expectation over particles and scenarios)
- Principled reward model (collision, time, creeping, goal)
- Integration controller for simulation
"""

import numpy as np
import random
from typing import List, Tuple, Dict, Optional, Any
from dataclasses import dataclass, field
from collections import defaultdict
import math
from scipy.stats import norm, truncnorm
import config as sim_config

# -------------------------
# CONFIGURATION
# -------------------------

@dataclass
class ImprovedPOMDPConfig:
    dt: float = 0.1
    av_max_speed: float = sim_config.AV_SPEED
    av_acceleration: float = 0.05
    creep_speed: float = 0.5

    cross_speed_mean: float = sim_config.CROSS_SPEED
    cross_speed_std: float = 0.5
    cross_speed_min: float = 0
    cross_speed_max: float = sim_config.CROSS_SPEED

    lane_width: float = 50
    collision_zone_width: float = 50
    stop_line_to_far_side: float = 140

    traffic_rate_per_second: float = 0.9
    occlusion_depth: float = 50.0

    detection_probability: float = 0.95
    position_noise_std: float = 0.5
    false_positive_rate: float = 0.01

    planning_horizon: int = 20
    discount_factor: float = 0.95
    num_particles: int = 100
    num_scenarios: int = 50

    collision_cost: float = -10000.0
    time_cost_per_second: float = -1.0
    creep_cost_per_second: float = -0.5
    goal_reward: float = 100.0

    min_safe_gap: float = 3.0
    collision_buffer: float = 2.0

    max_planning_time_ms: float = 50.0

CONFIG = ImprovedPOMDPConfig()


# -------------------------
# STATE REPRESENTATION
# -------------------------

@dataclass
class CarState:
    position: float
    velocity: float
    exists: bool = True

    def predict(self, dt: float) -> 'CarState':
        return CarState(position=self.position + self.velocity * dt,
                        velocity=self.velocity,
                        exists=self.exists)

@dataclass
class IntersectionState:
    av_position: float
    av_velocity: float
    left_lane_cars: List[CarState] = field(default_factory=list)
    right_lane_cars: List[CarState] = field(default_factory=list)

    def __post_init__(self):
        # Sort left ascending (more negative -> further away), right descending
        self.left_lane_cars.sort(key=lambda c: c.position)
        self.right_lane_cars.sort(key=lambda c: c.position, reverse=True)

    def copy(self) -> 'IntersectionState':
        return IntersectionState(
            av_position=self.av_position,
            av_velocity=self.av_velocity,
            left_lane_cars=[CarState(c.position, c.velocity, c.exists) for c in self.left_lane_cars],
            right_lane_cars=[CarState(c.position, c.velocity, c.exists) for c in self.right_lane_cars]
        )

@dataclass
class OcclusionRegion:
    lane: str
    near_boundary: float
    far_boundary: float

    def contains(self, position: float) -> bool:
        if self.lane == 'left':
            return self.near_boundary <= position <= self.far_boundary
        else:
            # right lane positions may decrease; interpret near/far accordingly
            return self.far_boundary <= position <= self.near_boundary


# -------------------------
# ACTIONS
# -------------------------

class Action:
    STOP = "stop"
    CREEP = "creep"
    GO = "go"
    ALL = [STOP, CREEP, GO]


# -------------------------
# OBSERVATIONS
# -------------------------

@dataclass
class Detection:
    lane: str
    position: float
    position_uncertainty: float = 0.5

@dataclass
class SensorObservation:
    detections: List[Detection] = field(default_factory=list)
    occlusions: List[OcclusionRegion] = field(default_factory=list)
    av_position: float = 0.0
    av_velocity: float = 0.0


# -------------------------
# TRANSITION MODEL
# -------------------------

class TransitionModel:
    def __init__(self, config: ImprovedPOMDPConfig = CONFIG):
        self.config = config

    def sample(self, state: IntersectionState, action: str) -> IntersectionState:
        cfg = self.config
        next_state = state.copy()

        # AV motion
        if action == Action.STOP:
            next_state.av_velocity = 0.0
        elif action == Action.CREEP:
            next_state.av_velocity = cfg.creep_speed
        elif action == Action.GO:
            next_state.av_velocity = min(state.av_velocity + cfg.av_acceleration * cfg.dt,
                                         cfg.av_max_speed)
        # integrate
        next_state.av_position = state.av_position + next_state.av_velocity * cfg.dt

        # Cross traffic: constant velocity with small noise
        for car in next_state.left_lane_cars:
            if car.exists:
                car.velocity += np.random.normal(0, 0.2)
                car.position += car.velocity * cfg.dt

        for car in next_state.right_lane_cars:
            if car.exists:
                car.velocity += np.random.normal(0, 0.2)
                car.position += car.velocity * cfg.dt

        # Remove cars far out of scene for numerical stability
        next_state.left_lane_cars = [c for c in next_state.left_lane_cars if -500 < c.position < 500]
        next_state.right_lane_cars = [c for c in next_state.right_lane_cars if -500 < c.position < 500]

        return next_state

    def is_deterministic_for_av(self) -> bool:
        return True


# -------------------------
# OBSERVATION MODEL
# -------------------------

class ObservationModel:
    def __init__(self, config: ImprovedPOMDPConfig = CONFIG):
        self.config = config

    def sample(self, state: IntersectionState, occlusions: List[OcclusionRegion]) -> SensorObservation:
        cfg = self.config
        obs = SensorObservation(av_position=state.av_position, av_velocity=state.av_velocity,
                                occlusions=occlusions.copy())

        # Helper to detect cars in a lane
        def detect_cars(cars: List[CarState], lane_name: str):
            for car in cars:
                if not car.exists:
                    continue
                # check occlusion
                is_occluded = any(occ.contains(car.position) for occ in occlusions if occ.lane == lane_name)
                if is_occluded:
                    continue
                if random.random() < cfg.detection_probability:
                    noisy_pos = car.position + np.random.normal(0, cfg.position_noise_std)
                    obs.detections.append(Detection(lane=lane_name,
                                                    position=noisy_pos,
                                                    position_uncertainty=cfg.position_noise_std))
        detect_cars(state.left_lane_cars, 'left')
        detect_cars(state.right_lane_cars, 'right')

        # False positives occasionally
        if random.random() < cfg.false_positive_rate:
            lane = random.choice(['left', 'right'])
            obs.detections.append(Detection(lane=lane, position=random.uniform(-50, 50),
                                            position_uncertainty=cfg.position_noise_std * 2))

        return obs


# -------------------------
# REWARD MODEL
# -------------------------

class RewardModel:
    def __init__(self, config: ImprovedPOMDPConfig = CONFIG):
        self.config = config

    def compute(self, state: IntersectionState, action: str, next_state: IntersectionState) -> float:
        cfg = self.config
        # collision check
        if self._collision_occurred(next_state):
            return cfg.collision_cost

        reward = 0.0
        reward += cfg.time_cost_per_second * cfg.dt
        if action == Action.CREEP:
            reward += cfg.creep_cost_per_second * cfg.dt
        if self._reached_goal(state, next_state):
            reward += cfg.goal_reward
        return reward

    def _collision_occurred(self, state: IntersectionState) -> bool:
        cfg = self.config
        # Only consider collision if AV is in collision zone
        if not (0 <= state.av_position <= cfg.collision_zone_width):
            return False
        # define zone interval around 0..collision_zone_width relative to cross-lane coordinates
        collision_x_min = -cfg.collision_buffer
        collision_x_max = cfg.collision_buffer

        for car in state.left_lane_cars + state.right_lane_cars:
            if not car.exists:
                continue
            if collision_x_min <= car.position <= collision_x_max:
                return True
        return False

    def _reached_goal(self, state: IntersectionState, next_state: IntersectionState) -> bool:
        return (state.av_position < self.config.stop_line_to_far_side and
                next_state.av_position >= self.config.stop_line_to_far_side)


# -------------------------
# BELIEF (Particle Filter)
# -------------------------

class Belief:
    def __init__(self, particles: List[IntersectionState], weights: Optional[List[float]] = None):
        assert len(particles) > 0, "Belief must have at least one particle"
        self.particles = particles
        if weights is None:
            weights = [1.0 / len(particles)] * len(particles)
        self.weights = np.array(weights, dtype=float)
        # guard against all-zero weights
        if self.weights.sum() <= 0:
            self.weights = np.array([1.0 / len(particles)] * len(particles))
        else:
            self.weights /= self.weights.sum()

    def sample(self) -> IntersectionState:
        idx = np.random.choice(len(self.particles), p=self.weights)
        return self.particles[idx].copy()

    def mean_av_state(self) -> Tuple[float, float]:
        pos = sum(w * p.av_position for w, p in zip(self.weights, self.particles))
        vel = sum(w * p.av_velocity for w, p in zip(self.weights, self.particles))
        return pos, vel

    def expected_num_cars(self, lane: str) -> float:
        counts = []
        for p in self.particles:
            cars = p.left_lane_cars if lane == 'left' else p.right_lane_cars
            counts.append(sum(1 for c in cars if c.exists))
        return float(np.average(counts, weights=self.weights))


# -------------------------
# BELIEF UPDATER
# -------------------------

class BeliefUpdater:
    def __init__(self, config: ImprovedPOMDPConfig = CONFIG):
        self.config = config
        self.obs_model = ObservationModel(config)

    def update(self, belief: Belief, action: str, observation: SensorObservation,
               transition_model: TransitionModel) -> Belief:
        cfg = self.config
        new_particles = []
        new_weights = []

        # Prediction + weighting by observation likelihood
        for particle, prior_w in zip(belief.particles, belief.weights):
            predicted = transition_model.sample(particle, action)
            likelihood = self._observation_likelihood(predicted, observation)
            if likelihood > 0:
                new_particles.append(predicted)
                new_weights.append(prior_w * likelihood)

        # If too few survive, regenerate particles consistent with observation
        if len(new_particles) < max(2, cfg.num_particles // 4):
            particles, weights = self._regenerate_particles(observation)
        else:
            particles, weights = new_particles, new_weights

        # Normalize weights and resample down/up to cfg.num_particles
        weights = np.array(weights, dtype=float)
        if weights.sum() <= 0 or np.isnan(weights.sum()):
            # fallback: uniform
            particles = [p.copy() for p in particles]
            weights = np.array([1.0 / len(particles)] * len(particles))
        else:
            weights /= weights.sum()

        if len(particles) > cfg.num_particles:
            particles, weights = self._resample(particles, weights, cfg.num_particles)
        elif len(particles) < cfg.num_particles:
            # simple duplication with jitter
            deficit = cfg.num_particles - len(particles)
            for i in range(deficit):
                idx = np.random.choice(len(particles))
                p = particles[idx].copy()
                # small jitter in positions for diversity
                for car in p.left_lane_cars + p.right_lane_cars:
                    car.position += np.random.normal(0, 0.5)
                particles.append(p)
                weights = np.append(weights, 1e-6)
            weights /= weights.sum()

        return Belief(particles, list(weights))

    def _observation_likelihood(self, state: IntersectionState, obs: SensorObservation) -> float:
        cfg = self.config
        log_like = 0.0

        # For each detection, find nearest car in that lane and score distance
        for d in obs.detections:
            cars = state.left_lane_cars if d.lane == 'left' else state.right_lane_cars
            closest_dist = float('inf')
            for car in cars:
                if not car.exists:
                    continue
                # occlusion check using observation occlusion regions
                is_occluded = any(occ.contains(car.position) for occ in obs.occlusions if occ.lane == d.lane)
                if is_occluded:
                    continue
                dist = abs(car.position - d.position)
                if dist < closest_dist:
                    closest_dist = dist
            if closest_dist < float('inf'):
                # higher probability for smaller distance; use normal pdf on offset
                log_like += norm.logpdf(closest_dist, 0, max(cfg.position_noise_std, 1e-3))
            else:
                # detection not explained by any car => likely false positive
                log_like += math.log(max(cfg.false_positive_rate, 1e-6))

        # For cars that would be visible but not detected, multiply by missed-detection prob
        for lane in ['left', 'right']:
            cars = state.left_lane_cars if lane == 'left' else state.right_lane_cars
            for car in cars:
                if not car.exists:
                    continue
                is_occluded = any(occ.contains(car.position) for occ in obs.occlusions if occ.lane == lane)
                if is_occluded:
                    continue
                # determine if any detection matches this car
                matched = any((d.lane == lane) and (abs(d.position - car.position) < 3.0) for d in obs.detections)
                if matched:
                    log_like += math.log(max(cfg.detection_probability, 1e-6))
                else:
                    log_like += math.log(max(1.0 - cfg.detection_probability, 1e-6))

        # Convert to probability
        try:
            like = math.exp(log_like)
        except OverflowError:
            like = float('inf') if log_like > 0 else 0.0
        return max(like, 0.0)

    def _regenerate_particles(self, obs: SensorObservation) -> Tuple[List[IntersectionState], List[float]]:
        cfg = self.config
        particles: List[IntersectionState] = []
        weights: List[float] = []

        for _ in range(cfg.num_particles):
            state = IntersectionState(av_position=obs.av_position, av_velocity=obs.av_velocity)
            # add detections
            for d in obs.detections:
                vel = self._sample_cross_traffic_velocity(d.lane)
                pos = d.position + np.random.normal(0, d.position_uncertainty if d.position_uncertainty > 0 else 0.1)
                car = CarState(position=pos, velocity=vel)
                if d.lane == 'left':
                    state.left_lane_cars.append(car)
                else:
                    state.right_lane_cars.append(car)
            # add occluded potential cars
            for occ in obs.occlusions:
                # expected number by Poisson with rate proportional to occlusion length / average spacing
                length = abs(occ.far_boundary - occ.near_boundary)
                lambda_rate = max(0.0, cfg.traffic_rate_per_second * (length / max(cfg.cross_speed_mean, 1.0)))
                num_hidden = np.random.poisson(lambda_rate)
                for _ in range(num_hidden):
                    if occ.lane == 'left':
                        pos = np.random.uniform(min(occ.near_boundary, occ.far_boundary),
                                                max(occ.near_boundary, occ.far_boundary))
                        vel = self._sample_cross_traffic_velocity('left')
                        state.left_lane_cars.append(CarState(pos, vel))
                    else:
                        pos = np.random.uniform(min(occ.far_boundary, occ.near_boundary),
                                                max(occ.far_boundary, occ.near_boundary))
                        vel = self._sample_cross_traffic_velocity('right')
                        state.right_lane_cars.append(CarState(pos, vel))
            particles.append(state)
            weights.append(1.0)

        # normalize weights
        weights = np.array(weights, dtype=float)
        weights /= weights.sum()
        return particles, list(weights)

    def _sample_cross_traffic_velocity(self, lane: str) -> float:
        cfg = self.config
        a = (cfg.cross_speed_min - cfg.cross_speed_mean) / cfg.cross_speed_std
        b = (cfg.cross_speed_max - cfg.cross_speed_mean) / cfg.cross_speed_std
        speed = truncnorm.rvs(a, b, loc=cfg.cross_speed_mean, scale=cfg.cross_speed_std)
        return speed if lane == 'left' else -speed

    def _resample(self, particles: List[IntersectionState], weights: List[float], num_samples: int):
        weights = np.array(weights, dtype=float)
        if weights.sum() <= 0:
            weights = np.array([1.0 / len(particles)] * len(particles))
        else:
            weights /= weights.sum()
        idxs = np.random.choice(len(particles), size=num_samples, replace=True, p=weights)
        new_particles = [particles[i].copy() for i in idxs]
        new_weights = [1.0 / num_samples] * num_samples
        return new_particles, new_weights


# -------------------------
# POLICY (QMDP)
# -------------------------

class QMDPPolicy:
    def __init__(self, config: ImprovedPOMDPConfig = CONFIG):
        self.config = config
        self.transition_model = TransitionModel(config)
        self.reward_model = RewardModel(config)
        self._value_cache: Dict[str, float] = {}

    def select_action(self, belief: Belief) -> str:
        cfg = self.config
        best_action = Action.STOP
        best_value = -float('inf')

        for action in Action.ALL:
            q_value = 0.0
            # Expectation over particles
            for particle, weight in zip(belief.particles, belief.weights):
                # sample scenarios from this particle for stochastic transitions
                scenario_vals = []
                for _ in range(max(1, cfg.num_scenarios)):
                    ns = self.transition_model.sample(particle, action)
                    immediate = self.reward_model.compute(particle, action, ns)
                    future = self._estimate_value(ns)
                    scenario_vals.append(immediate + cfg.discount_factor * future)
                q_value += weight * (float(np.mean(scenario_vals)))
            if q_value > best_value:
                best_value = q_value
                best_action = action
        return best_action

    def _estimate_value(self, state: IntersectionState) -> float:
        cfg = self.config
        key = self._state_key(state)
        if key in self._value_cache:
            return self._value_cache[key]

        # Terminal tests
        if RewardModel(cfg)._collision_occurred(state):
            val = cfg.collision_cost
        elif state.av_position >= cfg.stop_line_to_far_side:
            val = cfg.goal_reward
        else:
            # Heuristic: value increases as closer to goal and if safe gap exists
            dist = max(1e-3, cfg.stop_line_to_far_side - state.av_position)
            time_to_cross = dist / max(cfg.av_max_speed, 1e-3)
            if self._compute_safe_gap(state, time_to_cross):
                # prefer faster crossing
                time_cost = time_to_cross * cfg.time_cost_per_second
                val = cfg.goal_reward + time_cost
            else:
                # otherwise penalize waiting
                wait_est = self._estimate_wait_time(state)
                val = cfg.goal_reward + (time_to_cross + wait_est) * cfg.time_cost_per_second
        self._value_cache[key] = val
        return val

    def _compute_safe_gap(self, state: IntersectionState, time_needed: float) -> bool:
        cfg = self.config
        for cars in [state.left_lane_cars, state.right_lane_cars]:
            for car in cars:
                if not car.exists:
                    continue
                # compute time until car reaches crossing area (roughly position 0)
                if car.velocity == 0:
                    continue
                time_to_cross = (0 - car.position) / car.velocity if car.velocity != 0 else float('inf')
                if time_to_cross > 0 and time_to_cross < time_needed + cfg.min_safe_gap:
                    return False
        return True

    def _estimate_wait_time(self, state: IntersectionState) -> float:
        min_t = float('inf')
        for cars in [state.left_lane_cars, state.right_lane_cars]:
            for car in cars:
                if not car.exists or car.velocity == 0:
                    continue
                # time until car passes crossing
                t = (0 - car.position) / car.velocity
                if t > 0:
                    min_t = min(min_t, t)
        return min_t if min_t != float('inf') else 0.0

    def _state_key(self, state: IntersectionState) -> str:
        left_positions = tuple(round(c.position, 1) for c in state.left_lane_cars if c.exists)
        right_positions = tuple(round(c.position, 1) for c in state.right_lane_cars if c.exists)
        return f"{round(state.av_position,1)}_{left_positions}_{right_positions}"


# -------------------------
# MAIN AGENT
# -------------------------

class ImprovedPOMDPAgent:
    def __init__(self, config: ImprovedPOMDPConfig = CONFIG):
        self.config = config
        self.transition_model = TransitionModel(config)
        self.observation_model = ObservationModel(config)
        self.reward_model = RewardModel(config)
        self.belief_updater = BeliefUpdater(config)
        self.policy = QMDPPolicy(config)

        self.belief = self._initialize_belief()
        self.step_count = 0
        self.last_action: Optional[str] = None

    def _initialize_belief(self) -> Belief:
        cfg = self.config
        particles: List[IntersectionState] = []
        for _ in range(cfg.num_particles):
            s = IntersectionState(av_position=-5.0, av_velocity=0.0)
            # sparse initial traffic with some chance
            if random.random() < 0.3:
                pos = random.uniform(-30.0, -5.0)
                vel = self.belief_updater._sample_cross_traffic_velocity('left')
                s.left_lane_cars.append(CarState(pos, vel))
            if random.random() < 0.3:
                pos = random.uniform(5.0, 30.0)
                vel = self.belief_updater._sample_cross_traffic_velocity('right')
                s.right_lane_cars.append(CarState(pos, vel))
            particles.append(s)
        return Belief(particles)

    def decide_action(self, observation: SensorObservation, last_action: Optional[str] = None
                      ) -> Tuple[str, Dict[str, Any]]:
        self.step_count += 1

        # update belief if we have a previous action (action executed in world)
        if last_action is not None:
            self.belief = self.belief_updater.update(self.belief, last_action, observation, self.transition_model)

        # select action via policy
        action = self.policy.select_action(self.belief)

        mean_pos, mean_vel = self.belief.mean_av_state()
        expected_left = self.belief.expected_num_cars('left')
        expected_right = self.belief.expected_num_cars('right')

        info = {
            'step': self.step_count,
            'action': action,
            'belief_av_position': mean_pos,
            'belief_av_velocity': mean_vel,
            'expected_cars_left': expected_left,
            'expected_cars_right': expected_right,
            'num_particles': len(self.belief.particles),
            'detections': len(observation.detections),
            'occlusions': len(observation.occlusions)
        }

        # store last_action for next update
        self.last_action = action

        if self.step_count % 10 == 0:
            print(f"[t={self.step_count}] Action={action}, pos={mean_pos:.2f}, vel={mean_vel:.2f}, "
                  f"El={expected_left:.2f}, Er={expected_right:.2f}")

        return action, info


# -------------------------
# Helper conversion functions / integration
# -------------------------

def create_sensor_observation_from_simulation(av_position: float,
                                             av_velocity: float,
                                             visible_cars: List[Any],
                                             occlusion_boundaries: List[Dict],
                                             config: ImprovedPOMDPConfig = CONFIG,
                                             add_noise: bool = False) -> SensorObservation:
    obs = SensorObservation(av_position=av_position, av_velocity=av_velocity)
    # visible_cars: either dict-like or object-like with attributes x, vx, direction_int
    for c in visible_cars:
        if isinstance(c, dict):
            lane = 'left' if c.get('direction_int', 0) == 1 else 'right'
            x = c.get('position', c.get('x', 0.0))
        else:
            lane = 'left' if getattr(c, 'direction_int', 0) == 1 else 'right'
            x = getattr(c, 'x', getattr(c, 'position', 0.0))

        if add_noise:
            if random.random() < config.detection_probability:
                pos = x + np.random.normal(0, config.position_noise_std)
                obs.detections.append(Detection(lane=lane, position=pos, position_uncertainty=config.position_noise_std))
        else:
            obs.detections.append(Detection(lane=lane, position=x, position_uncertainty=0.0))

    for occ in occlusion_boundaries:
        obs.occlusions.append(OcclusionRegion(lane=occ['lane'], near_boundary=occ['near'], far_boundary=occ['far']))

    return obs


def extract_occlusion_boundaries_from_fov(fov_polygon: List[Tuple[float, float]],
                                         intersection_center_x: float,
                                         expected_fov_width: float = 700.0,
                                         occlusion_depth: float = 50.0) -> List[Dict]:
    if len(fov_polygon) < 2:
        return []
    fov_points = fov_polygon[1:]
    xs = [pt[0] for pt in fov_points]
    left_x = min(xs)
    right_x = max(xs)
    expected_left = intersection_center_x - expected_fov_width / 2.0
    expected_right = intersection_center_x + expected_fov_width / 2.0
    occlusions = []
    threshold = 50.0
    if left_x > expected_left + threshold:
        occlusions.append({'lane': 'left', 'near': left_x, 'far': left_x - occlusion_depth})
    if right_x < expected_right - threshold:
        occlusions.append({'lane': 'right', 'near': right_x, 'far': right_x + occlusion_depth})
    return occlusions


# -------------------------
# Integration Controller
# -------------------------

class POMDPIntersectionController:
    def __init__(self, config: Optional[ImprovedPOMDPConfig] = None):
        if config is None:
            config = ImprovedPOMDPConfig(dt=1.0/sim_config.FPS, num_particles=50, num_scenarios=20, planning_horizon=15)
        self.config = config
        self.agent = ImprovedPOMDPAgent(config)
        self.last_action = None
        print(f"[POMDP] Initialized with {config.num_particles} particles, {config.num_scenarios} scenarios.")

    def should_av_go(self, av, cross_traffic: List[Any], blockers) -> Tuple[bool, Dict]:
        intersection_line = getattr(av, 'intersection_line', sim_config.HEIGHT+sim_config.LANE_WIDTH)
        av_position = -(av.rect.top - intersection_line)  # example conversion, adjust to your sim
        av_velocity = getattr(av, 'velocity', getattr(av, 'speed', 0.0))

        # get FOV polygon from av (must be provided by your sim)
        fov_polygon = av.get_fov_polygon(blockers) if hasattr(av, 'get_fov_polygon') else [(0,0),(100,0),(100,100)]
        occlusion_boundaries = extract_occlusion_boundaries_from_fov(fov_polygon, intersection_center_x=intersection_line,
                                                                      expected_fov_width=700.0, occlusion_depth=self.config.occlusion_depth)

        obs = create_sensor_observation_from_simulation(av_position=av_position, av_velocity=av_velocity,
                                                        visible_cars=cross_traffic, occlusion_boundaries=occlusion_boundaries,
                                                        config=self.config, add_noise=False)
        action, info = self.agent.decide_action(obs, self.last_action)
        self.last_action = action

        # map to simulation commands
        if action == Action.GO:
            should_go = True
        else:
            should_go = False

        info.update({
            'av_position': av_position,
            'av_velocity': av_velocity,
            'num_visible_cars': len(cross_traffic),
            'num_occlusions': len(occlusion_boundaries),
            'occlusions': occlusion_boundaries
        })
        return should_go, info


# -------------------------
# Drop-in function
# -------------------------

_global_pomdp_agent: Optional[POMDPIntersectionController] = None

def should_av_go_pomdp(cross_traffic, av, blockers, pomdp_agent: Optional[POMDPIntersectionController] = None) -> bool:
    global _global_pomdp_agent
    if pomdp_agent is None:
        if _global_pomdp_agent is None:
            _global_pomdp_agent = POMDPIntersectionController()
        pomdp_agent = _global_pomdp_agent
    should_go, info = pomdp_agent.should_av_go(av, cross_traffic, blockers)
    return should_go


# -------------------------
# Example run
# -------------------------

def run_pomdp_agent_example():
    print("=== Improved POMDP Agent Example ===")
    agent = ImprovedPOMDPAgent()
    last_action = None

    # We'll create simple dict-based "visible cars" for the helper function
    for t in range(60):
        # simple scenario: car approaches from left starting t=10
        visible_cars = []
        if t >= 10:
            visible_cars.append({'direction_int': 1, 'position': -30 + (t - 10) * 1.0, 'vx': 12.0})

        # occlusion on right starting early
        occlusions = [{'lane': 'right', 'near': 15.0, 'far': 50.0}]

        obs = create_sensor_observation_from_simulation(
            av_position=-5.0 + t * 0.05,
            av_velocity=1.0,
            visible_cars=visible_cars,
            occlusion_boundaries=occlusions,
            add_noise=False
        )

        action, info = agent.decide_action(obs, last_action)
        last_action = action

        if action == Action.GO and t > 30:
            print(f"[t={t}] DECISION: GO - Proceeding through intersection!")
            break

    print("=== Simulation Complete ===")

# If run directly, execute example
if __name__ == "__main__":
    run_pomdp_agent_example()
