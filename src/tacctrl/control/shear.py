from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Protocol

import numpy as np

from tacctrl.config import FingerConfig, ModuleConfig, ShearControllerConfig, build_default_config
from tacctrl.control.pid import BoundedPID
from tacctrl.kinematics.dexh13_hand import DexH13HandModel
from tacctrl.tactile import ModuleShearFeatures, ShearFeatureExtractor

LOG = logging.getLogger(__name__)


class HandInterface(Protocol):
    def get_joint_positions_radian(self) -> np.ndarray: ...

    def get_finger_tactile(self) -> np.ndarray: ...

    def set_joint_positions_radian(self, joint_values: List[float]) -> None: ...


@dataclass
class ModuleRuntime:
    config: ModuleConfig
    pid: BoundedPID
    last_features: Optional[ModuleShearFeatures] = None
    reference_position: Optional[np.ndarray] = None
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))


class ShearGraspController:
    """Implements the admittance-based shear control described in guidance.md."""

    _CONTACT_FINGERS = ("thumb", "index", "middle")

    def __init__(
        self,
        *,
        hardware: HandInterface,
        kinematics: DexH13HandModel,
        config: Optional[ShearControllerConfig] = None,
    ) -> None:
        self.hardware = hardware
        self.kinematics = kinematics
        self.config = config or build_default_config()

        self._joint_count = len(tuple(self.config.joint_order))
        self._gravity_world = np.array([0.0, 0.0, -1.0], dtype=float)
        self._gravity_world /= max(np.linalg.norm(self._gravity_world), 1e-9)

        self._module_runtime: Dict[str, ModuleRuntime] = {}
        module_indices: List[int] = []
        for finger_cfg in self.config.finger_configs:
            for module_cfg in finger_cfg.modules:
                gains = self.config.module_gains.get(module_cfg.name)
                if gains is None:
                    raise KeyError(f"Module '{module_cfg.name}' missing gains configuration")
                self._module_runtime[module_cfg.name] = ModuleRuntime(
                    config=module_cfg,
                    pid=BoundedPID(gains.kp, gains.ki, gains.kd, gains.integral_limit),
                )
                module_indices.append(module_cfg.tactile_index)

        expected_count = max(module_indices, default=-1) + 1
        self._expected_module_count = max(expected_count, len(self.config.module_gains))

        self._extractor = ShearFeatureExtractor(
            modules=[name for name in self.config.module_gains],
            lowpass_alpha=self.config.shear_lowpass_alpha,
            diff_window=self.config.shear_rate_window,
            derivative_alpha=self.config.shear_derivative_alpha,
        )

        self._joint_name_to_index = {name: idx for idx, name in enumerate(self.config.joint_order)}
        self._joint_name_to_velocity = kinematics.velocity_indices()

        self._mass = np.asarray(self.config.admittance_mass, dtype=float)
        self._damping = np.asarray(self.config.admittance_damping, dtype=float)
        self._stiffness = np.asarray(self.config.admittance_stiffness, dtype=float)
        if self._mass.shape != (3,) or self._damping.shape != (3,) or self._stiffness.shape != (3,):
            raise ValueError("admittance_mass/damping/stiffness must be length-3 sequences")
        self._dls_lambda = float(self.config.admittance_damping_lambda)
        self._joint_step_limit = float(self.config.joint_step_limit)
        self._tactile_scale = float(self.config.tactile_count_to_newton)

        if self.config.initial_joint_targets:
            if len(self.config.initial_joint_targets) != self._joint_count:
                raise ValueError("initial_joint_targets must match joint_order length")
            self._initial_targets = np.asarray(self.config.initial_joint_targets, dtype=float)
        else:
            self._initial_targets = None

        if self.config.pregrasp_joint_targets:
            if len(self.config.pregrasp_joint_targets) != self._joint_count:
                raise ValueError("pregrasp_joint_targets must match joint_order length")
            self._pregrasp_targets = np.asarray(self.config.pregrasp_joint_targets, dtype=float)
        else:
            self._pregrasp_targets = None

        self._pregrasp_tolerance = float(self.config.pregrasp_tolerance)
        self._pregrasp_kp = float(self.config.pregrasp_kp)
        self._contact_threshold = float(self.config.finger_contact_force_threshold)
        self._normal_setpoint = float(self.config.normal_force_setpoint)

    # ------------------------------------------------------------------ helpers
    def reset(self) -> None:
        for runtime in self._module_runtime.values():
            runtime.pid.reset()
            runtime.last_features = None
            runtime.velocity[:] = 0.0

    def _read_joint_positions(self) -> np.ndarray:
        values = np.asarray(self.hardware.get_joint_positions_radian(), dtype=float)
        if values.size < self._joint_count:
            padded = np.zeros(self._joint_count, dtype=float)
            padded[: values.size] = values
            return padded
        return values[: self._joint_count]

    def _read_tactile_modules(self) -> np.ndarray:
        raw = np.asarray(self.hardware.get_finger_tactile(), dtype=float)
        if raw.size == 0:
            return np.zeros((self._expected_module_count, 3), dtype=float)
        usable = raw if raw.size % 3 == 0 else raw[: raw.size - (raw.size % 3)]
        modules = usable.reshape(-1, 3)
        if modules.shape[0] < self._expected_module_count:
            padded = np.zeros((self._expected_module_count, 3), dtype=float)
            padded[: modules.shape[0]] = modules
            modules = padded
        else:
            modules = modules[: self._expected_module_count]
        if self._tactile_scale != 1.0:
            modules = modules * self._tactile_scale
        return modules

    def _gravity_scaling(self, rotation_ws: np.ndarray) -> np.ndarray:
        def axis_gain(axis: np.ndarray) -> float:
            axis_unit = axis / max(np.linalg.norm(axis), 1e-9)
            cos_theta = np.clip(abs(float(axis_unit.dot(self._gravity_world))), 0.0, 1.0)
            theta = float(np.arccos(cos_theta))
            gain = 1.0 - (2.0 / np.pi) * theta
            return float(np.clip(gain, 0.0, 1.0))

        return np.array([axis_gain(rotation_ws[:, 0]), axis_gain(rotation_ws[:, 1])], dtype=float)

    def _contact_strengths(self, tactile_modules: np.ndarray) -> Dict[str, float]:
        strengths: Dict[str, float] = {}
        for finger_cfg in self.config.finger_configs:
            total = 0.0
            for module in finger_cfg.modules:
                idx = module.tactile_index
                if idx < tactile_modules.shape[0]:
                    total += max(0.0, tactile_modules[idx][2])
            strengths[finger_cfg.name] = total
        return strengths

    def _check_contact_established(self, tactile_modules: np.ndarray) -> bool:
        strengths = self._contact_strengths(tactile_modules)
        return all(strengths.get(finger, 0.0) >= self._contact_threshold for finger in self._CONTACT_FINGERS)

    def _send_positions(self, q_cmd: np.ndarray) -> None:
        q_cmd = np.asarray(q_cmd, dtype=float)
        if q_cmd.size != self._joint_count:
            raise ValueError("Joint vector has unexpected length")
        self.hardware.set_joint_positions_radian(q_cmd.tolist())

    def _initialize_admittance_references(self, q: np.ndarray) -> None:
        for runtime in self._module_runtime.values():
            _, position, _ = self.kinematics.frame_pose_jacobian(runtime.config.frame, q)
            runtime.reference_position = position.copy()
            runtime.velocity[:] = 0.0

    def _pregrasp_step(self, joint_positions: np.ndarray, tactile_modules: np.ndarray) -> None:
        if self._pregrasp_targets is None:
            self._send_positions(joint_positions)
        else:
            error = self._pregrasp_targets - joint_positions
            delta = self._pregrasp_kp * error
            delta = np.clip(delta, -self._joint_step_limit, self._joint_step_limit)
            q_cmd = joint_positions + delta
            self._send_positions(q_cmd)

        if self._check_contact_established(tactile_modules):
            LOG.info("Pre-grasp contact established; switching to shear control.")
            q = self.kinematics.configuration_from_ordered(joint_positions)
            self._initialize_admittance_references(q)
            self._state = "shear"
            self.reset()

    # ------------------------------------------------------------------ runtime
    def step(self, dt: float) -> Dict[str, ModuleShearFeatures]:
        tactile_modules = self._read_tactile_modules()
        joint_positions = self._read_joint_positions()
        q = self.kinematics.configuration_from_ordered(joint_positions)

        if self._state == "pregrasp":
            self._pregrasp_step(joint_positions, tactile_modules)
            return {}

        module_features: Dict[str, ModuleShearFeatures] = {}
        joint_delta = np.zeros(self._joint_count, dtype=float)
        finger_normal_sums: Dict[str, float] = {}
        finger_entries: Dict[str, List[Dict[str, object]]] = {}

        for finger_cfg in self.config.finger_configs:
            entries: List[Dict[str, object]] = []

            for module_cfg in finger_cfg.modules:
                runtime = self._module_runtime[module_cfg.name]
                if module_cfg.tactile_index >= tactile_modules.shape[0]:
                    LOG.warning(
                        "Tactile packet missing index %d (module %s)",
                        module_cfg.tactile_index,
                        module_cfg.name,
                    )
                    runtime.pid.reset()
                    runtime.reference_position = None
                    runtime.velocity[:] = 0.0
                    continue

                rotation, position, jacobian = self.kinematics.frame_pose_jacobian(module_cfg.frame, q)
                gravity_scale = self._gravity_scaling(rotation)

                features = self._extractor.process(module_cfg.name, tactile_modules[module_cfg.tactile_index], dt)
                features.gravity_scale[:] = gravity_scale
                features.shear_rate[:2] *= gravity_scale
                features.shear_command[:] = 0.0

                runtime.last_features = features
                module_features[module_cfg.name] = features

                if features.normal_force < self.config.contact_force_threshold:
                    runtime.pid.reset()
                    runtime.reference_position = None
                    runtime.velocity[:] = 0.0
                    continue

                shear_error = -features.shear_rate[:2]
                shear_cmd_xy = runtime.pid.update(shear_error, dt)
                shear_cmd_xy *= float(module_cfg.weight)

                local_cmd = np.array([shear_cmd_xy[0], shear_cmd_xy[1], 0.0], dtype=float)
                features.shear_command[:] = local_cmd

                entries.append(
                    {
                        "features": features,
                        "rotation": rotation,
                        "jacobian": jacobian[:3, :],
                        "local_cmd": local_cmd,
                        "position": position,
                        "runtime": runtime,
                        "weight": float(module_cfg.weight),
                    }
                )

            finger_entries[finger_cfg.name] = entries

        for finger_cfg in self.config.finger_configs:
            entries = finger_entries[finger_cfg.name]
            if not entries:
                continue

            tangential_levels = np.array(
                [entry["features"].tangential_force + 1e-3 for entry in entries],
                dtype=float,
            )
            total_level = tangential_levels.sum()
            if total_level <= 0.0:
                weights = np.full(len(entries), 1.0 / len(entries))
            else:
                weights = tangential_levels / total_level

            slip_max = max(entry["features"].slip_ratio for entry in entries)
            slip_excess = max(0.0, slip_max - finger_cfg.friction_mu_min)

            for weight, entry in zip(weights, entries):
                features: ModuleShearFeatures = entry["features"]
                rotation = entry["rotation"]
                jacobian = entry["jacobian"]
                position = entry["position"]
                runtime: ModuleRuntime = entry["runtime"]

                scaled_local = entry["local_cmd"].copy()
                desired_normal = self._normal_setpoint - features.normal_force
                if slip_excess > 0.0:
                    allowable = max(0.0, finger_cfg.normal_force_max - features.normal_force)
                    desired_normal += min(finger_cfg.normal_force_gain * slip_excess, allowable)
                if features.normal_force > finger_cfg.normal_force_max:
                    desired_normal -= self.config.normal_force_backoff * (
                        features.normal_force - finger_cfg.normal_force_max
                    )
                desired_normal = np.clip(desired_normal, -finger_cfg.normal_force_max, finger_cfg.normal_force_max)
                scaled_local[2] += desired_normal
                features.shear_command[:] = scaled_local

                if runtime.reference_position is None:
                    runtime.reference_position = position.copy()
                    runtime.velocity[:] = 0.0

                force_world = rotation @ scaled_local
                force_measured = rotation @ features.force_filtered
                force_error = force_world - force_measured

                x = position
                x_ref = runtime.reference_position
                x_dot = runtime.velocity

                acc = (force_error - self._damping * x_dot - self._stiffness * (x - x_ref)) / self._mass
                x_dot_new = x_dot + acc * dt
                runtime.velocity = x_dot_new
                delta_x = x_dot_new * dt

                jac_linear = jacobian
                jjt = jac_linear @ jac_linear.T + (self._dls_lambda**2) * np.eye(3)
                try:
                    solve = np.linalg.solve(jjt, delta_x)
                except np.linalg.LinAlgError:
                    solve = np.linalg.lstsq(jjt, delta_x, rcond=None)[0]

                delta_q = jac_linear.T @ solve
                joint_delta += weight * delta_q

        joint_delta = np.clip(joint_delta, -self._joint_step_limit, self._joint_step_limit)
        q_cmd = joint_positions + joint_delta
        self._send_positions(q_cmd)
        return module_features

    def run(self, *, rate_hz: float = 150.0, warmup_time: float = 0.5) -> None:
        dt = 1.0 / rate_hz
        next_tick = time.perf_counter() + dt
        if self._pregrasp_targets is not None:
            self._state = "pregrasp"
        else:
            self._state = "shear"
        self.reset()
        if self._initial_targets is not None:
            self._send_positions(self._initial_targets)
            settle_deadline = time.perf_counter() + 3.0
            while time.perf_counter() < settle_deadline:
                current = self._read_joint_positions()
                if np.max(np.abs(current - self._initial_targets)) <= self._pregrasp_tolerance:
                    break
                time.sleep(0.05)
        time.sleep(warmup_time)

        LOG.info("Starting shear controller loop at %.1f Hz", rate_hz)
        try:
            while True:
                start = time.perf_counter()
                self.step(dt)
                elapsed = time.perf_counter() - start
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                next_tick += dt
        except KeyboardInterrupt:
            LOG.info("Shear controller interrupted by user.")


__all__ = ["ShearGraspController"]
