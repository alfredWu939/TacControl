"""
Configuration helpers and defaults for the DexH13 shear-based controller.

The real hand exposes 16 actuated joints and 11 tactile modules (each with
3-axis force sensing).  The defaults in this module encode a pragmatic mapping
between tactile modules, URDF frame names, and finger-level limits.  Users can
override these dictionaries at runtime (for instance via YAML) if their sensor
wiring differs.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Sequence

@dataclass(frozen=True)
class ModuleConfig:
    """Mapping from a tactile module to its URDF frame and owning finger."""

    name: str
    frame: str
    finger: str
    tactile_index: int
    weight: float = 1.0


@dataclass(frozen=True)
class FingerConfig:
    """Finger-level configuration for shear-force aggregation and limits."""

    name: str
    joint_names: Sequence[str]
    modules: Sequence[ModuleConfig]
    normal_force_max: float
    friction_mu_min: float
    normal_force_gain: float = 0.0


@dataclass(frozen=True)
class ControlGains:
    """PID gains applied per tangential axis on each tactile module."""

    kp: float
    ki: float
    kd: float
    integral_limit: float = 2.0


@dataclass(frozen=True)
class ShearControllerConfig:
    """
    Consolidated configuration for the shear controller.

    Contains default kinematic ordering, module mapping, controller gains,
    and electrical limits.
    """

    joint_order: Sequence[str]
    finger_configs: Sequence[FingerConfig]
    module_gains: Dict[str, ControlGains]
    torque_constants: Dict[str, float]
    current_limits: Dict[str, float]
    shear_rate_window: int = 4
    shear_lowpass_alpha: float = 0.35
    shear_derivative_alpha: float = 0.2
    shear_rate_dt: float = 0.01  # seconds between samples (overwritten at runtime)
    slip_ratio_threshold: float = 0.65
    slip_ratio_target: float = 0.5
    normal_force_backoff: float = 0.1
    contact_force_threshold: float = 0.1
    gravity_compensation: bool = False
    finger_contact_force_threshold: float = 0.15
    pregrasp_joint_targets: Sequence[float] = ()
    pregrasp_tolerance: float = 0.02
    pregrasp_kp: float = 0.8
    admittance_mass: Sequence[float] = (0.08, 0.08, 0.05)
    admittance_damping: Sequence[float] = (8.0, 8.0, 6.0)
    admittance_stiffness: Sequence[float] = (120.0, 120.0, 90.0)
    admittance_damping_lambda: float = 1.0e-3
    joint_step_limit: float = 0.02
    initial_joint_targets: Sequence[float] = ()
    tactile_count_to_newton: float = 0.1
    normal_force_setpoint: float = 1.0
    contact_loss_timeout: float = 0.3


# --------------------------------------------------------------------------- defaults

RIGHT_JOINT_ORDER: List[str] = [
    # index finger
    "right_index_joint_0",
    "right_index_joint_1",
    "right_index_joint_2",
    "right_index_joint_3",
    # middle finger
    "right_middle_joint_0",
    "right_middle_joint_1",
    "right_middle_joint_2",
    "right_middle_joint_3",
    # ring finger
    "right_ring_joint_0",
    "right_ring_joint_1",
    "right_ring_joint_2",
    "right_ring_joint_3",
    # thumb
    "right_thumb_joint_0",
    "right_thumb_joint_1",
    "right_thumb_joint_2",
    "right_thumb_joint_3",
]

LEFT_JOINT_ORDER: List[str] = [
    "left_index_joint_0",
    "left_index_joint_1",
    "left_index_joint_2",
    "left_index_joint_3",
    "left_middle_joint_0",
    "left_middle_joint_1",
    "left_middle_joint_2",
    "left_middle_joint_3",
    "left_ring_joint_0",
    "left_ring_joint_1",
    "left_ring_joint_2",
    "left_ring_joint_3",
    "left_thumb_joint_0",
    "left_thumb_joint_1",
    "left_thumb_joint_2",
    "left_thumb_joint_3",
]

RIGHT_MODULES: List[ModuleConfig] = [
    ModuleConfig("right_index_pad_proximal", "right_index_tactile_link_0", "index", tactile_index=0, weight=1.0),
    ModuleConfig("right_index_pad_middle", "right_index_tactile_link_1", "index", tactile_index=1, weight=1.0),
    ModuleConfig("right_index_pad_distal", "right_index_tactile_link_2", "index", tactile_index=2, weight=1.0),
    ModuleConfig("right_middle_pad_proximal", "right_middle_tactile_link_0", "middle", tactile_index=3, weight=1.0),
    ModuleConfig("right_middle_pad_middle", "right_middle_tactile_link_1", "middle", tactile_index=4, weight=1.0),
    ModuleConfig("right_middle_pad_distal", "right_middle_tactile_link_2", "middle", tactile_index=5, weight=1.0),
    ModuleConfig("right_ring_pad_proximal", "right_ring_tactile_link_0", "ring", tactile_index=6, weight=1.0),
    ModuleConfig("right_ring_pad_middle", "right_ring_tactile_link_1", "ring", tactile_index=7, weight=1.0),
    ModuleConfig("right_ring_pad_distal", "right_ring_tactile_link_2", "ring", tactile_index=8, weight=1.0),
    ModuleConfig("right_thumb_pad_proximal", "right_thumb_tactile_link_0", "thumb", tactile_index=9, weight=1.0),
    ModuleConfig("right_thumb_pad_distal", "right_thumb_tactile_link_1", "thumb", tactile_index=10, weight=1.0),
]

LEFT_MODULES: List[ModuleConfig] = [
    ModuleConfig("left_index_pad_proximal", "left_index_tactile_link_0", "index", tactile_index=0, weight=1.0),
    ModuleConfig("left_index_pad_middle", "left_index_tactile_link_1", "index", tactile_index=1, weight=1.0),
    ModuleConfig("left_index_pad_distal", "left_index_tactile_link_2", "index", tactile_index=2, weight=1.0),
    ModuleConfig("left_middle_pad_proximal", "left_middle_tactile_link_0", "middle", tactile_index=3, weight=1.0),
    ModuleConfig("left_middle_pad_middle", "left_middle_tactile_link_1", "middle", tactile_index=4, weight=1.0),
    ModuleConfig("left_middle_pad_distal", "left_middle_tactile_link_2", "middle", tactile_index=5, weight=1.0),
    ModuleConfig("left_ring_pad_proximal", "left_ring_tactile_link_0", "ring", tactile_index=6, weight=1.0),
    ModuleConfig("left_ring_pad_middle", "left_ring_tactile_link_1", "ring", tactile_index=7, weight=1.0),
    ModuleConfig("left_ring_pad_distal", "left_ring_tactile_link_2", "ring", tactile_index=8, weight=1.0),
    ModuleConfig("left_thumb_pad_proximal", "left_thumb_tactile_link_0", "thumb", tactile_index=9, weight=1.0),
    ModuleConfig("left_thumb_pad_distal", "left_thumb_tactile_link_1", "thumb", tactile_index=10, weight=1.0),
]

def _build_finger_configs(joint_order: Sequence[str], modules: Sequence[ModuleConfig]) -> List[FingerConfig]:
    return [
        FingerConfig(
            name="index",
            joint_names=joint_order[0:4],
            modules=[m for m in modules if m.finger == "index"],
            normal_force_max=7.5,
            friction_mu_min=0.35,
            normal_force_gain=0.6,
        ),
        FingerConfig(
            name="middle",
            joint_names=joint_order[4:8],
            modules=[m for m in modules if m.finger == "middle"],
            normal_force_max=7.5,
            friction_mu_min=0.35,
            normal_force_gain=0.6,
        ),
        FingerConfig(
            name="ring",
            joint_names=joint_order[8:12],
            modules=[m for m in modules if m.finger == "ring"],
            normal_force_max=6.0,
            friction_mu_min=0.35,
            normal_force_gain=0.6,
        ),
        FingerConfig(
            name="thumb",
            joint_names=joint_order[12:16],
            modules=[m for m in modules if m.finger == "thumb"],
            normal_force_max=8.5,
            friction_mu_min=0.4,
            normal_force_gain=0.7,
        ),
    ]

# Tangential PID gains per module; tuned conservatively for paper-cup handling.
def _default_module_gains(modules: Sequence[ModuleConfig]) -> Dict[str, ControlGains]:
    return {
        module.name: ControlGains(kp=0.85, ki=1.2, kd=0.02, integral_limit=1.5)
        for module in modules
    }


def _torque_map(default: float, joint_order: Sequence[str]) -> Dict[str, float]:
    return {name: default for name in joint_order}


DEFAULT_PREGRASP_JOINT_TARGETS: List[float] = [0, math.pi / 180 * 50.0, math.pi / 180 * 50.0, 0,
                   0, math.pi / 180 * 50.0, math.pi / 180 * 50.0, 0, 
                   0, math.pi / 180 * 50.0, math.pi / 180 * 50.0, 0, 
                   -math.pi / 180 * 15.0, math.pi / 180 * 85.0, math.pi / 180 * 50.0, math.pi / 180 * 40.0]

DEFAULT_INITIAL_JOINT_TARGETS: List[float] = [
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, math.pi / 180 * 85.0, 0, 0,
]


def build_default_config() -> ShearControllerConfig:
    """
    Construct the standard configuration used by the shear controller.

    Returns:
        ShearControllerConfig: aggregated defaults ready for consumption.
    """

    hand = os.environ.get("DEXH13_HAND", "right").lower()
    if "left" in hand:
        joint_order = LEFT_JOINT_ORDER
        modules = LEFT_MODULES
    else:
        joint_order = RIGHT_JOINT_ORDER
        modules = RIGHT_MODULES

    finger_configs = _build_finger_configs(joint_order, modules)
    module_gains = _default_module_gains(modules)
    torque_constants = _torque_map(0.12, joint_order)
    current_limits = {name: 3.0 for name in joint_order}

    return ShearControllerConfig(
        joint_order=tuple(joint_order),
        finger_configs=tuple(finger_configs),
        module_gains=module_gains,
        torque_constants=torque_constants,
        current_limits=current_limits,
        pregrasp_joint_targets=tuple(DEFAULT_PREGRASP_JOINT_TARGETS),
        initial_joint_targets=tuple(DEFAULT_INITIAL_JOINT_TARGETS),
    )


__all__ = [
    "ModuleConfig",
    "FingerConfig",
    "ControlGains",
    "ShearControllerConfig",
    "RIGHT_JOINT_ORDER",
    "RIGHT_MODULES",
    "LEFT_JOINT_ORDER",
    "LEFT_MODULES",
    "DEFAULT_PREGRASP_JOINT_TARGETS",
    "DEFAULT_INITIAL_JOINT_TARGETS",
    "build_default_config",
]
