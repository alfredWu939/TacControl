from __future__ import annotations

import collections
from dataclasses import dataclass
from typing import Deque, Dict, Iterable

import numpy as np


@dataclass
class ModuleShearFeatures:
    """Container for filtered tactile data per module."""

    module: str
    force_raw: np.ndarray
    force_filtered: np.ndarray
    shear_rate: np.ndarray
    shear_command: np.ndarray
    normal_force: float
    tangential_force: float
    slip_ratio: float
    gravity_scale: np.ndarray


class _ModuleState:
    __slots__ = ("lowpass", "history", "shear_prev")

    def __init__(self, dim: int, history_len: int) -> None:
        self.lowpass = np.zeros(dim, dtype=float)
        self.history: Deque[np.ndarray] = collections.deque(maxlen=history_len)
        self.shear_prev = np.zeros(dim, dtype=float)


class ShearFeatureExtractor:
    """
    Shear feature extraction pipeline.

    Implements first-order low-pass filtering on Fx/Fy/Fz, a finite-difference
    shear rate, and the friction-coefficient ratio used for slip detection.
    """

    def __init__(
        self,
        modules: Iterable[str],
        lowpass_alpha: float,
        diff_window: int,
        derivative_alpha: float,
        epsilon: float = 1e-3,
    ) -> None:
        if diff_window < 1:
            raise ValueError("diff_window must be >= 1")
        self._alpha = float(np.clip(lowpass_alpha, 0.0, 1.0))
        self._derivative_alpha = float(np.clip(derivative_alpha, 0.0, 1.0))
        self._history_len = diff_window
        self._eps = float(epsilon)
        self._states: Dict[str, _ModuleState] = {
            name: _ModuleState(dim=3, history_len=diff_window) for name in modules
        }

    def process(self, module: str, force_vector: np.ndarray, dt: float) -> ModuleShearFeatures:
        """
        Process one tactile module sample.

        Args:
            module: module identifier configured in ModuleConfig.
            force_vector: raw [Fx, Fy, Fz] in the module local frame.
            dt: time delta in seconds since the previous controller iteration.
        """
        if module not in self._states:
            raise KeyError(f"Unknown tactile module '{module}'")

        state = self._states[module]
        vec = np.asarray(force_vector, dtype=float)
        if vec.shape != (3,):
            raise ValueError(f"force_vector must be length-3; got shape {vec.shape}")

        # low-pass filter (first order)
        state.lowpass = self._alpha * vec + (1.0 - self._alpha) * state.lowpass

        # maintain history for shear finite difference
        state.history.append(state.lowpass.copy())
        if len(state.history) >= 2:
            prev = state.history[0]
        else:
            prev = state.lowpass
        shear_delta = (state.lowpass - prev) / max(dt * (len(state.history) - 1), self._eps)

        # optional filtering on shear derivative to suppress spikes
        state.shear_prev = (
            self._derivative_alpha * shear_delta + (1.0 - self._derivative_alpha) * state.shear_prev
        )

        tangential_force = float(np.linalg.norm(state.lowpass[:2]))
        normal_force = float(state.lowpass[2])
        slip_ratio = tangential_force / max(abs(normal_force), self._eps)

        return ModuleShearFeatures(
            module=module,
            force_raw=vec,
            force_filtered=state.lowpass.copy(),
            shear_rate=state.shear_prev.copy(),
            shear_command=np.zeros(3, dtype=float),
            normal_force=normal_force,
            tangential_force=tangential_force,
            slip_ratio=slip_ratio,
            gravity_scale=np.ones(2, dtype=float),
        )


__all__ = ["ModuleShearFeatures", "ShearFeatureExtractor"]
