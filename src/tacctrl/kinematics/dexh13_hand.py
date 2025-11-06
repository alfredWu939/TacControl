from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping, Sequence

import numpy as np

try:
    import pinocchio as pin
    from pinocchio.robot_wrapper import RobotWrapper
except ImportError as exc:  # pragma: no cover - executed only without Pinocchio
    raise ImportError(
        "Pinocchio is required for tacctrl's kinematics module. "
        "Install it with `pip install pin` before running the controller."
    ) from exc


class DexH13HandModel:
    """
    Pinocchio-based kinematics helper for the DexH13 hand.

    Loads the hand-only URDF and exposes utilities to:
      * convert between joint dictionaries and Pinocchio configuration vectors
      * compute contact frame Jacobians using LOCAL_WORLD_ALIGNED semantics
      * query the world orientation of tactile frames
    """

    def __init__(self, urdf_path: str, joint_order: Sequence[str]) -> None:
        self.urdf_path = str(Path(urdf_path).expanduser().resolve())
        # Ensure Pinocchio can find meshes referenced with relative paths in the
        # URDF (e.g. "./meshes/..."). Pass the URDF directory as a package
        # directory so buildModelsFromUrdf can resolve those mesh filenames.
        urdf_dir = str(Path(self.urdf_path).parent)
        try:
            self.robot = RobotWrapper.BuildFromURDF(self.urdf_path, package_dirs=[urdf_dir])
        except TypeError:
            # Older Pinocchio versions accepted package_dirs as a positional
            # argument. Fall back to the positional form for compatibility.
            self.robot = RobotWrapper.BuildFromURDF(self.urdf_path, [urdf_dir])
        self.model = self.robot.model
        self.data = self.robot.data

        self.joint_order = tuple(joint_order)
        if len(self.joint_order) == 0:
            raise ValueError("joint_order must contain at least one entry")

        # map joint names to configuration indices within q
        self._q_index: Dict[str, int] = {}
        self._v_index: Dict[str, int] = {}
        for name in self.joint_order:
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                raise ValueError(f"Joint '{name}' not present in URDF '{self.urdf_path}'")
            idx_q = self.model.idx_qs[joint_id]
            idx_v = self.model.idx_vs[joint_id]
            self._q_index[name] = int(idx_q)
            self._v_index[name] = int(idx_v)

        self._frame_ids: Dict[str, int] = {}

    # ------------------------------------------------------------------ joint utilities
    def zero_configuration(self) -> np.ndarray:
        """Return the neutral configuration vector of the Pinocchio model."""
        return pin.neutral(self.model)

    def configuration_from_joints(self, joint_values: Mapping[str, float]) -> np.ndarray:
        """
        Map a joint dictionary to the Pinocchio configuration vector.

        Unspecified joints default to their neutral value.
        """
        q = pin.neutral(self.model)
        for name, value in joint_values.items():
            if name not in self._q_index:
                continue
            q[self._q_index[name]] = float(value)
        return q

    def configuration_from_ordered(self, values: Sequence[float]) -> np.ndarray:
        """
        Map an ordered 16-vector (matching joint_order) to configuration space.
        """
        if len(values) != len(self.joint_order):
            raise ValueError(f"Expected {len(self.joint_order)} joint values, got {len(values)}")
        q = pin.neutral(self.model)
        for idx, name in enumerate(self.joint_order):
            q[self._q_index[name]] = float(values[idx])
        return q

    def velocity_indices(self) -> Dict[str, int]:
        """Return a mapping from joint names to velocity indices."""
        return dict(self._v_index)

    # ------------------------------------------------------------------ frames
    def _frame_id(self, frame: str) -> int:
        if frame not in self._frame_ids:
            self._frame_ids[frame] = self.model.getFrameId(frame)
            if self._frame_ids[frame] == len(self.model.frames):
                raise ValueError(f"Frame '{frame}' not found in URDF '{self.urdf_path}'")
        return self._frame_ids[frame]

    def frame_rotation(self, frame: str, q: np.ndarray) -> np.ndarray:
        """
        Compute the world rotation matrix of the specified frame.
        """
        rotation, _, _ = self.frame_pose_jacobian(frame, q)
        return rotation

    def frame_linear_jacobian(self, frame: str, q: np.ndarray) -> np.ndarray:
        """
        Compute the 3xN linear Jacobian for the given frame (LOCAL_WORLD_ALIGNED).
        """
        _, _, jacobian = self.frame_pose_jacobian(frame, q)
        return jacobian[:3, :].copy()

    def frame_pose_jacobian(self, frame: str, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute rotation, translation, and full 6xN Jacobian for the given frame.
        """
        frame_id = self._frame_id(frame)
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, frame_id)
        jacobian = pin.getFrameJacobian(
            self.model,
            self.data,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        rotation = self.data.oMf[frame_id].rotation.copy()
        translation = self.data.oMf[frame_id].translation.copy()
        return rotation, translation, jacobian.copy()

    def frame_kinematics(self, frame: str, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        rotation, _, jacobian = self.frame_pose_jacobian(frame, q)
        return rotation, jacobian[:3, :].copy()


__all__ = ["DexH13HandModel"]
