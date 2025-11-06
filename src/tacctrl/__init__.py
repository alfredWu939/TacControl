"""
Shear-based grasp control toolkit for the Paxini DexH13 tactile hand.

Modules within `tacctrl` expose:
  - hardware access helpers for the DexH13 SDK
  - Pinocchio-based kinematic utilities for the hand
  - tactile preprocessing filters and features
  - shear-force closed-loop controllers

The package is organised to keep control logic independent from the CLI
entry points so that both simulation and on-robot scripts can reuse it.
"""

__all__ = [
    "control",
    "hardware",
    "kinematics",
    "tactile",
]
