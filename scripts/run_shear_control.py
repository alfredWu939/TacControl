#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
import sys
import threading
from pathlib import Path


def _ensure_src_on_path() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    src_dir = repo_root / "src"
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))


_ensure_src_on_path()

import os

import numpy as np
from pxdex.dh13 import ControlMode

from tacctrl.config import ShearControllerConfig, build_default_config
from tacctrl.control.shear import ShearGraspController
from tacctrl.hardware import RealPxDH13
from tacctrl.kinematics.dexh13_hand import DexH13HandModel

def _default_urdf(hand: str) -> Path:
    suffix = "left" if hand == "left" else "right"
    return Path(
        f"robot_description/xarm_paxini/hands/paxini_hand/dexh13_hand_{suffix}_description.urdf"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the shear-based grasp controller on a Paxini DexH13 hand.",
    )
    parser.add_argument("--hand", choices=["right", "left"], default="left", help="DexH13 hand (right/left).")
    parser.add_argument("--hand-tty", default="/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_BKDVb114J19-if00-port0", help="DexH13 serial port.") # usb-Prolific_Technology_Inc._USB-Serial_Controller_ATDHb114J19-if00-port0
    parser.add_argument("--camera-port", default="/dev/video0", help="Camera port required by the SDK handshake.")
    parser.add_argument("--rate-hz", type=float, default=150.0, help="Controller update frequency.")
    parser.add_argument("--driver-rate-hz", type=float, default=200.0, help="Background driver refresh rate.")
    parser.add_argument("--warmup", type=float, default=0.5, help="Seconds to wait before closing the loop.")
    parser.add_argument("--skip-home", action="store_true", help="Skip initMotorPosition on connect.")
    parser.add_argument("--calibrate-tactile", action="store_true", help="Run tactile calibration after connection.")
    parser.add_argument("--tactile-dim", type=int, default=33, help="Number of tactile channels to read (3*N modules).")
    parser.add_argument("--log-level", default="INFO", help="Python logging level (DEBUG, INFO, ...).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    os.environ["DEXH13_HAND"] = args.hand.lower()

    config: ShearControllerConfig = build_default_config()
    urdf_path = _default_urdf(args.hand).expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    kinematics = DexH13HandModel(str(urdf_path), config.joint_order)

    driver = RealPxDH13(
        handy_port_num=args.hand_tty,
        camera_port_num=args.camera_port,
        units="rad",
        rate_hz=max(1, int(args.driver_rate_hz)),
        control_mode=ControlMode.POSITION_CONTROL_MODE,
        tactile_dim=max(3, int(args.tactile_dim)),
        init_home=not args.skip_home,
        auto_calibrate=args.calibrate_tactile,
    )
    driver.start()

    controller = ShearGraspController(
        hardware=driver,
        kinematics=kinematics,
        config=config,
    )

    stop_event = threading.Event()

    def tactile_monitor() -> None:
        while not stop_event.is_set():
            data = driver.get_finger_tactile()
            module_target = 11
            modules = np.zeros((module_target, 3), dtype=float)
            usable = min(data.size // 3, module_target)
            if usable > 0:
                modules[:usable] = (
                    np.asarray(data[: usable * 3], dtype=float).reshape(-1, 3)
                    * config.tactile_count_to_newton
                )
            formatted = ", ".join(
                f"{idx}:({vals[0]:.3f},{vals[1]:.3f},{vals[2]:.3f})"
                for idx, vals in enumerate(modules)
            )
            print(f"[tactile len={data.size}] {formatted}")
            stop_event.wait(0.5)

    monitor_thread = threading.Thread(target=tactile_monitor, name="tactile-monitor", daemon=True)
    monitor_thread.start()

    try:
        controller.run(rate_hz=args.rate_hz, warmup_time=args.warmup)
    finally:
        stop_event.set()
        monitor_thread.join(timeout=1.0)
        driver.stop(disconnect=True)


if __name__ == "__main__":
    main()
