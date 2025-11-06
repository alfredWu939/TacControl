import argparse
import contextlib
import multiprocessing
import time
from typing import List, Optional, Sequence

import numpy as np
from pxdex.dh13 import ControlMode, Dex13MotorSpeed, DexH13Control, FingerAngle


class RealPxDH13:
    """Background-process controller for Paxini DexH13 hand (16 DOF)."""

    ALL_FINGERS = list(range(16))
    DEFAULT_SPEED_JOINT_MAP = (
        0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 15
    )

    def __init__(
        self,
        handy_port_num: str = "/dev/ttyUSB0",
        camera_port_num: str = "/dev/video0",
        units: str = "rad",
        rate_hz: int = 83,
        control_mode: ControlMode = ControlMode.POSITION_CONTROL_MODE,
        tactile_dim: int = 33,
        init_home: bool = True,
        auto_calibrate: bool = False,
        max_speed_rpm: int = 20000,
        speed_joint_map: Optional[Sequence[int]] = None,
    ) -> None:
        units = units.lower()
        if units not in ("rad", "deg"):
            raise ValueError("units must be 'rad' or 'deg'")

        self.handy_port_num = handy_port_num
        self.camera_port_num = camera_port_num
        self.units = units
        self.rate_hz = max(1, int(rate_hz))
        self.control_mode = control_mode
        self._tactile_capacity = max(1, int(tactile_dim))
        self._init_home = bool(init_home)
        self._auto_calibrate = bool(auto_calibrate)
        self._max_speed_rpm = max(0, int(max_speed_rpm))

        if speed_joint_map is None:
            speed_joint_map = RealPxDH13.DEFAULT_SPEED_JOINT_MAP
        mapped = tuple(int(idx) for idx in speed_joint_map)
        if len(mapped) != len(RealPxDH13.DEFAULT_SPEED_JOINT_MAP):
            raise ValueError("speed_joint_map must contain 13 joint indices")
        if any(idx < 0 or idx >= 16 for idx in mapped):
            raise ValueError("speed_joint_map indices must be between 0 and 15")
        self._speed_joint_map = mapped

        self._lock = multiprocessing.Lock()
        self._enable = multiprocessing.Value("b", False)
        # separate enables for current/speed control
        self._enable_current = multiprocessing.Value("b", False)
        self._enable_speed = multiprocessing.Value("b", False)
        self._stop = multiprocessing.Event()
        self._cmd = multiprocessing.Array("d", [0.0] * 16)
        # motor target current and speed commands (length-16 arrays)
        self._cmd_current = multiprocessing.Array("d", [0.0] * 16)
        self._cmd_speed = multiprocessing.Array("d", [0.0] * 16)
        self._feedback = multiprocessing.Array("d", [0.0] * 16)
        # also keep angle feedback (when available) and tactile sensors
        self._feedback_angle = multiprocessing.Array("d", [0.0] * 16)
        self._tactile = multiprocessing.Array("d", [0.0] * self._tactile_capacity)
        self._tactile_len = multiprocessing.Value("i", 0)
        self._proc: Optional[multiprocessing.Process] = None

    # ------------------------------------------------------------------ helpers
    def _ensure_process(self) -> None:
        if self._proc and self._proc.is_alive():
            return

        self._stop.clear()
        self._enable.value = False
        self._enable_current.value = False
        self._enable_speed.value = False
        with self._lock:
            for i in range(16):
                self._cmd[i] = 0.0
                self._cmd_current[i] = 0.0
                self._cmd_speed[i] = 0.0
                self._feedback[i] = 0.0
                self._feedback_angle[i] = 0.0
            for i in range(self._tactile_capacity):
                self._tactile[i] = 0.0
            self._tactile_len.value = 0

        self._proc = multiprocessing.Process(
            target=self._worker,
            args=(
                self.handy_port_num,
                self.camera_port_num,
                self._cmd,
                self._cmd_current,
                self._cmd_speed,
                self._feedback,
                self._feedback_angle,
                self._tactile,
                self._tactile_len,
                self._enable,
                self._enable_current,
                self._enable_speed,
                self._stop,
                self._lock,
                self.units,
                self.rate_hz,
                self.control_mode.value,
                self._tactile_capacity,
                int(self._init_home),
                int(self._auto_calibrate),
                self._speed_joint_map,
                self._max_speed_rpm,
            ),
            daemon=True,
        )
        self._proc.start()
        print(f"[PxDH13] background started ({self.rate_hz} Hz, units={self.units}).")

    # ------------------------------------------------------------------ API
    def start(self) -> None:
        self._ensure_process()

    def set_finger_cmd(
        self,
        finger_ids: List[int],
        positions: List[float],
        kps: Optional[List[float]] = None,
        kis: Optional[List[float]] = None,
        kds: Optional[List[float]] = None,
        tor_maxs: Optional[List[float]] = None,
    ) -> None:
        del finger_ids, kps, kis, kds, tor_maxs  # unused but kept for signature compatibility

        if positions is None or len(positions) != 16:
            raise ValueError("[PxDH13] positions must be a length-16 list.")

        self._ensure_process()
        
        # clamp commands to safe ranges (abd joints: [-20, 20], others: [0, 90] in degrees)
        if self.units == "deg":
            ab_lower, ab_upper = -20.0, 20.0
            flex_lower, flex_upper = 0.0, 90.0
        else:
            ab_lower, ab_upper = np.deg2rad(-20.0), np.deg2rad(20.0)
            flex_lower, flex_upper = 0.0, np.deg2rad(90.0)
        clamped_positions = [float(v) for v in positions]
        for idx in (0, 4, 8):
            clamped_positions[idx] = float(np.clip(clamped_positions[idx], ab_lower, ab_upper))
        for idx in range(16):
            if idx in (0, 4, 8):
                continue
            clamped_positions[idx] = float(np.clip(clamped_positions[idx], flex_lower, flex_upper))

        # clamp commands to safe ranges (abd joints: [-20, 20], others: [0, 90] in degrees)
        if self.units == "deg":
            ab_lower, ab_upper = -20.0, 20.0
            flex_lower, flex_upper = 0.0, 90.0
        else:
            ab_lower, ab_upper = np.deg2rad(-20.0), np.deg2rad(20.0)
            flex_lower, flex_upper = 0.0, np.deg2rad(90.0)

        clamped_positions = [float(v) for v in positions]
        for idx in (0, 4, 8):
            clamped_positions[idx] = float(np.clip(clamped_positions[idx], ab_lower, ab_upper))
        for idx in range(16):
            if idx in (0, 4, 8):
                continue
            clamped_positions[idx] = float(np.clip(clamped_positions[idx], flex_lower, flex_upper))

        with self._lock:
            for i, v in enumerate(clamped_positions):
                self._cmd[i] = float(v)
            self._enable.value = True

    def set_joint_positions_radian(self, joint_values: List[float]) -> None:
        if joint_values is None or len(joint_values) != 16:
            raise ValueError("[PxDH13] joint_positions must be a length-16 list.")
        self.set_finger_cmd(self.ALL_FINGERS, list(joint_values))

    def set_motor_target_current(
        self,
        finger_ids: List[int],
        currents: List[float],
    ) -> None:
        """Set motor target currents for all 16 joints (background process).

        The API expects a length-16 list. Values are sent to the background worker
        which calls the controller's current-target API.
        """
        del finger_ids  # kept for signature compatibility
        if currents is None or len(currents) != 16:
            raise ValueError("[PxDH13] currents must be a length-16 list.")

        self._ensure_process()
        with self._lock:
            for i, v in enumerate(currents):
                self._cmd_current[i] = float(v)
            self._enable_current.value = True

    def set_motor_target_speed(
        self,
        finger_ids: List[int],
        speeds: List[float],
    ) -> None:
        """Set motor target speeds for all 16 joints (background process)."""
        del finger_ids
        if speeds is None or len(speeds) != 16:
            raise ValueError("[PxDH13] speeds must be a length-16 list.")

        self._ensure_process()
        with self._lock:
            for i, v in enumerate(speeds):
                self._cmd_speed[i] = float(v)
            self._enable_speed.value = True

    def get_finger_qpos(self) -> np.ndarray:
        if not (self._proc and self._proc.is_alive()):
            return np.zeros(16, dtype=float)
        with self._lock:
            return np.asarray(self._feedback[:], dtype=float)

    def get_joint_positions_angle(self) -> np.ndarray:
        """Return the latest joint positions in angle units (if available)."""
        if not (self._proc and self._proc.is_alive()):
            return np.zeros(16, dtype=float)
        with self._lock:
            return np.asarray(self._feedback_angle[:], dtype=float)

    def get_joint_positions_radian(self) -> np.ndarray:
        """Return the latest joint positions in radians."""
        return self.get_finger_qpos()

    def get_finger_tactile(self) -> np.ndarray:
        """Return the latest tactile sensor readings (per-joint / per-finger)."""
        if not (self._proc and self._proc.is_alive()):
            return np.zeros(self._tactile_capacity, dtype=float)
        with self._lock:
            length = max(0, min(self._tactile_len.value, self._tactile_capacity))
            return np.asarray(self._tactile[:length], dtype=float)

    def stop(self, disconnect: bool = False) -> None:
        if not self._proc:
            return
        self._stop.set()
        self._proc.join(timeout=2.0)
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=1.0)
        self._proc = None
        print("[PxDH13] background stopped.")

        if disconnect:
            try:
                ctl = DexH13Control()
                if ctl.isMotorEnabled():
                    ctl.disableMotor()
                ctl.disconnectHandy()
            except Exception:
                pass

    # ------------------------------------------------------------------ worker
    @staticmethod
    def _worker(
        handy_port_num: str,
        camera_port_num: str,
        sh_cmd,
        sh_cmd_current,
        sh_cmd_speed,
        sh_feedback,
        sh_feedback_angle,
        sh_tactile,
        sh_tactile_len,
        sh_enable,
        sh_enable_current,
        sh_enable_speed,
        sh_stop,
        sh_lock,
        units: str,
        rate_hz: int,
        control_mode_value: int,
        tactile_capacity: int,
        init_home_flag: int,
        auto_calibrate_flag: int,
        speed_joint_map,
        max_speed_rpm: int,
    ) -> None:
        dt = 1.0 / max(1, rate_hz)
        ctl = DexH13Control()
        control_mode = ControlMode(control_mode_value)
        init_home = bool(init_home_flag)
        auto_calibrate = bool(auto_calibrate_flag)
        joint_map = tuple(int(idx) for idx in speed_joint_map)
        rpm_limit = max(0, int(max_speed_rpm))

        def connect_and_prepare() -> bool:
            try:
                ctl.activeHandy(handy_port_num, camera_port_num)
                if not ctl.isConnectHandy():
                    return False
                if init_home:
                    try:
                        ctl.initMotorPosition()
                    except Exception:
                        pass
                if not ctl.setMotorControlMode(control_mode):
                    return False
                if control_mode == ControlMode.SPEED_CONTROL_MODE and rpm_limit > 0:
                    with contextlib.suppress(Exception):
                        ctl.setMotorMaxSpeed(rpm_limit)
                if not ctl.isMotorEnabled():
                    if not ctl.enableMotor():
                        return False
                if auto_calibrate:
                    with contextlib.suppress(Exception):
                        ctl.calibrateSensor()
                return True
            except Exception as exc:
                print(f"[PxDH13-worker] init error: {exc}")
                return False

        def make_angles(values: List[float]) -> List[FingerAngle]:
            thumb = RealPxDH13._make_fa(values[12], values[13], values[14], values[15])
            index = RealPxDH13._make_fa(values[0], values[1], values[2], values[3])
            middle = RealPxDH13._make_fa(values[4], values[5], values[6], values[7])
            ring = RealPxDH13._make_fa(values[8], values[9], values[10], values[11])
            return [thumb, index, middle, ring]

        def make_speeds(values: List[float]):
            if Dex13MotorSpeed is None:
                return None
            joint_values = [values[idx] for idx in joint_map]
            rpm = np.asarray(joint_values, dtype=float) * (60.0 / (2.0 * np.pi))
            limit = rpm_limit if rpm_limit > 0 else 24000.0
            rpm = np.clip(rpm, -limit, limit)
            speeds = Dex13MotorSpeed()
            assigned = False
            for motor_idx, rpm_val in enumerate(rpm, start=1):
                attr = f"motor{motor_idx}_speed"
                if hasattr(speeds, attr):
                    try:
                        setattr(speeds, attr, int(round(rpm_val)))
                        assigned = True
                    except AttributeError:
                        continue
            if assigned and not hasattr(make_speeds, "_logged_first"):
                print("[PxDH13-worker] speed command sample (rpm):", [int(round(val)) for val in rpm])
                setattr(make_speeds, "_logged_first", True)
            return speeds if assigned else None

        ready = False

        while not sh_stop.is_set():
            if not ready:
                ready = connect_and_prepare()
                if not ready:
                    time.sleep(0.5)
                    continue
            # if no command type is enabled, just sleep
            if not (sh_enable.value or sh_enable_current.value or sh_enable_speed.value):
                time.sleep(dt)
                continue

            # read all command arrays under lock
            with sh_lock:
                current_cmd = [sh_cmd[i] for i in range(16)]
                current_target = [sh_cmd_current[i] for i in range(16)]
                speed_target = [sh_cmd_speed[i] for i in range(16)]

            # send position command if enabled
            try:
                if sh_enable.value:
                    if units == "deg":
                        angles = make_angles(current_cmd)
                        ctl.setJointPositionsAngle(angles)
                    else:
                        ctl.setJointPositionsRadian(current_cmd)
            except Exception as exc:
                print(f"[PxDH13-worker] position command error: {exc}")
                ready = False
                continue

            # send current command if enabled
            try:
                if sh_enable_current.value:
                    # controller API expected a sequence of 16 values
                    ctl.setMotorTargetCurrent(current_target)
            except Exception as exc:
                print(f"[PxDH13-worker] current command error: {exc}")
                ready = False
                continue

            # send speed command if enabled
            try:
                if sh_enable_speed.value:
                    speed_msg = make_speeds(speed_target)
                    if speed_msg is not None:
                        try:
                            ctl.setMotorTargetSpeed(speed_msg)
                        except Exception:
                            speed_msg = None
                    if speed_msg is None:
                        # fall back to position integration if SDK lacks speed structures or call failed
                        current_q = ctl.getJointPositionsRadian()
                        if len(current_q) >= 16:
                            next_q = [float(current_q[i] + speed_target[i] * dt) for i in range(16)]
                            ctl.setJointPositionsRadian(next_q)
            except Exception as exc:
                print(f"[PxDH13-worker] speed command error: {exc}")
                ready = False
                continue

            # gather feedback (radian positions)
            try:
                feedback = ctl.getJointPositionsRadian()
                with sh_lock:
                    for i in range(min(16, len(feedback))):
                        sh_feedback[i] = float(feedback[i])
            except Exception:
                pass

            # gather angle feedback if available
            try:
                angles_fb = ctl.getJointPositionsAngle()
                with sh_lock:
                    for i in range(min(16, len(angles_fb))):
                        sh_feedback_angle[i] = float(angles_fb[i])
            except Exception:
                pass

            # gather tactile sensor feedback if available
            try:
                tactile = ctl.getFingerTactile()
                flat_buffer: List[float] = []
                for entry in tactile:
                    try:
                        flat_buffer.extend((float(entry.x), float(entry.y), float(entry.z)))
                    except AttributeError:
                        flat_buffer.append(float(entry))
                with sh_lock:
                    length = min(tactile_capacity, len(flat_buffer))
                    for i in range(length):
                        sh_tactile[i] = flat_buffer[i]
                    for i in range(length, tactile_capacity):
                        sh_tactile[i] = 0.0
                    sh_tactile_len.value = length
            except Exception:
                pass

            time.sleep(dt)

        try:
            if ctl.isMotorEnabled():
                ctl.disableMotor()
            ctl.disconnectHandy()
        except Exception:
            pass

    @staticmethod
    def _make_fa(j1, j2, j3, j4) -> FingerAngle:
        fa = FingerAngle()
        fa.joint1, fa.joint2, fa.joint3, fa.joint4 = j1, j2, j3, j4
        return fa


# --------------------------------------------------------------------------- demo
def main():
    parser = argparse.ArgumentParser(description="Paxini DH13：输入 16 维角度直接驱动手指（后台进程）。")
    parser.add_argument("--hand-tty", default="/dev/ttyUSB0", help="手部串口 (例如 /dev/ttyUSB0)")
    parser.add_argument("--camera-port", default="/dev/video0", help="摄像头设备 (例如 /dev/video0)")
    parser.add_argument("--units", choices=["rad", "deg"], default="rad", help="输入角度单位")
    parser.add_argument(
        "--angles",
        type=float,
        nargs=16,
        metavar=("a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "a12", "a13", "a14", "a15"),
        help="16 个关节角度，顺序：index→middle→ring→thumb (abd, flex1, flex2, flex3)",
    )
    parser.add_argument("--hold-sec", type=float, default=3.0, help="保持目标姿态秒数（非循环模式）")
    parser.add_argument("--loop", action="store_true", help="循环发送角度直到 Ctrl+C 终止")
    parser.add_argument("--rate-hz", type=int, default=83, help="命令刷新频率")
    args = parser.parse_args()

    if args.angles is None:
        print("[Info] 未提供 --angles，使用默认示例动作。")
        if args.units == "deg":
            angles = [
                0.0, 50.0, 50.0, 50.0,
                0.0, 50.0, 50.0, 50.0,
                0.0, 50.0, 50.0, 50.0,
                10.0, 50.0, 50.0, 50.0,
            ]
        else:
            angles = [
                0.0, 0.45, 0.52, 0.52,
                0.0, 0.45, 0.52, 0.52,
                0.0, 0.45, 0.52, 0.52,
                0.17, 0.35, 0.35, 0.35,
            ]
    else:
        angles = list(args.angles)

    hand = RealPxDH13(
        handy_port_num=args.hand_tty,
        camera_port_num=args.camera_port,
        units=args.units,
        rate_hz=args.rate_hz,
    )

    try:
        hand.set_finger_cmd(hand.ALL_FINGERS, angles)
        if args.loop:
            print("[Info] 循环保持姿态，Ctrl+C 退出。")
            while True:
                hand.set_finger_cmd(hand.ALL_FINGERS, angles)
                time.sleep(0.1)
        else:
            print(f"[Info] 保持 {args.hold_sec} 秒后归零。")
            time.sleep(max(0.0, args.hold_sec))
    except KeyboardInterrupt:
        print("[Info] 收到 Ctrl+C，准备退出。")
    finally:
        print("[Info] 指令归零。")
        hand.set_finger_cmd(hand.ALL_FINGERS, [0.0] * 16)
        hand.stop(disconnect=True)


if __name__ == "__main__":
    main()
