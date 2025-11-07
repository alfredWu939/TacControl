# TacCtrl – DexH13 Shear/Admittance Control

TacCtrl implements a shear-force based grasp stabiliser for the Paxini DexH13 tactile hand.  It pairs the vendor’s background SDK process with an admittance controller that converts tactile shear/normal feedback into joint-space position targets via the hand URDF (Pinocchio).  The codebase is designed to run on real hardware; there is no simulator mode at this time.

## Repository Layout

```
guidance.md          # design notes (explains control maths)
robot_description/   # URDF + meshes for DexH13 (recently updated)
scripts/             # CLI entry points (run_shear_control.py, diagnostics, …)
src/tacctrl/         # python package (config, controller, tactile filters, hardware wrapper)
```

Key components:

* `scripts/run_shear_control.py` – CLI that launches the DexH13 SDK background thread, loads the URDF, and runs the admittance shear controller.
* `src/tacctrl/control/shear.py` – admittance controller: pre-grasp → shear loop, tactile gravity weighting, Jacobian mapping.
* `src/tacctrl/hardware/paxinihand_wrapper_bgthread.py` – thin wrapper around `pxdex.dh13`, keeps SDK calls inside a background process (position & tactile I/O only).
* `src/tacctrl/config.py` – all tunable parameters: pre-grasp pose, tactile gains, virtual mass/damping/stiffness, tactile scaling, etc.

## Prerequisites

* Python ≥ 3.10
* Paxini DexH13 SDK (`pxdex` wheel provided by Paxini)
* `numpy`, `pin`, and any other deps already listed in your environment
* Access to the DexH13 hand via USB serial + camera (the CLI defaults point at `/dev/serial/by-id/...` and `/dev/video0`)

## Running the Controller

```bash
conda activate neural-teleop2  # or your virtualenv containing pxdex/numpy/pin
cd TacCtrl
python scripts/run_shear_control.py \
    --hand-tty /dev/ttyUSB0 \
    --camera-port /dev/video0 \
    --urdf robot_description/xarm_paxini/hands/paxini_hand/dexh13_hand_right_description.urdf \
    --rate-hz 150 \
    --driver-rate-hz 200 \
    --calibrate-tactile
```

While the script is running it prints the 11 tactile modules every 0.5 s (already scaled to newtons).  Press `Ctrl+C` to stop; the background SDK process is shut down automatically.

> **Left-hand support** – set the environment variable `DEXH13_HAND=left` (and pass the left-hand URDF via `--urdf`) to automatically use the left-hand joint/tactile ordering.

### Customising / Tuning

* **Pre-grasp / initial pose** – `DEFAULT_INITIAL_JOINT_TARGETS` and `DEFAULT_PREGRASP_JOINT_TARGETS` in `src/tacctrl/config.py` define the thumb-opposition posture followed by the slow closure.
* **Tactile scaling** – `tactile_count_to_newton` (default `0.1`) converts SDK “counts” to N; change it after you calibrate the sensors.
* **Normal force setpoint** – `normal_force_setpoint` (default `1.0` N) is the per-module target when the admittance loop runs.
* **Virtual admittance** – `admittance_mass`, `admittance_damping`, `admittance_stiffness` give the diagonal `M/B/K` matrices used when mapping tactile force error into joint motion.
* **Normal-force limits** – per-finger `normal_force_max` prevent crushing the object; tune them alongside `normal_force_backoff`.

See `guidance.md` for the mathematical background (in Chinese).

## Notes About the Updated URDF

The controller always loads the URDF path passed via `--urdf` (defaults to the file under `robot_description/`).  As long as the updated URDF preserves link/frame names, no additional code changes are required—the Pinocchio `DexH13HandModel` reads rotation/translation/Jacobian data directly from the URDF at runtime.  If you rename frames or add/remove tactile links, remember to update `DEFAULT_MODULES` in `src/tacctrl/config.py` accordingly.

## Development Tips

* The hardware wrapper keeps the SDK calls inside a `multiprocessing.Process`; avoid touching the SDK from the main thread.
* Tactile printouts are useful for quick diagnostics; for deeper inspection, run `scripts/diagnose_tactile.py`.
* When adjusting gains, use light objects first—virtual admittance can quickly become unstable with overly small damping.

## License

No license file is included; please add one before publishing the repository publicly if needed.
