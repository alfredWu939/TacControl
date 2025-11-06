**面向 Paxini DexH13** 的「剪切-力（shear）驱动抓取控制」方案，专门把**雅可比映射**补齐，逻辑从“触觉阵列 → 指尖接触力 → 关节力矩/电流”一路走通。为避免纸杯压瘪，同时不丢物体，这套方案把 **剪切变化率** 作为快环误差、**法向力** 作为约束与参照；在 DexH13 上按**多指多关节**实现.

---

# 0. 基本设定（硬件与接口）

* **触觉读取**：DexH13 SDK 提供
  `getFingerTactile()`（每个触觉模块的合力 (x,y,z)）与
  `getFingerTactileDetail()`（阵列分布力，总 1140 个通道；部分模块 60 点，其余 120 点） 。
  共 **11 个触觉模块**（拇指+食/中/环分布总计 11）。
* **关节/电机控制**：Python/C++ SDK 有
  `setMotorTargetCurrent`（电机目标电流）、`setJointPositions*`（角度/弧度）、`getJointPositions*`（读角）等接口，可做电流/位置/速度控制与上电校准、故障处理等  。
* **通信**：支持 Modbus/EtherCAT，上层可二次开发；SDK 依赖运动学库（可用 URDF 结合 Pinocchio 计算雅可比）。

> 机构参数采纳：**拇指 4 个电机，其余各指 3 个**；**拇指 2 个触觉模块，其余各指 3 个**（与 SDK 报告的 11 模块总数一致）。

---

Algorithm: Jacobian-Based Shear-Force Grasp Stabilization (Orientation-Weighted)

# ----------------------------------------------------------
# 1. Pre-grasp establishment (same as before)
# ----------------------------------------------------------
function EstablishPregrasp()
    q_pregrasp ← predefined joint configuration (pregrasp posture)
    grasped ← False
    while grasped = False do
        tactile_data ← getFingerTactile()
        q_current ← getJointPositionsRadian()
        moveHandToward(q_pregrasp, q_current)
        contact_flags ← evaluateContact(tactile_data, pressure_threshold)
        if contact_flags.thumb and contact_flags.index and contact_flags.middle then
            grasped ← True
        end if
    end while
end function


# ----------------------------------------------------------
# 2. Dynamic stabilization with full 3D orientation weighting
# ----------------------------------------------------------
initialize PID controllers for shear-force control
i ← 0
while True do
    # 2.1  Get current pose and rotation of the grasp frame
    pose ← robot.getPose()
    (α, β, γ) ← extractRotation(pose)              # Euler angles (XYZ/SXYZ convention)

    # 2.2  Build rotation matrix of grasp frame relative to base
    R_SXYZ ← R_z(γ) * R_y(β) * R_x(α)

    # 2.3  Compute rotated local axes of the grasp frame
    x_prime ← R_SXYZ * [1, 0, 0]^T
    y_prime ← R_SXYZ * [0, −1, 0]^T

    # 2.4  Define gravity vector in global frame (downward z_G)
    g ← [0, 0, −1]^T

    # 2.5  Compute orientation angles between gravity and local axes
    θ_x ← arccos( dot(g, x_prime) / (‖g‖ ‖x_prime‖) )
    θ_y ← arccos( dot(g, y_prime) / (‖g‖ ‖y_prime‖) )

    # 2.6  Compute scaling factors (Eq. 2–3 in the paper)
    S_x ← 1 − (2 * θ_x / π)
    S_y ← 1 − (2 * θ_y / π)

    # 2.7  Acquire tactile forces
    (f_t, f_n, n_hat) ← extractForces(tactile_data)
    J ← computeHandJacobian(q_current)

    # 2.8  Compute shear-force variation
    if i ≥ δ then
        ΔF_x ← f_t.x[i] − f_t.x[i − δ]
        ΔF_y ← f_t.y[i] − f_t.y[i − δ]

        # Weighted control inputs according to hand orientation
        u_x ← S_x * ΔF_x
        u_y ← S_y * ΔF_y

        # PID in tangential domain (ΔF_target = 0)
        u_t ← PID_x(u_x) + PID_y(u_y)

        # Normal-force limiting term
        f_n_excess ← ReLU(f_n − f_n_max)
        u_n ← −K_n * f_n_excess * n_hat

        # Combine tangential and normal components in the contact frame
        f_cmd ← u_t + u_n                      # desired net contact force (N, 3×1 vector)

        # Compute corresponding Cartesian velocity (admittance behavior)
        # M, B, K: virtual mass, damping, stiffness matrices in task space
        # Δx_dot_cmd = A * (f_cmd - f_meas) can be simplified as:
        x_ddot_cmd ← M⁻¹ * (f_cmd - B * x_dot - K * (x - x_ref))
        x_dot_cmd ← x_dot + x_ddot_cmd * dt
        x_cmd ← x + x_dot_cmd * dt

        # Map Cartesian motion to joint motion using the Jacobian pseudoinverse
        Δq_cmd ← J⁺ * (x_cmd - x)              # J⁺: damped pseudoinverse of J
        q_cmd ← q_current + Δq_cmd

        # Send joint position command to the robot (admittance control output)
        pxdex.setJointPositionsRadian(q_cmd)
    end if

    i ← i + 1
end while
