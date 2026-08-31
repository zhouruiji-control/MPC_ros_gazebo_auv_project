import os
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# CSV文件
# ============================================================

csvFile = "auv_mpc_variable_target.csv"


# ============================================================
# 图片保存文件夹
# ============================================================

outputFolder = "mpc_variable_target_figures"

os.makedirs(
    outputFolder,
    exist_ok=True
)


# ============================================================
# MPC约束参数
# ============================================================

outputMax = 5.0
outputMin = -5.0

# output_rate = 3 N/s
# dt = 0.1 s
# 每一步最大控制量变化：
# 3 × 0.1 = 0.3 N
maxDeltaOutput = 0.3

# OSQP数值求解允许存在很小的数值误差
# 例如 5.00032 N 和 0.30049 N/step 不应判定为约束失效
constraintTolerance = 1e-3


# ============================================================
# 读取CSV
# ============================================================

data = pd.read_csv(csvFile)


# ============================================================
# 读取数据
# ============================================================

time = data["time"]

targetDepth = data["target_depth"]

trueDepth = data["current_depth"]
measuredDepth = data["measured_depth"]
filteredDepth = data["filtered_depth"]

trueVelocity = data["velocity"]
measuredVelocity = data["measured_velocity"]
filteredVelocity = data["filtered_velocity"]

error = data["error"]

output = data["output"]


# ============================================================
# 计算MPC控制增量
#
# deltaOutput(k)
# =
# output(k) - output(k-1)
# ============================================================

deltaOutput = output.diff()


# ============================================================
# 1. 深度对比
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    trueDepth,
    label="True Depth",
    linewidth=2
)

plt.plot(
    time,
    measuredDepth,
    label="Measured Depth",
    linewidth=1,
    alpha=0.7
)

plt.plot(
    time,
    filteredDepth,
    label="Filtered Depth",
    linewidth=2
)

plt.plot(
    time,
    targetDepth,
    "--",
    label="Target Depth",
    linewidth=2
)

plt.xlabel("Time (s)")
plt.ylabel("Depth (m)")

plt.title(
    "MPC Depth Response with Measurement Noise"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "01_depth_response.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 2. 测量噪声与滤波效果
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    measuredDepth - trueDepth,
    label="Measurement Noise"
)

plt.plot(
    time,
    filteredDepth - trueDepth,
    label="Filtered Error"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Depth Difference (m)")

plt.title(
    "Depth Measurement Noise and Filtering"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "02_depth_noise_filter.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 3. 深度误差
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    error,
    linewidth=2
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Depth Error (m)")

plt.title(
    "MPC Depth Error"
)

plt.grid(True)

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "03_depth_error.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 4. 垂向速度对比
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    trueVelocity,
    label="True Velocity",
    linewidth=2
)

plt.plot(
    time,
    measuredVelocity,
    label="Measured Velocity",
    linewidth=1,
    alpha=0.7
)

plt.plot(
    time,
    filteredVelocity,
    label="Filtered Velocity",
    linewidth=2
)

plt.xlabel("Time (s)")
plt.ylabel("Vertical Velocity (m/s)")

plt.title(
    "MPC Vertical Velocity with Measurement Noise"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "04_vertical_velocity.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 5. MPC控制输出
#
# 同时画出：
# +5 N
# -5 N
#
# 用于检查MPC推力约束
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    output,
    linewidth=2,
    label="MPC Output"
)

plt.axhline(
    outputMax,
    linestyle="--",
    label="Upper Force Limit"
)

plt.axhline(
    outputMin,
    linestyle="--",
    label="Lower Force Limit"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Control Output (N)")

plt.title(
    "MPC Control Output"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "05_control_output.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 6. MPC控制量变化率约束
#
# delta u(k)
# =
# u(k) - u(k-1)
#
# 当前约束：
#
# -0.3 <= delta u <= 0.3
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    deltaOutput,
    linewidth=2,
    label="Control Increment"
)

plt.axhline(
    maxDeltaOutput,
    linestyle="--",
    label="Upper Rate Limit"
)

plt.axhline(
    -maxDeltaOutput,
    linestyle="--",
    label="Lower Rate Limit"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Control Increment (N)")

plt.title(
    "MPC Control Rate Constraint"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "06_control_rate_constraint.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 7. 综合图
# ============================================================

fig, ax = plt.subplots(
    5,
    1,
    figsize=(12, 17)
)


# ------------------------------------------------------------
# 第1张：深度
# ------------------------------------------------------------

ax[0].plot(
    time,
    trueDepth,
    label="True Depth",
    linewidth=2
)

ax[0].plot(
    time,
    measuredDepth,
    label="Measured Depth",
    linewidth=1,
    alpha=0.7
)

ax[0].plot(
    time,
    filteredDepth,
    label="Filtered Depth",
    linewidth=2
)

ax[0].plot(
    time,
    targetDepth,
    "--",
    label="Target Depth",
    linewidth=2
)

ax[0].set_ylabel(
    "Depth (m)"
)

ax[0].set_title(
    "MPC Depth Response"
)

ax[0].grid(True)
ax[0].legend()


# ------------------------------------------------------------
# 第2张：误差
# ------------------------------------------------------------

ax[1].plot(
    time,
    error,
    linewidth=2
)

ax[1].axhline(
    0.0,
    linestyle="--"
)

ax[1].set_ylabel(
    "Error (m)"
)

ax[1].set_title(
    "Depth Error"
)

ax[1].grid(True)


# ------------------------------------------------------------
# 第3张：速度
# ------------------------------------------------------------

ax[2].plot(
    time,
    trueVelocity,
    label="True Velocity",
    linewidth=2
)

ax[2].plot(
    time,
    measuredVelocity,
    label="Measured Velocity",
    linewidth=1,
    alpha=0.7
)

ax[2].plot(
    time,
    filteredVelocity,
    label="Filtered Velocity",
    linewidth=2
)

ax[2].set_ylabel(
    "Velocity (m/s)"
)

ax[2].set_title(
    "Vertical Velocity"
)

ax[2].grid(True)
ax[2].legend()


# ------------------------------------------------------------
# 第4张：控制量
# ------------------------------------------------------------

ax[3].plot(
    time,
    output,
    linewidth=2,
    label="MPC Output"
)

ax[3].axhline(
    outputMax,
    linestyle="--",
    label="Upper Force Limit"
)

ax[3].axhline(
    outputMin,
    linestyle="--",
    label="Lower Force Limit"
)

ax[3].axhline(
    0.0,
    linestyle="--"
)

ax[3].set_ylabel(
    "Control Output (N)"
)

ax[3].set_title(
    "MPC Control Output"
)

ax[3].grid(True)
ax[3].legend()


# ------------------------------------------------------------
# 第5张：控制量变化
# ------------------------------------------------------------

ax[4].plot(
    time,
    deltaOutput,
    linewidth=2,
    label="Control Increment"
)

ax[4].axhline(
    maxDeltaOutput,
    linestyle="--",
    label="Upper Rate Limit"
)

ax[4].axhline(
    -maxDeltaOutput,
    linestyle="--",
    label="Lower Rate Limit"
)

ax[4].axhline(
    0.0,
    linestyle="--"
)

ax[4].set_xlabel(
    "Time (s)"
)

ax[4].set_ylabel(
    "Delta Output (N)"
)

ax[4].set_title(
    "MPC Control Rate Constraint"
)

ax[4].grid(True)
ax[4].legend()


plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "07_mpc_noise_summary.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 打印基本信息
# ============================================================

print()
print("========================================")
print("MPC Noise Experiment")
print("========================================")

print(
    "Target depth:",
    targetDepth.iloc[-1],
    "m"
)

print(
    "Final true depth:",
    trueDepth.iloc[-1],
    "m"
)

print(
    "Final depth error:",
    error.iloc[-1],
    "m"
)

print(
    "Maximum depth:",
    trueDepth.max(),
    "m"
)

print(
    "Maximum velocity:",
    trueVelocity.abs().max(),
    "m/s"
)

print(
    "Maximum control output:",
    output.abs().max(),
    "N"
)

print(
    "Maximum control increment:",
    deltaOutput.abs().max(),
    "N"
)

print(
    "Maximum measurement depth error:",
    (measuredDepth - trueDepth).abs().max(),
    "m"
)

print(
    "Maximum filtered depth error:",
    (filteredDepth - trueDepth).abs().max(),
    "m"
)


# ============================================================
# MPC约束检查
# ============================================================

print()
print("========================================")
print("MPC Constraint Check")
print("========================================")

print(
    "Force limit:",
    outputMin,
    "to",
    outputMax,
    "N"
)

print(
    "Maximum absolute force:",
    output.abs().max(),
    "N"
)

print(
    "Rate limit:",
    maxDeltaOutput,
    "N/step"
)

print(
    "Maximum absolute control increment:",
    deltaOutput.abs().max(),
    "N/step"
)


# ============================================================
# 判断推力限制是否满足
# ============================================================

if (
    output.max() <= outputMax + constraintTolerance
    and
    output.min() >= outputMin - constraintTolerance
):
    print(
        "Force constraint: PASSED"
    )
else:
    print(
        "Force constraint: FAILED"
    )


# ============================================================
# 判断变化率限制是否满足
# ============================================================

if (
    deltaOutput.dropna().abs().max()
    <=
    maxDeltaOutput + constraintTolerance
):
    print(
        "Rate constraint: PASSED"
    )
else:
    print(
        "Rate constraint: FAILED"
    )


print("========================================")
print()

print(
    "Figures saved in:",
    outputFolder
)