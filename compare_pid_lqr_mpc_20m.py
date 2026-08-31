import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# 1. 实验文件
# ============================================================

pidFile = "auv_pid_20m.csv"
lqrFile = "auv_lqr_20m.csv"
mpcFile = "auv_mpc_20m.csv"


# ============================================================
# 2. 实验目标
# ============================================================

targetDepthValue = 20.0


# ============================================================
# 3. 执行器约束
# ============================================================

outputMax = 5.0
outputMin = -5.0

# 控制输出变化率：
#
# 3 N/s × 0.1 s = 0.3 N/step
maxDeltaOutput = 0.3


# ============================================================
# 4. 数值约束检查容差
# ============================================================

constraintTolerance = 0.005


# ============================================================
# 5. 输出文件夹
# ============================================================

outputFolder = "pid_lqr_mpc_20m_comparison"

os.makedirs(
    outputFolder,
    exist_ok=True
)


# ============================================================
# 6. 检查并读取CSV
# ============================================================

def read_csv_checked(fileName):

    if not os.path.exists(fileName):

        raise FileNotFoundError(
            f"文件不存在: {fileName}"
        )

    if os.path.getsize(fileName) == 0:

        raise ValueError(
            f"CSV文件为空: {fileName}"
        )

    data = pd.read_csv(fileName)

    if data.empty:

        raise ValueError(
            f"CSV没有实验数据: {fileName}"
        )

    print(
        fileName,
        ":",
        len(data),
        "rows"
    )

    return data


pidRaw = read_csv_checked(pidFile)
lqrRaw = read_csv_checked(lqrFile)
mpcRaw = read_csv_checked(mpcFile)


# ============================================================
# 7. 检查必须存在的列
# ============================================================

def check_required_columns(
    data,
    fileName,
    requiredColumns
):

    missingColumns = [

        column
        for column in requiredColumns
        if column not in data.columns

    ]

    if missingColumns:

        raise ValueError(
            f"{fileName} 缺少列: {missingColumns}\n"
            f"当前列为: {list(data.columns)}"
        )


check_required_columns(
    pidRaw,
    pidFile,
    [
        "time",
        "current_depth",
        "output"
    ]
)

check_required_columns(
    lqrRaw,
    lqrFile,
    [
        "time",
        "current_depth",
        "output"
    ]
)

check_required_columns(
    mpcRaw,
    mpcFile,
    [
        "time",
        "current_depth",
        "output"
    ]
)


# ============================================================
# 8. 将三种控制器统一成相同数据格式
#
# 最终只保留公平比较真正需要的量：
#
# time
# target_depth
# current_depth
# output
#
# PID不再强制要求velocity列
# ============================================================

def normalize_data(
    rawData,
    controllerName
):

    data = pd.DataFrame()

    data["time"] = (
        rawData["time"]
        .astype(float)
    )

    # --------------------------------------------------------
    # 如果原CSV有target_depth就直接使用
    #
    # 如果没有，则使用本实验统一目标20m
    # --------------------------------------------------------

    if "target_depth" in rawData.columns:

        data["target_depth"] = (
            rawData["target_depth"]
            .astype(float)
        )

    else:

        data["target_depth"] = (
            targetDepthValue
        )


    data["current_depth"] = (
        rawData["current_depth"]
        .astype(float)
    )

    data["output"] = (
        rawData["output"]
        .astype(float)
    )


    # --------------------------------------------------------
    # 去除NaN和无穷值
    # --------------------------------------------------------

    data = data.replace(
        [np.inf, -np.inf],
        np.nan
    )

    data = data.dropna(
        subset=[
            "time",
            "target_depth",
            "current_depth",
            "output"
        ]
    )


    # --------------------------------------------------------
    # 按时间排序
    # --------------------------------------------------------

    data = data.sort_values(
        by="time"
    )


    # --------------------------------------------------------
    # 删除重复时间点
    # --------------------------------------------------------

    data = data.drop_duplicates(
        subset="time",
        keep="first"
    )


    data = data.reset_index(
        drop=True
    )


    print(
        controllerName,
        "normalized:",
        len(data),
        "rows"
    )

    return data


pid = normalize_data(
    pidRaw,
    "PID"
)

lqr = normalize_data(
    lqrRaw,
    "LQR"
)

mpc = normalize_data(
    mpcRaw,
    "MPC"
)


# ============================================================
# 9. 找到三种实验共同时间范围
#
# 避免某个实验运行100s，
# 另一个运行120s，
# 导致RMSE、控制能量比较不公平
# ============================================================

commonStartTime = max(
    pid["time"].iloc[0],
    lqr["time"].iloc[0],
    mpc["time"].iloc[0]
)

commonEndTime = min(
    pid["time"].iloc[-1],
    lqr["time"].iloc[-1],
    mpc["time"].iloc[-1]
)


pid = pid[
    (pid["time"] >= commonStartTime)
    &
    (pid["time"] <= commonEndTime)
].copy()

lqr = lqr[
    (lqr["time"] >= commonStartTime)
    &
    (lqr["time"] <= commonEndTime)
].copy()

mpc = mpc[
    (mpc["time"] >= commonStartTime)
    &
    (mpc["time"] <= commonEndTime)
].copy()


print()

print(
    "Common comparison time:",
    commonStartTime,
    "s ~",
    commonEndTime,
    "s"
)

print()


# ============================================================
# 10. 统一计算误差与控制增量
#
# 不使用三个程序原来保存的error
#
# 统一定义：
#
# error = current_depth - target_depth
#
# 因为之前PID和LQR/MPC可能采用不同误差符号定义
# ============================================================

def prepare_data(data):

    data = data.copy()

    data["comparison_error"] = (
        data["current_depth"]
        -
        data["target_depth"]
    )


    # --------------------------------------------------------
    # 控制增量
    #
    # 第一时刻假设上一时刻输出为0
    #
    # Δu(0) = u(0) - 0
    # --------------------------------------------------------

    outputArray = (
        data["output"]
        .to_numpy()
    )

    deltaOutput = np.diff(
        np.insert(
            outputArray,
            0,
            0.0
        )
    )

    data["delta_output"] = (
        deltaOutput
    )

    return data


pid = prepare_data(pid)
lqr = prepare_data(lqr)
mpc = prepare_data(mpc)


# ============================================================
# 11. 性能指标计算函数
# ============================================================

def calculate_metrics(
    data,
    controllerName
):

    time = (
        data["time"]
        .to_numpy()
    )

    depth = (
        data["current_depth"]
        .to_numpy()
    )

    target = (
        data["target_depth"]
        .to_numpy()
    )

    output = (
        data["output"]
        .to_numpy()
    )

    error = (
        data["comparison_error"]
        .to_numpy()
    )

    deltaOutput = (
        data["delta_output"]
        .to_numpy()
    )


    # ========================================================
    # 基本参数
    # ========================================================

    initialDepth = (
        depth[0]
    )

    targetValue = (
        target[-1]
    )

    stepAmplitude = (
        targetValue
        -
        initialDepth
    )


    # ========================================================
    # 1. 上升时间 Rise Time
    #
    # 10% → 90%
    #
    # 对0 → 20m：
    #
    # 2m → 18m
    # ========================================================

    depth10 = (
        initialDepth
        +
        0.1 * stepAmplitude
    )

    depth90 = (
        initialDepth
        +
        0.9 * stepAmplitude
    )


    if stepAmplitude >= 0:

        index10 = np.where(
            depth >= depth10
        )[0]

        index90 = np.where(
            depth >= depth90
        )[0]

    else:

        index10 = np.where(
            depth <= depth10
        )[0]

        index90 = np.where(
            depth <= depth90
        )[0]


    if (
        len(index10) > 0
        and
        len(index90) > 0
    ):

        t10 = (
            time[index10[0]]
        )

        t90 = (
            time[index90[0]]
        )

        riseTime = (
            t90
            -
            t10
        )

    else:

        riseTime = np.nan


    # ========================================================
    # 2. 首次达到目标时间
    #
    # 找第一次达到或者越过20m
    # ========================================================

    if stepAmplitude >= 0:

        targetIndex = np.where(
            depth >= targetValue
        )[0]

    else:

        targetIndex = np.where(
            depth <= targetValue
        )[0]


    if len(targetIndex) > 0:

        firstTargetTime = (
            time[
                targetIndex[0]
            ]
        )

    else:

        firstTargetTime = np.nan


    # ========================================================
    # 3. 调节时间 Settling Time
    #
    # 使用±2%阶跃幅值
    #
    # 0 → 20m：
    #
    # ±0.4m
    #
    # 即：
    #
    # 19.6m ~ 20.4m
    #
    # 进入后要求后续始终不离开该范围
    # ========================================================

    settlingBand = (
        0.02
        *
        abs(stepAmplitude)
    )

    if settlingBand == 0:

        settlingBand = 0.02


    insideBand = (
        np.abs(
            depth
            -
            targetValue
        )
        <=
        settlingBand
    )


    settlingTime = np.nan


    for i in range(
        len(time)
    ):

        if np.all(
            insideBand[i:]
        ):

            settlingTime = (
                time[i]
            )

            break


    # ========================================================
    # 4. 最大深度
    # ========================================================

    maximumDepth = np.max(
        depth
    )


    # ========================================================
    # 5. 最大超调量
    #
    # Mp =
    #
    # (zmax - zd)
    # -----------
    #   Δz
    #
    # ×100%
    # ========================================================

    if stepAmplitude > 0:

        overshoot = (
            (
                maximumDepth
                -
                targetValue
            )
            /
            abs(stepAmplitude)
            *
            100.0
        )

    else:

        minimumDepth = np.min(
            depth
        )

        overshoot = (
            (
                targetValue
                -
                minimumDepth
            )
            /
            abs(stepAmplitude)
            *
            100.0
        )


    overshoot = max(
        0.0,
        overshoot
    )


    # ========================================================
    # 6. 最终误差
    # ========================================================

    finalError = abs(
        error[-1]
    )


    # ========================================================
    # 7. 稳态误差
    #
    # 使用最后10秒：
    #
    # 平均绝对误差
    #
    # 避免只看最后一个采样点受到噪声影响
    # ========================================================

    endTime = (
        time[-1]
    )

    steadyStartTime = max(
        time[0],
        endTime - 10.0
    )

    steadyMask = (
        time
        >=
        steadyStartTime
    )


    steadyStateError = np.mean(
        np.abs(
            error[
                steadyMask
            ]
        )
    )


    # ========================================================
    # 8. RMSE
    #
    # 整个共同实验时间范围计算
    # ========================================================

    rmse = np.sqrt(
        np.mean(
            error ** 2
        )
    )


    # ========================================================
    # 9. 最大控制量
    # ========================================================

    maximumControl = np.max(
        np.abs(
            output
        )
    )


    # ========================================================
    # 10. 最大控制增量
    # ========================================================

    maximumDeltaControl = np.max(
        np.abs(
            deltaOutput
        )
    )


    # ========================================================
    # 11. 控制能量
    #
    # Ju = ∫u²dt
    #
    # 使用梯形积分，
    # 比简单Σu²dt对时间误差更稳妥
    # ========================================================

    if len(time) > 1:

        controlEnergy = np.trapz(
            output ** 2,
            time
        )

    else:

        controlEnergy = np.nan


    # ========================================================
    # 12. 平均绝对控制量
    # ========================================================

    meanAbsoluteControl = np.mean(
        np.abs(
            output
        )
    )


    # ========================================================
    # 返回结果
    # ========================================================

    return {

        "Controller":
            controllerName,

        "Rise Time 10-90% (s)":
            riseTime,

        "First Target Time (s)":
            firstTargetTime,

        "Settling Time 2% (s)":
            settlingTime,

        "Maximum Depth (m)":
            maximumDepth,

        "Overshoot (%)":
            overshoot,

        "Final Error (m)":
            finalError,

        "Steady-State Error (m)":
            steadyStateError,

        "RMSE (m)":
            rmse,

        "Maximum Control (N)":
            maximumControl,

        "Maximum Delta Control (N/step)":
            maximumDeltaControl,

        "Mean Absolute Control (N)":
            meanAbsoluteControl,

        "Control Energy (N^2*s)":
            controlEnergy

    }


# ============================================================
# 12. 分别计算PID、LQR、MPC
# ============================================================

pidMetrics = calculate_metrics(
    pid,
    "PID"
)

lqrMetrics = calculate_metrics(
    lqr,
    "LQR"
)

mpcMetrics = calculate_metrics(
    mpc,
    "MPC"
)


# ============================================================
# 13. 生成性能指标表
# ============================================================

metricsTable = pd.DataFrame(
    [
        pidMetrics,
        lqrMetrics,
        mpcMetrics
    ]
)


# ============================================================
# 14. 数值保留5位小数
# ============================================================

numericColumns = (
    metricsTable
    .select_dtypes(
        include=[np.number]
    )
    .columns
)

metricsTable[
    numericColumns
] = (
    metricsTable[
        numericColumns
    ]
    .round(5)
)


# ============================================================
# 15. 保存性能指标CSV
# ============================================================

metricsFile = os.path.join(
    outputFolder,
    "pid_lqr_mpc_20m_metrics.csv"
)

metricsTable.to_csv(
    metricsFile,
    index=False
)


# ============================================================
# 16. 保存统一后的数据
#
# 方便以后检查
# ============================================================

pid.to_csv(
    os.path.join(
        outputFolder,
        "normalized_pid_20m.csv"
    ),
    index=False
)

lqr.to_csv(
    os.path.join(
        outputFolder,
        "normalized_lqr_20m.csv"
    ),
    index=False
)

mpc.to_csv(
    os.path.join(
        outputFolder,
        "normalized_mpc_20m.csv"
    ),
    index=False
)


# ============================================================
# 17. 打印性能指标
# ============================================================

print()

print(
    "========================================================================================================================="
)

print(
    "PID / LQR / MPC 20 m Fair Comparison"
)

print(
    "========================================================================================================================="
)

print()

print(
    metricsTable.to_string(
        index=False
    )
)

print()

print(
    "========================================================================================================================="
)

print()


# ============================================================
# 18. 深度响应对比
# ============================================================

plt.figure(
    figsize=(10, 6)
)

plt.plot(
    pid["time"],
    pid["current_depth"],
    linewidth=2,
    label="PID"
)

plt.plot(
    lqr["time"],
    lqr["current_depth"],
    linewidth=2,
    label="LQR"
)

plt.plot(
    mpc["time"],
    mpc["current_depth"],
    linewidth=2,
    label="MPC"
)

plt.axhline(
    targetDepthValue,
    linestyle="--",
    linewidth=2,
    label="Target Depth"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Depth (m)"
)

plt.title(
    "PID vs LQR vs MPC - Depth Response (20 m)"
)

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "01_depth_comparison.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 19. 深度误差对比
# ============================================================

plt.figure(
    figsize=(10, 6)
)

plt.plot(
    pid["time"],
    pid["comparison_error"],
    linewidth=2,
    label="PID"
)

plt.plot(
    lqr["time"],
    lqr["comparison_error"],
    linewidth=2,
    label="LQR"
)

plt.plot(
    mpc["time"],
    mpc["comparison_error"],
    linewidth=2,
    label="MPC"
)

plt.axhline(
    0.0,
    linestyle="--",
    label="Zero Error"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Depth Error (m)"
)

plt.title(
    "PID vs LQR vs MPC - Depth Error (20 m)"
)

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "02_error_comparison.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 20. 控制输出对比
# ============================================================

plt.figure(
    figsize=(10, 6)
)

plt.plot(
    pid["time"],
    pid["output"],
    linewidth=2,
    label="PID"
)

plt.plot(
    lqr["time"],
    lqr["output"],
    linewidth=2,
    label="LQR"
)

plt.plot(
    mpc["time"],
    mpc["output"],
    linewidth=2,
    label="MPC"
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

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Control Output (N)"
)

plt.title(
    "PID vs LQR vs MPC - Control Output (20 m)"
)

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "03_control_comparison.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 21. 控制增量对比
# ============================================================

plt.figure(
    figsize=(10, 6)
)

plt.plot(
    pid["time"],
    pid["delta_output"],
    linewidth=1.5,
    label="PID"
)

plt.plot(
    lqr["time"],
    lqr["delta_output"],
    linewidth=1.5,
    label="LQR"
)

plt.plot(
    mpc["time"],
    mpc["delta_output"],
    linewidth=1.5,
    label="MPC"
)

plt.axhline(
    maxDeltaOutput,
    linestyle="--",
    label="+0.3 N/step"
)

plt.axhline(
    -maxDeltaOutput,
    linestyle="--",
    label="-0.3 N/step"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Delta Control Output (N)"
)

plt.title(
    "PID vs LQR vs MPC - Control Rate (20 m)"
)

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "04_control_rate_comparison.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 22. 综合4图
# ============================================================

fig, ax = plt.subplots(
    4,
    1,
    figsize=(12, 15)
)


# ------------------------------------------------------------
# 第1图：深度
# ------------------------------------------------------------

ax[0].plot(
    pid["time"],
    pid["current_depth"],
    linewidth=2,
    label="PID"
)

ax[0].plot(
    lqr["time"],
    lqr["current_depth"],
    linewidth=2,
    label="LQR"
)

ax[0].plot(
    mpc["time"],
    mpc["current_depth"],
    linewidth=2,
    label="MPC"
)

ax[0].axhline(
    targetDepthValue,
    linestyle="--",
    linewidth=2,
    label="Target Depth"
)

ax[0].set_ylabel(
    "Depth (m)"
)

ax[0].set_title(
    "Depth Response"
)

ax[0].grid(True)

ax[0].legend()


# ------------------------------------------------------------
# 第2图：误差
# ------------------------------------------------------------

ax[1].plot(
    pid["time"],
    pid["comparison_error"],
    linewidth=2,
    label="PID"
)

ax[1].plot(
    lqr["time"],
    lqr["comparison_error"],
    linewidth=2,
    label="LQR"
)

ax[1].plot(
    mpc["time"],
    mpc["comparison_error"],
    linewidth=2,
    label="MPC"
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

ax[1].legend()


# ------------------------------------------------------------
# 第3图：控制输出
# ------------------------------------------------------------

ax[2].plot(
    pid["time"],
    pid["output"],
    linewidth=2,
    label="PID"
)

ax[2].plot(
    lqr["time"],
    lqr["output"],
    linewidth=2,
    label="LQR"
)

ax[2].plot(
    mpc["time"],
    mpc["output"],
    linewidth=2,
    label="MPC"
)

ax[2].axhline(
    outputMax,
    linestyle="--",
    label="+5 N"
)

ax[2].axhline(
    outputMin,
    linestyle="--",
    label="-5 N"
)

ax[2].axhline(
    0.0,
    linestyle="--"
)

ax[2].set_ylabel(
    "Control Output (N)"
)

ax[2].set_title(
    "Control Output"
)

ax[2].grid(True)

ax[2].legend()


# ------------------------------------------------------------
# 第4图：控制变化量
# ------------------------------------------------------------

ax[3].plot(
    pid["time"],
    pid["delta_output"],
    linewidth=1.5,
    label="PID"
)

ax[3].plot(
    lqr["time"],
    lqr["delta_output"],
    linewidth=1.5,
    label="LQR"
)

ax[3].plot(
    mpc["time"],
    mpc["delta_output"],
    linewidth=1.5,
    label="MPC"
)

ax[3].axhline(
    maxDeltaOutput,
    linestyle="--",
    label="+0.3 N/step"
)

ax[3].axhline(
    -maxDeltaOutput,
    linestyle="--",
    label="-0.3 N/step"
)

ax[3].axhline(
    0.0,
    linestyle="--"
)

ax[3].set_xlabel(
    "Time (s)"
)

ax[3].set_ylabel(
    "Delta Output (N)"
)

ax[3].set_title(
    "Control Rate"
)

ax[3].grid(True)

ax[3].legend()


plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "05_pid_lqr_mpc_20m_summary.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 23. 执行器约束检查
# ============================================================

print()

print(
    "================================================="
)

print(
    "Control Constraint Check"
)

print(
    "================================================="
)


for controllerName, data in [

    ("PID", pid),
    ("LQR", lqr),
    ("MPC", mpc)

]:

    maxControl = (
        data["output"]
        .abs()
        .max()
    )

    maxDelta = (
        data["delta_output"]
        .abs()
        .max()
    )


    forcePassed = (
        maxControl
        <=
        outputMax
        +
        constraintTolerance
    )


    ratePassed = (
        maxDelta
        <=
        maxDeltaOutput
        +
        constraintTolerance
    )


    print()

    print(
        controllerName
    )

    print(
        "Maximum control:",
        round(
            maxControl,
            6
        ),
        "N"
    )

    print(
        "Maximum delta control:",
        round(
            maxDelta,
            6
        ),
        "N/step"
    )

    print(
        "Force constraint:",
        "PASSED"
        if forcePassed
        else
        "FAILED"
    )

    print(
        "Rate constraint:",
        "PASSED"
        if ratePassed
        else
        "FAILED"
    )


print()

print(
    "================================================="
)

print()


# ============================================================
# 24. 最终提示
# ============================================================

print(
    "Comparison figures saved in:",
    outputFolder
)

print(
    "Metrics saved in:",
    metricsFile
)

print()