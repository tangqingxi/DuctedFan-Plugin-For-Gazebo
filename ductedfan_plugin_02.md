# 🚀Gazebo 插件开发记录

## DuctedFanPlugin_02：基于 `gazebo_motor_model` 的电机模型替代版本

本版本用于验证自定义 `DuctedFanPlugin` 对 PX4-Gazebo 中原有 `gazebo_motor_model` 的替代可行性。当前版本重点完成插件加载、转速指令接收、电机动态响应、推力计算、反扭矩计算、力和力矩施加，以及仿真悬停验证等基础功能。

当前版本中的推力与力矩计算方法仍沿用 `gazebo_motor_model` 的基本建模方式，尚未引入`DuctedFan`论文中的气动计算模型。后续版本 `ductedfan_plugin_03` 将在该框架基础上进一步替换力和力矩计算方法，加入导管风扇相关的气动建模内容。

## 1. 插件替代验证

已完成 `plugins/ductedfan_plugin.cpp` 与 `plugins/ductedfan_plugin.h` 的基本实现，并将原有 `libgazebo_motor_model.so` 替换为自定义的 `libDuctedFanPlugin.so`，用于驱动旋翼系统完成仿真飞行。

该验证主要用于确认以下内容：

- 自定义插件能够被 Gazebo 正常加载；
- 插件能够正确读取 SDF 文件中的关键参数；
- 插件能够接收 PX4 发布的电机转速指令；
- 插件能够根据转速指令计算并施加对应的推力与力矩；
- 替换原有电机插件后，飞行器仍能够完成基本飞行任务。

<div style="overflow: auto;">
  <img src="/image/ductedfan_plugin_04.png" alt="DuctedFanPlugin 插件替代验证结果 1" style="float: left; width: 48%; margin-right: 4%;">
  <img src="/image/ductedfan_plugin_05.png" alt="DuctedFanPlugin 插件替代验证结果 2" style="float: left; width: 48%;">
</div>

## 2. 定高悬停验证

在替换原有 `gazebo_motor_model` 后，涵道式无人机 SHW09 已能够在 PX4-Gazebo 联合仿真环境中完成定高悬停任务。该结果表明，当前版本的 `DuctedFanPlugin` 已具备基本的旋翼驱动能力，能够支撑后续气动模型扩展与控制效果验证。

<div style="overflow: auto;">
  <img src="/image/ductedfan_plugin_06.png" alt="SHW09 定高悬停验证结果 1" style="float: left; width: 48%; margin-right: 4%;">
  <img src="/image/ductedfan_plugin_07.png" alt="SHW09 定高悬停验证结果 2" style="float: left; width: 48%;">
</div>

## 3. 命令行调试信息输出

为便于验证插件内部变量的计算过程，当前版本在 `plugins/ductedfan_plugin.cpp` 中加入了命令行调试输出功能。该功能用于观察电机转速、实际电机响应、推力、气动阻力、阻力矩、滚转力矩以及推力缩放系数等关键变量。

### 3.1 调试输出所需头文件

当前版本中加入了以下两个头文件，用于支持命令行输出和时间间隔控制。该部分主要用于调试阶段，后续在关闭调试输出后可以删除。

```cpp
// 用于临时运行时调试输出的头文件。
// 无需调试输出时，可移除这些头文件。
#include <iostream>
#include <chrono>
```

### 3.2 调试输出位置

调试输出代码位于以下函数中：

```cpp
void DuctedFanModel::UpdateForcesAndMoments()
```

该函数是插件中计算并施加旋翼力和力矩的核心函数，因此适合用于观察关键中间变量的变化情况。

### 3.3 调试输出控制方式

调试输出由 `DEBUG_PRINT` 变量控制，时间间隔由 `elapsed_ms` 变量控制。当 `DEBUG_PRINT` 设置为 `true`，且 `elapsed_ms >= 1000` 时，程序每隔 1000 ms 输出一次调试信息。

```cpp
// ================= Debug output =========================
// 在重复调用更新方法时，保留上一次的打印时间。
// DEBUG_PRINT 用于控制是否启用运行时调试输出。
const bool DEBUG_PRINT = true;

if (DEBUG_PRINT) {
    static std::chrono::steady_clock::time_point last_print_time =
        std::chrono::steady_clock::now();

    auto current_time = std::chrono::steady_clock::now();

    // 计算已流逝的时间（单位：毫秒）
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count();

    // 每 1000 毫秒（1秒）打印一次调试信息。
    if (elapsed_ms >= 1000) {
        std::cout << "--- Debug Info [Motor " << motor_number_ << "] ---" << std::endl;
        std::cout << "motor_rot_vel_: " << motor_rot_vel_ << "\n"
                  << "real_motor_velocity: " << real_motor_velocity << "\n"
                  << "force * scalar: " << force * scalar << "\n"
                  << "air_drag: " << air_drag << "\n"
                  << "drag_torque: " << drag_torque << "\n"
                  << "rolling_moment: " << rolling_moment << "\n"
                  << "scalar: " << scalar << std::endl;
        std::cout << "----------------------------------" << std::endl;

        // 打印完成后更新时间戳。
        last_print_time = current_time;
    }
}
// =======================================================
```

### 3.4 调试信息展示

命令行调试信息主要用于观察旋翼相关的力和力矩计算结果。当前输出内容包括：

- `motor_rot_vel_`：电机转速指令；
- `real_motor_velocity`：经过一阶滤波后的实际电机转速；
- `force * scalar`：考虑缩放系数后的推力；
- `air_drag`：旋翼叶片气动阻力；
- `drag_torque`：旋翼反扭矩；
- `rolling_moment`：滚转力矩；
- `scalar`：推力缩放系数。

![DuctedFanPlugin 命令行调试信息](/image/ductedfan_plugin_08.png)

## 4. 当前版本结论

`DuctedFanPlugin_02` 已完成对原有 `gazebo_motor_model` 的基础替代，并通过 SHW09 定高悬停任务验证了插件的基本可用性。该版本尚未改变原有电机模型的核心力学计算方法，其主要作用是建立可运行、可调试、可扩展的自定义插件框架。

后续 `DuctedFanPlugin_03` 将重点引入导管风扇气动模型，对推力、轴向来流影响、旋翼阻力、阻力矩和滚转力矩等内容进行进一步建模与验证。
