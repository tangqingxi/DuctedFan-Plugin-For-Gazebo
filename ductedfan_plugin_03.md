# 🚀Gazebo 插件开发记录：DuctedFanPlugin_03

## 1. 开发目标

本文件记录 `DuctedFanPlugin_03` 的开发过程。本版本在 `DuctedFanPlugin_02` 已完成涵道与机翼气动力计算、样条插值和调试打印的基础上，进一步将气动力与气动力矩统一施加到 SHC09 机体刚体上，并扩展涵道风扇模型中的诱导速度、非轴流阻尼、涵道反扭矩和风扇陀螺力矩等动力学项。

当前版本主要完成以下工作：

1. 在 SDF 插件配置中补充涵道扩压比、桨盘面积、空气密度、俯仰阻尼、涵道反扭矩和风扇转动惯量等参数；
2. 在 `ductedfan_plugin.h` 中增加对应成员变量，用于保存新增气动参数；
3. 在 `Load()` 函数中获取 `base_link`，作为后续统一施加机体气动力和气动力矩的刚体对象；
4. 在 `UpdateForcesAndMoments()` 中按照 MATLAB / 论文模型重构力与力矩计算流程；
5. 将 MATLAB 坐标系下的合力与合力矩转换为 Gazebo ENU 局部坐标系；
6. 通过 `AddRelativeForce()` 和 `AddRelativeTorque()` 将合力与合力矩作用于 `base_link`；
7. 保留电机转速滤波与关节速度设定逻辑，使插件能够继续承接原 `gazebo_motor_model` 的基础电机控制功能。

> 当前状态：`DuctedFanPlugin` 已经基本替代原 `gazebo_motor_model` 的电机控制与力矩计算职责，并完成气动力 / 气动力矩对机体刚体的实际施加。`lift_drag` 插件的功能尚未完全合入，后续重点是继续补充 `M_cs` 等剩余气动力矩项，并开展悬停的仿真验证。

---

## 2. 本版本与上一版本的关系

| 版本 | 已完成内容 | 当前不足 |
|---|---|---|
| `DuctedFanPlugin_02` | 完成 MATLAB 样条系数导出、CSV 读取、`ppval()` 等价计算、涵道与机翼力 / 力矩计算、调试打印 | 力与力矩仅计算和打印，尚未施加到模型刚体 |
| `DuctedFanPlugin_03` | 在 `02` 基础上补充总气动力 / 总力矩合成，并施加到 `base_link`；增加非轴流阻尼、涵道反扭矩、风扇陀螺力矩等项 | `M_cs` 等剩余模型项尚未接入，仍需开展系统性工况验证 |

---

## 3. 文件与模块规划

| 模块 | 文件 / 位置 | 本次修改内容 |
|---|---|---|
| SDF 插件配置 | `SHC09.sdf.jinja` | 增加涵道气动、阻尼、反扭矩和风扇惯量参数 |
| 插件头文件 | `ductedfan_plugin.h` | 增加新增参数对应的成员变量 |
| 插件主逻辑 | `ductedfan_plugin.cpp` | 获取 `base_link`，重构力 / 力矩计算，并施加到机体刚体 |

---

## 4. SDF 参数扩展

### 4.1 修改位置

在 `SHC09.sdf.jinja` 中的 `rotor_ductedfan_plugin` 插件块内补充以下参数。该部分参数用于支撑涵道风扇完整力 / 力矩模型计算。

```bash
<!-- k_T0：悬停时的拉力系数 -->
<motorConstant>2.8964e-05</motorConstant>

<!-- k_Th：推力-速度耦合项，与 C_T_J0 相关 -->
<thrustVelocityCoupling>-3.7614e-04</thrustVelocityCoupling>

<!-- k_Ts：推力-速度-迎角耦合项，与 C_T_J 相关 -->
<thrustAoaCoupling>-2.6733e-04</thrustAoaCoupling>

<!-- k_Ns：侧向力-速度-迎角耦合项，与 C_N_J 相关 -->
<sideForceVelocityCoupling>5.0699e-04</sideForceVelocityCoupling>

<!-- k_Q0：风扇反扭矩系数，与 C_Q 相关 -->
<momentConstant>5.7792e-07</momentConstant>

<!-- l_cpz：侧向力 N_df 对机体中心的力臂 -->
<sideForceArmZ>-0.15</sideForceArmZ>

<!-- k_cpx：拉力中心法向偏移系数 -->
<thrustCenterOffsetCoeff>2.6619</thrustCenterOffsetCoeff>

<!-- sd：涵道扩压比 -->
<ductExpansionRatio>0.7</ductExpansionRatio>

<!-- S：桨盘面积，单位 m^2 -->
<ductDiskArea>0.0408</ductDiskArea>

<!-- den：空气密度，单位 kg/m^3 -->
<airDensity>1.225</airDensity>

<!-- k_my0：非轴流俯仰阻尼常数项 -->
<pitchDampingConstant>0.0198</pitchDampingConstant>

<!-- k_myv：非轴流俯仰阻尼速度项系数 -->
<pitchDampingVelocityCoeff>0.0015</pitchDampingVelocityCoeff>

<!-- k_sta：涵道反扭矩系数 -->
<ductTorqueCoeff>5.3688e-04</ductTorqueCoeff>

<!-- I_fan：风扇转动惯量，单位 kg·m^2 -->
<fanInertia>3.7e-05</fanInertia>
```

### 4.2 参数含义

| SDF 标签 | 代码变量 | 物理含义 |
|---|---|---|
| `ductExpansionRatio` | `duct_sd_` | 涵道扩压比，用于诱导速度计算 |
| `ductDiskArea` | `duct_S_` | 桨盘面积 |
| `airDensity` | `air_density_` | 空气密度 |
| `pitchDampingConstant` | `k_my0_` | 非轴流俯仰阻尼常数项 |
| `pitchDampingVelocityCoeff` | `k_myv_` | 非轴流俯仰阻尼速度项系数 |
| `ductTorqueCoeff` | `k_sta_` | 涵道反扭矩系数 |
| `fanInertia` | `I_fan_` | 风扇转动惯量 |

---

## 5. `ductedfan_plugin.h` 修改

在 `private` 区域中增加以下成员变量，用于保存 SDF 中新增的涵道气动与力矩参数。

```cpp
double duct_sd_;       // 涵道扩压比
double duct_S_;        // 桨盘面积
double air_density_;   // 空气密度

double k_my0_;         // 非轴流俯仰阻尼常数项
double k_myv_;         // 非轴流俯仰阻尼速度项系数

double k_sta_;         // 涵道反扭矩系数
double I_fan_;         // 风扇转动惯量
```

说明：原始记录中写作“增加三个成员变量”，实际新增的是七个成员变量。这里统一修正为“增加以下成员变量”。

---

## 6. `ductedfan_plugin.cpp` 修改

### 6.1 在 `Load()` 函数中获取机体基准连杆

本版本不再仅对旋翼连杆进行局部处理，而是将合成后的气动力和气动力矩统一施加到机体基准连杆 `base_link`。因此，需要在 `Load()` 函数中获取 `base_link` 指针。

```cpp
// 获取机体基准连杆，用于后续施加合力和合力矩
base_link_ = model_->GetLink("base_link");
if (!base_link_) {
  gzerr << "[ductedfan_plugin] 无法找到 base_link，机体气动力和力矩将无法施加！\n";
}
```

该处理的作用是：

1. 将涵道风扇、机翼和阻尼等多个来源的气动力统一作用到机体刚体；
2. 避免将总气动力误施加到局部旋翼连杆，导致力学作用点和响应对象不一致；
3. 为后续整合 `lift_drag` 插件中的气动模型提供统一接口。

### 6.2 读取新增 SDF 参数

在 `Load()` 函数中，继续使用 `getSdfParam` 读取新增参数。第四个参数为默认值，用于避免 SDF 文件缺失参数时导致插件初始化失败。

```cpp
getSdfParam<double>(_sdf, "ductExpansionRatio", duct_sd_, 0.7);
getSdfParam<double>(_sdf, "ductDiskArea", duct_S_, M_PI * 0.114 * 0.114);
getSdfParam<double>(_sdf, "airDensity", air_density_, 1.225);

getSdfParam<double>(_sdf, "pitchDampingConstant", k_my0_, 0.0);
getSdfParam<double>(_sdf, "pitchDampingVelocityCoeff", k_myv_, 0.0);

getSdfParam<double>(_sdf, "ductTorqueCoeff", k_sta_, 0.0);
getSdfParam<double>(_sdf, "fanInertia", I_fan_, 0.0);
```

---

## 7. `UpdateForcesAndMoments()` 计算流程

`UpdateForcesAndMoments()` 是本版本的核心函数。其主要计算流程如下：

1. 读取 Gazebo 关节角速度，并通过 `rotor_velocity_slowdown_sim_` 还原真实风扇转速；
2. 获取 `base_link` 在世界坐标系下的线速度、角速度和姿态；
3. 根据世界风速计算相对风速，并转换到机体系；
4. 将 Gazebo ENU 机体系速度映射到 MATLAB 模型所采用的坐标定义；
5. 计算机翼迎角、涵道迎角及相关三角量；
6. 通过样条插值得到 `dt`、`dn`、`wl`、`wd`、`wm`；
7. 计算涵道推力、侧向力、俯仰力矩和反扭矩；
8. 计算机翼气动力和机翼俯仰力矩；
9. 计算诱导速度、非轴流阻尼力矩、涵道反扭矩和风扇陀螺力矩；
10. 合成 MATLAB 坐标系下的总气动力和总气动力矩；
11. 将力和力矩转换回 Gazebo ENU 局部坐标系；
12. 使用 Gazebo 接口将合力和合力矩施加到 `base_link`；
13. 更新电机转速滤波结果，并设置关节速度；
14. 输出调试信息，用于检查各中间变量和合力 / 合力矩结果。

### 7.1 坐标转换关系

本版本继续沿用 MATLAB 模型与 Gazebo ENU 局部坐标之间的转换关系。

| 物理量 | Gazebo ENU 机体系 | MATLAB 模型坐标系 |
|---|---|---|
| 侧向速度 | `v_enu` | `u_mat` |
| 前向速度 | `u_enu` | `v_mat` |
| 轴向速度 | `w_enu`，向上为正 | `w_mat = -w_enu`，向下为正 |
| 滚转角速度 | `q_enu` | `p_mat` |
| 俯仰角速度 | `p_enu` | `q_mat` |
| 偏航角速度 | `r_enu` | `r_mat = -r_enu` |

对应代码为：

```cpp
double u_mat = v_enu;   // MATLAB 侧向 x
double v_mat = u_enu;   // MATLAB 前向 y
double w_mat = -w_enu;  // MATLAB 轴向 z，向下为正

double p_mat = q_enu;   // MATLAB 滚转角速度
double q_mat = p_enu;   // MATLAB 俯仰角速度
double r_mat = -r_enu;  // MATLAB 偏航角速度
```

---

## 8. 力与力矩模型整理

### 8.1 涵道风扇力与力矩

涵道风扇部分主要计算以下四个量：

| 变量 | 含义 |
|---|---|
| `Ducted_T` | 涵道推力 |
| `Ducted_N` | 涵道侧向力 |
| `Ducted_M` | 涵道俯仰力矩 |
| `Ducted_Q` | 风扇反扭矩 |

核心计算代码如下：

```cpp
double Ducted_O2  = O * O;
double Ducted_VaO = Ducted_Va_body * O;

// 涵道推力
double Ducted_T = motor_constant_ * Ducted_O2
       + Ducted_VaO * (k_Th_ + k_Ts_ * Ducted_csAOA)
       + Ducted_Va_body * Ducted_Va_body * dt;

if (!reversible_) {
  Ducted_T = std::abs(Ducted_T);
}

// 涵道侧向力
double Ducted_N = Ducted_VaO * k_Ns_ * Ducted_sAOA
                + Ducted_Va_body * Ducted_Va_body * dn;

// 涵道俯仰力矩
double Ducted_M = 0.0;
if (O > 0.0) {
  Ducted_M = Ducted_N * l_cpz_
           + Ducted_T * k_cpx_ * Ducted_Va_body / O * Ducted_sAOA;
} else {
  Ducted_M = Ducted_N * l_cpz_;
}

// 风扇反扭矩
double Ducted_Q = moment_constant_ * Ducted_O2;
```

### 8.2 机翼力与力矩

机翼部分沿用上一版本的样条插值结果，主要计算 `Wing_Fy`、`Wing_Fz` 和 `Wing_My`。

```cpp
double wl = 0.0;
double wd = 0.0;
double wm = 0.0;

if (!wing_spline_wl_.empty()) wl = PpvalSpline(wing_spline_wl_, AOA, true);
if (!wing_spline_wd_.empty()) wd = PpvalSpline(wing_spline_wd_, AOA, true);
if (!wing_spline_wm_.empty()) wm = PpvalSpline(wing_spline_wm_, AOA, true);

double Vvw2 = Vvw_mat * Vvw_mat;
double Wing_Fy = Vvw2 * wl;   // MATLAB 前向 y 轴力
double Wing_Fz = Vvw2 * wd;   // MATLAB 轴向 z 轴力，向下为正
double Wing_My = Vvw2 * wm;   // MATLAB 侧向 x 轴俯仰力矩
```

### 8.3 总力合成

在 MATLAB 坐标系下，涵道侧向力、涵道推力和机翼气动力被合成为总气动力。

```cpp
double Ve = -w_mat / 2.0
          + std::sqrt((w_mat / 2.0) * (w_mat / 2.0)
          + Ducted_T / (duct_sd_ * air_density_ * duct_S_));

double F1 = -Ducted_N * Sduct;
double F2 = -Ducted_N * Cduct - Wing_Fy;
double F3 = -Ducted_T - Wing_Fz;

ignition::math::Vector3d F_mat(F1, F2, F3);
```

其中：

| 变量 | 含义 |
|---|---|
| `Ve` | 涵道诱导速度相关量 |
| `F1` | MATLAB 侧向 x 轴合力分量 |
| `F2` | MATLAB 前向 y 轴合力分量 |
| `F3` | MATLAB 轴向 z 轴合力分量，向下为正 |

### 8.4 总力矩合成

总力矩由涵道俯仰力矩、机翼俯仰力矩、非轴流阻尼力矩、风扇反扭矩、涵道反扭矩和风扇陀螺力矩共同组成。

```cpp
// 非轴流阻尼力矩
double M_my = -p_mat * (k_my0_ + k_myv_ * Vvw_mat);

// 风扇反扭矩
double M_fan = -turning_direction_ * Ducted_Q;

// 涵道反扭矩
double M_sta = turning_direction_ * k_sta_ * Ve * Ve;

// 风扇陀螺力矩
double M_gyro_x = -I_fan_ * O * q_mat;
double M_gyro_y =  I_fan_ * O * p_mat;
double M_gyro_z = 0.0;

// MATLAB 坐标系下的总力矩
double M1 = -Ducted_M * Cduct - Wing_My + M_my + M_gyro_x;
double M2 =  Ducted_M * Sduct + M_gyro_y;
double M3 =  M_fan + M_sta + M_gyro_z;

ignition::math::Vector3d M_mat(M1, M2, M3);
```

### 8.5 转换回 Gazebo ENU 坐标系并施加

将 MATLAB 坐标系下的总力和总力矩转换为 Gazebo ENU 局部坐标系后，统一施加到 `base_link`。

```cpp
ignition::math::Vector3d F_enu(F_mat.Y(), F_mat.X(), -F_mat.Z());
ignition::math::Vector3d M_enu(M_mat.Y(), M_mat.X(), -M_mat.Z());

if (base_link_) {
  base_link_->AddRelativeForce(F_enu);
  base_link_->AddRelativeTorque(M_enu);
}
```

该步骤是本版本相较于 `DuctedFanPlugin_02` 的关键推进：上一版本仅完成气动力与力矩的计算和打印，本版本已经将其接入 Gazebo 动力学求解过程。

---

## 9. 电机速度控制逻辑

力与力矩施加完成后，仍保留原始电机速度滤波和关节速度设定流程。该部分用于接收 PX4 / Gazebo 电机速度指令，并将滤波后的转速设置到旋翼关节上。

```cpp
double ref_motor_rot_vel;
ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
```

说明：

1. `rotor_velocity_filter_` 用于模拟电机转速响应的一阶动态特性；
2. `turning_direction_` 用于区分顺时针和逆时针旋转方向；
3. `rotor_velocity_slowdown_sim_` 用于降低 Gazebo 中的可视化旋转速度，并在力学计算中还原真实物理转速；
4. 当前代码中 PID 模式仍被屏蔽，实际采用直接设置关节速度的方式。

---

## 10. 调试输出与验证变量

当前版本保留 `DEBUG_PRINT` 调试输出，并每隔约 1 秒打印一次关键变量。

```cpp
const bool DEBUG_PRINT = true;
```

重点检查以下变量：

| 变量 | 检查目的 |
|---|---|
| `real_motor_velocity` | 判断真实风扇转速是否正确还原 |
| `AOA(deg)` | 判断机翼迎角是否合理 |
| `Ducted_AOA(deg)` | 判断涵道迎角是否合理 |
| `dt`, `dn` | 判断涵道样条插值是否正常 |
| `wl`, `wd`, `wm` | 判断机翼样条插值是否正常 |
| `Ducted_T` | 判断涵道推力是否随转速平方和来流变化 |
| `Ducted_N` | 判断侧向力是否随横向来流变化 |
| `Ducted_M` | 判断涵道俯仰力矩是否随侧向力和拉力中心偏移变化 |
| `Ducted_Q` | 判断风扇反扭矩是否随转速平方变化 |
| `Ve` | 判断诱导速度项是否存在异常值 |
| `M_my` | 判断非轴流阻尼力矩是否与角速度和速度相关 |
| `M_fan` | 判断风扇反扭矩方向是否与旋转方向一致 |
| `M_sta` | 判断涵道反扭矩项是否正常 |
| `F_enu` | 判断转换后的 Gazebo 局部合力方向是否合理 |
| `M_enu` | 判断转换后的 Gazebo 局部合力矩方向是否合理 |

---

## 11. 建议验证流程

### 11.1 静态悬停工况

目标：验证无明显来流时，涵道推力是否主要由 `motor_constant_ * O^2` 决定。(已完好)

检查重点：

1. `Ducted_T` 是否为主要力项；
2. `Ducted_N` 是否接近零或保持较小；
3. `F_enu` 的轴向分量方向是否正确；
4. 机体是否出现非预期横向漂移或滚转 / 俯仰响应。

## 12. 当前问题与后续工作

### 12.1 当前已完成

1. `DuctedFanPlugin` 已经完成对原 `gazebo_motor_model` 核心功能的替代；
2. 已经从单纯计算气动力 / 气动力矩推进到实际施加到 `base_link`；
3. 已经将涵道风扇、机翼、非轴流阻尼、涵道反扭矩和陀螺力矩纳入统一计算框架；
4. 已经建立 MATLAB 坐标系与 Gazebo ENU 局部坐标系之间的转换关系；
5. 已经保留电机速度滤波、转速还原和关节速度设置逻辑。

### 12.2 仍需完善

1. 补充尚未接入的 `M_cs` 力矩项；
2. 继续核对所有力和力矩方向，尤其是 `F_enu`、`M_enu` 与 Gazebo `AddRelativeForce()` / `AddRelativeTorque()` 的坐标含义；
3. 与原 `lift_drag` 插件进行功能对齐，明确哪些机翼气动项需要保留、替代或合并；
4. 将力 / 力矩计算函数进一步拆分为独立子函数，提高代码可读性和后续维护性。

---

## 13. 本次修改总结

`DuctedFanPlugin_03` 的核心目标是将上一版本中已经计算出的气动力和气动力矩真正接入 Gazebo 动力学仿真。本版本通过获取 `base_link`、补充涵道风扇模型参数、重构 `UpdateForcesAndMoments()` 计算流程、完成 MATLAB 坐标系到 Gazebo ENU 坐标系的转换，并使用 `AddRelativeForce()` 和 `AddRelativeTorque()` 施加合力与合力矩，使插件从“气动力计算模块”推进为“可参与动力学仿真的涵道风扇综合模型插件”。

从开发阶段看，本版本已经完成涵道风扇建模的主体闭环。下一阶段应重点完成 `lift_drag` 相关功能整合、`M_cs` 力矩项补充，以及悬停、前飞、侧风等典型工况下的系统性验证。

---

## 14. 核心函数完整记录

以下保留本版本 `UpdateForcesAndMoments()` 的核心实现，便于后续对照源码继续修改。

```cpp
// ======== 核心：更新作用在机体上的力和力矩 ========
void DuctedFanModel::UpdateForcesAndMoments() {
  // 获取当前关节角速度，单位 rad/s
  motor_rot_vel_ = joint_->GetVelocity(0);

  // 检查混叠风险。若仿真步长过大，高转速可能无法被准确捕捉
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_
          << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }

  // 将仿真关节速度还原为物理真实风扇转速
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double O = real_motor_velocity;

  // 获取机体基准连杆在世界坐标系下的线速度与姿态
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d body_velocity = base_link_->WorldLinearVel();
  ignition::math::Pose3d link_pose = base_link_->WorldPose();
#else
  ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(base_link_->GetWorldLinearVel());
  ignition::math::Pose3d link_pose = ignitionFromGazeboMath(base_link_->GetWorldPose());
#endif

  // 计算相对风速，并转换到机体系
  ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;
  ignition::math::Vector3d wind_body = link_pose.Rot().RotateVectorReverse(relative_wind_velocity);

  // 获取机体角速度，并转换到机体系
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d ang_vel_world = base_link_->WorldAngularVel();
#else
  ignition::math::Vector3d ang_vel_world = ignitionFromGazeboMath(base_link_->GetWorldAngularVel());
#endif
  ignition::math::Vector3d ang_vel = link_pose.Rot().RotateVectorReverse(ang_vel_world);

  // Gazebo ENU 机体系速度分量
  double u_enu = wind_body.X();
  double v_enu = wind_body.Y();
  double w_enu = wind_body.Z();

  double p_enu = ang_vel.X();
  double q_enu = ang_vel.Y();
  double r_enu = ang_vel.Z();

  // 转换为 MATLAB 模型坐标定义
  double u_mat = v_enu;
  double v_mat = u_enu;
  double w_mat = -w_enu;

  double p_mat = q_enu;
  double q_mat = p_enu;
  double r_mat = -r_enu;

  // 计算机翼迎角及平面速度分量
  double Vvw_mat = sqrt(v_mat * v_mat + w_mat * w_mat);
  double Vuv_mat = sqrt(u_mat * u_mat + v_mat * v_mat);
  const double Vuv_mat_min = 1e-12;

  double AOA = 0.0;
  if (Vvw_mat <= 1.0) {
    AOA = M_PI / 2.0;
  } else {
    double cosA = -w_mat / Vvw_mat;
    cosA = ignition::math::clamp(cosA, -1.0, 1.0);
    AOA = acos(cosA);
  }

  double Sduct = 0.0;
  double Cduct = 1.0;
  if (Vuv_mat > Vuv_mat_min) {
    Sduct = u_mat / Vuv_mat;
    Cduct = v_mat / Vuv_mat;
  }

  // 计算涵道迎角
  double Ducted_Vuv = sqrt(u_mat * u_mat + v_mat * v_mat);
  double Ducted_Va_body = sqrt(u_mat * u_mat + v_mat * v_mat + w_mat * w_mat);
  const double Ducted_Va_min = 1e-6;

  double Ducted_csAOA = 0.0;
  double Ducted_sAOA = 1.0;
  double Ducted_AOA = M_PI / 2.0;

  if (Ducted_Va_body <= Ducted_Va_min) {
    Ducted_AOA = M_PI / 2.0;
    Ducted_csAOA = 0.0;
    Ducted_sAOA = 1.0;
  } else {
    Ducted_csAOA = -w_mat / Ducted_Va_body;
    Ducted_csAOA = ignition::math::clamp(Ducted_csAOA, -1.0, 1.0);
    Ducted_AOA = acos(Ducted_csAOA);
    Ducted_sAOA = Ducted_Vuv / Ducted_Va_body;
  }

  // 涵道样条插值
  double dt = 0.0;
  double dn = 0.0;
  if (!spline_dt_.empty()) dt = PpvalSpline(spline_dt_, Ducted_AOA, true);
  if (!spline_dn_.empty()) dn = PpvalSpline(spline_dn_, Ducted_AOA, true);

  // 涵道力与力矩计算
  double Ducted_O2 = O * O;
  double Ducted_VaO = Ducted_Va_body * O;

  double Ducted_T = motor_constant_ * Ducted_O2
         + Ducted_VaO * (k_Th_ + k_Ts_ * Ducted_csAOA)
         + Ducted_Va_body * Ducted_Va_body * dt;

  if (!reversible_) {
    Ducted_T = std::abs(Ducted_T);
  }

  double Ducted_N = Ducted_VaO * k_Ns_ * Ducted_sAOA
                  + Ducted_Va_body * Ducted_Va_body * dn;

  double Ducted_M = 0.0;
  if (O > 0.0) {
    Ducted_M = Ducted_N * l_cpz_
             + Ducted_T * k_cpx_ * Ducted_Va_body / O * Ducted_sAOA;
  } else {
    Ducted_M = Ducted_N * l_cpz_;
  }

  double Ducted_Q = moment_constant_ * Ducted_O2;

  // 机翼样条插值与力矩计算
  double wl = 0.0;
  double wd = 0.0;
  double wm = 0.0;
  if (!wing_spline_wl_.empty()) wl = PpvalSpline(wing_spline_wl_, AOA, true);
  if (!wing_spline_wd_.empty()) wd = PpvalSpline(wing_spline_wd_, AOA, true);
  if (!wing_spline_wm_.empty()) wm = PpvalSpline(wing_spline_wm_, AOA, true);

  double Vvw2 = Vvw_mat * Vvw_mat;
  double Wing_Fy = Vvw2 * wl;
  double Wing_Fz = Vvw2 * wd;
  double Wing_My = Vvw2 * wm;

  // MATLAB 坐标系下的总气动力
  double Ve = -w_mat / 2.0
            + std::sqrt((w_mat / 2.0) * (w_mat / 2.0)
            + Ducted_T / (duct_sd_ * air_density_ * duct_S_));

  double F1 = -Ducted_N * Sduct;
  double F2 = -Ducted_N * Cduct - Wing_Fy;
  double F3 = -Ducted_T - Wing_Fz;
  ignition::math::Vector3d F_mat(F1, F2, F3);

  // MATLAB 坐标系下的总气动力矩
  double M_my = -p_mat * (k_my0_ + k_myv_ * Vvw_mat);
  double M_fan = -turning_direction_ * Ducted_Q;
  double M_sta = turning_direction_ * k_sta_ * Ve * Ve;

  double M_gyro_x = -I_fan_ * O * q_mat;
  double M_gyro_y =  I_fan_ * O * p_mat;
  double M_gyro_z = 0.0;

  double M1 = -Ducted_M * Cduct - Wing_My + M_my + M_gyro_x;
  double M2 =  Ducted_M * Sduct + M_gyro_y;
  double M3 =  M_fan + M_sta + M_gyro_z;
  ignition::math::Vector3d M_mat(M1, M2, M3);

  // 转换到 Gazebo ENU 局部坐标系
  ignition::math::Vector3d F_enu(F_mat.Y(), F_mat.X(), -F_mat.Z());
  ignition::math::Vector3d M_enu(M_mat.Y(), M_mat.X(), -M_mat.Z());

  // 将合力和合力矩施加到机体基准连杆
  if (base_link_) {
    base_link_->AddRelativeForce(F_enu);
    base_link_->AddRelativeTorque(M_enu);
  }

  // 电机速度滤波与关节速度设定
  double ref_motor_rot_vel;
  ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);

  // 调试输出
  const bool DEBUG_PRINT = true;
  if (DEBUG_PRINT) {
    static std::chrono::steady_clock::time_point last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count();

    if (elapsed_ms >= 1000) {
      std::cout << "--- 调试信息 [电机 " << motor_number_ << "] ---" << std::endl;
      std::cout << "真实风扇转速: " << real_motor_velocity << "\n"
                << "机翼迎角 deg: " << AOA * 180.0 / M_PI
                << " wl: " << wl << " wd: " << wd << " wm: " << wm << "\n"
                << "涵道迎角 deg: " << Ducted_AOA * 180.0 / M_PI
                << " dt: " << dt << " dn: " << dn << "\n"
                << "涵道推力: " << Ducted_T
                << " 涵道侧向力: " << Ducted_N
                << " 涵道俯仰力矩: " << Ducted_M
                << " 风扇反扭矩: " << Ducted_Q << "\n"
                << "诱导速度相关量 Ve: " << Ve << " m/s\n"
                << "p_mat: " << p_mat << " q_mat: " << q_mat << " r_mat: " << r_mat << "\n"
                << "非轴流阻尼力矩 M_my: " << M_my
                << " 风扇反扭矩 M_fan: " << M_fan
                << " 涵道反扭矩 M_sta: " << M_sta << "\n"
                << "Gazebo 局部总力矩: " << M_enu << "\n"
                << "Gazebo 局部总力: " << F_enu << std::endl;
      std::cout << "----------------------------------" << std::endl;

      last_print_time = current_time;
    }
  }
}
```
