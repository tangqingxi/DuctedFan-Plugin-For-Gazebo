# 🚀Gazebo 插件开发记录：DuctedFanPlugin_02

## 1. 开发目标

本文件记录 `DuctedFanPlugin_02` 的开发过程，目标是在 Gazebo 中实现 SHC09 机型涵道风扇的力与力矩计算。当前版本主要完成以下内容：

1. 将 MATLAB 中的涵道与机翼气动样条系数导出为 `.csv` 文件；
2. 在 SDF 中配置涵道风扇相关气动参数与样条文件路径；
3. 在 Gazebo 插件中实现 MATLAB `ppval()` 的等价计算函数；
4. 在 `DuctedFanPlugin` 中读取样条系数，并计算涵道风扇与机翼相关的力和力矩；
5. 通过调试打印验证计算结果。

> 当前状态：力与力矩已经完成计算与打印，但暂未正式施加到模型机刚体上。

---

## 2. 文件与目录规划

| 模块 | 文件 / 目录 | 作用 |
|---|---|---|
| MATLAB 数据处理 | `dt_dn_wl_wd_wm.m` | 将 `.mat` 原始气动数据转换为样条系数 `.csv` 文件 |
| 样条系数文件 | `Tools/sitl_gazebo/models/SHC09/*.csv` | 存放涵道与机翼气动样条系数 |
| SDF 配置 | SHC09 对应 SDF 文件 | 配置插件参数、样条文件路径和电机参数 |
| 样条读取与插值 | `spline_ppval.h / spline_ppval.cpp` | 实现 CSV 读取与 `ppval()` 等价计算 |
| 插件主逻辑 | `ductedfan_plugin.h / ductedfan_plugin.cpp` | 加载参数、计算气动力与气动力矩 |
| 编译配置 | `Tools/sitl_gazebo/CMakeLists.txt` | 将新增源文件加入插件编译 |

---

## 3. MATLAB 样条系数导出

### 3.1 气动系数来源

涵道与机翼气动系数由 `.mat` 文件提供，并通过 MATLAB 的 `spline()` 函数进行三次样条拟合。随后使用 `unmkpp()` 提取分段多项式系数，并导出为 `.csv` 文件，供 Gazebo 插件读取。

涉及的气动系数包括：

| 系数 | 含义 |
|---|---|
| `dt` | 涵道拉力系数 |
| `dn` | 涵道侧向力系数 |
| `wl` | 机翼升力系数 |
| `wd` | 机翼侧向力 / 阻力相关系数 |
| `wm` | 机翼俯仰力矩系数 |

### 3.2 MATLAB 导出脚本

创建 `dt_dn_wl_wd_wm.m`，用于将 MATLAB 样条对象转换为 CSV 文件。

```matlab
% 1. 加载 duct、wing 原始数据
data1 = load('duct_raw.mat');
duct_raw = data1.duct_raw;
clear data1;

data2 = load('wing_raw.mat');
wing_raw = data2.wing_raw;
clear data2;

% 2. 三次样条拟合
dt = spline(duct_raw(:,1), duct_raw(:,2));  % 涵道拉力系数
dn = spline(duct_raw(:,1), duct_raw(:,3));  % 涵道侧向力系数

wl = spline(wing_raw(:,1), wing_raw(:,2));  % 机翼升力系数
wd = spline(wing_raw(:,1), wing_raw(:,3));  % 机翼侧向力 / 阻力相关系数
wm = spline(wing_raw(:,1), wing_raw(:,4));  % 机翼俯仰力矩系数

% 3. 导出 dt
[breaks_dt, coefs_dt] = unmkpp(dt);
data_dt = [breaks_dt(1:end-1).', breaks_dt(2:end).', coefs_dt];
writematrix(data_dt, 'duct_spline_dt.csv');

% 4. 导出 dn
[breaks_dn, coefs_dn] = unmkpp(dn);
data_dn = [breaks_dn(1:end-1).', breaks_dn(2:end).', coefs_dn];
writematrix(data_dn, 'duct_spline_dn.csv');

% 5. 导出 wl
[breaks_wl, coefs_wl] = unmkpp(wl);
data_wl = [breaks_wl(1:end-1).', breaks_wl(2:end).', coefs_wl];
writematrix(data_wl, 'wing_spline_wl.csv');

% 6. 导出 wd
[breaks_wd, coefs_wd] = unmkpp(wd);
data_wd = [breaks_wd(1:end-1).', breaks_wd(2:end).', coefs_wd];
writematrix(data_wd, 'wing_spline_wd.csv');

% 7. 导出 wm
[breaks_wm, coefs_wm] = unmkpp(wm);
data_wm = [breaks_wm(1:end-1).', breaks_wm(2:end).', coefs_wm];
writematrix(data_wm, 'wing_spline_wm.csv');
```

### 3.3 CSV 文件放置位置

导出的 `.csv` 文件放置在以下目录：

```text
Tools/sitl_gazebo/models/SHC09/
```

文件包括：

```text
duct_spline_dt.csv
duct_spline_dn.csv
wing_spline_wl.csv
wing_spline_wd.csv
wing_spline_wm.csv
```

---

## 4. SDF 插件配置

在 SHC09 的 SDF 文件中添加 `DuctedFanPlugin` 配置。该部分包含电机基础参数、涵道风扇气动参数、样条文件路径以及电机控制话题。

```bash
<plugin name='rotor_ductedfan_plugin' filename='libDuctedFanPlugin.so'>
  <robotNamespace></robotNamespace>
  <jointName>rotor_joint</jointName>
  <linkName>rotor</linkName>
  <turningDirection>ccw</turningDirection>
  <timeConstantUp>0.0125</timeConstantUp>
  <timeConstantDown>0.025</timeConstantDown>
  <maxRotVelocity>2450</maxRotVelocity>

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

  <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
  <motorNumber>0</motorNumber>

  <!-- 当前版本不启用原始旋翼空气阻力项 -->
  <rotorDragCoefficient>0</rotorDragCoefficient>
  <rollingMomentCoefficient>0</rollingMomentCoefficient>

  <!-- 涵道气动系数样条文件路径 -->
  <ductSplineDtFile>model://SHC09/duct_spline_dt.csv</ductSplineDtFile>
  <ductSplineDnFile>model://SHC09/duct_spline_dn.csv</ductSplineDnFile>

  <!-- 机翼气动系数样条文件路径 -->
  <wingSplineWlFile>model://SHC09/wing_spline_wl.csv</wingSplineWlFile>
  <wingSplineWdFile>model://SHC09/wing_spline_wd.csv</wingSplineWdFile>
  <wingSplineWmFile>model://SHC09/wing_spline_wm.csv</wingSplineWmFile>

  <motorSpeedPubTopic>/motor_speed/0</motorSpeedPubTopic>
  <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
  <robotNamespace></robotNamespace>
</plugin>
```

---

## 5. 实现 MATLAB `ppval()` 等价功能

### 5.1 创建 `spline_ppval.cpp`

在以下目录中创建源文件：

```text
Tools/sitl_gazebo/src/ductedfan_plugin/spline_ppval.cpp
```

```cpp
#include "ductedfan_plugin/spline_ppval.h"

#include <gazebo/common/Console.hh>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace gazebo
{

bool LoadSplineFromCsv(const std::string& file_path,
                       std::vector<SplineSegment>& spline_data)
{
  std::ifstream file(file_path.c_str());

  if (!file.is_open())
  {
    gzerr << "[SplinePPVal] 无法打开样条系数文件: "
          << file_path << "\n";
    return false;
  }

  spline_data.clear();

  std::string line;
  int line_index = 0;

  while (std::getline(file, line))
  {
    line_index++;

    if (line.empty())
    {
      continue;
    }

    std::replace(line.begin(), line.end(), ',', ' ');

    std::stringstream ss(line);

    SplineSegment segment;

    if (!(ss >> segment.x_left
             >> segment.x_right
             >> segment.a
             >> segment.b
             >> segment.c
             >> segment.d))
    {
      gzerr << "[SplinePPVal] 第 " << line_index
            << " 行无法解析: " << line << "\n";
      return false;
    }

    spline_data.push_back(segment);
  }

  if (spline_data.empty())
  {
    gzerr << "[SplinePPVal] 样条系数为空: "
          << file_path << "\n";
    return false;
  }

  std::sort(spline_data.begin(), spline_data.end(),
            [](const SplineSegment& lhs, const SplineSegment& rhs)
            {
              return lhs.x_left < rhs.x_left;
            });

  gzmsg << "[SplinePPVal] 成功读取样条区间数量: "
        << spline_data.size() << "\n";

  return true;
}

double PpvalSpline(const std::vector<SplineSegment>& spline_data,
                   double x,
                   bool clamp_input)
{
  if (spline_data.empty())
  {
    gzerr << "[SplinePPVal] 样条数据为空，返回 0\n";
    return 0.0;
  }

  double x_eval = x;

  if (clamp_input)
  {
    if (x_eval < spline_data.front().x_left)
    {
      x_eval = spline_data.front().x_left;
    }

    if (x_eval > spline_data.back().x_right)
    {
      x_eval = spline_data.back().x_right;
    }
  }

  const SplineSegment* selected_segment = nullptr;

  for (const auto& segment : spline_data)
  {
    if (x_eval >= segment.x_left && x_eval <= segment.x_right)
    {
      selected_segment = &segment;
      break;
    }
  }

  if (selected_segment == nullptr)
  {
    if (x_eval < spline_data.front().x_left)
    {
      selected_segment = &spline_data.front();
    }
    else
    {
      selected_segment = &spline_data.back();
    }
  }

  const double dx = x_eval - selected_segment->x_left;

  const double y =
      selected_segment->a * dx * dx * dx
    + selected_segment->b * dx * dx
    + selected_segment->c * dx
    + selected_segment->d;

  return y;
}

}
```

### 5.2 创建 `spline_ppval.h`

在以下目录中创建头文件：

```text
Tools/sitl_gazebo/include/ductedfan_plugin/spline_ppval.h
```

```cpp
#pragma once

#include <string>
#include <vector>

namespace gazebo
{

struct SplineSegment
{
  double x_left;
  double x_right;
  double a;
  double b;
  double c;
  double d;
};

// 从 CSV 文件读取 MATLAB 导出的 pp 系数。
bool LoadSplineFromCsv(const std::string& file_path,
                       std::vector<SplineSegment>& spline_data);

// 实现 MATLAB ppval(pp, x) 的等价计算。
double PpvalSpline(const std::vector<SplineSegment>& spline_data,
                   double x,
                   bool clamp_input = false);

}
```

---

## 6. 修改 `ductedfan_plugin.h`

### 6.1 读取新增 SDF 参数

在插件参数读取位置加入以下内容：

```cpp
getSdfParam<double>(_sdf, "thrustVelocityCoupling", k_Th_, 0.0);
getSdfParam<double>(_sdf, "thrustAoaCoupling", k_Ts_, 0.0);
getSdfParam<double>(_sdf, "sideForceVelocityCoupling", k_Ns_, 0.0);
getSdfParam<double>(_sdf, "sideForceArmZ", l_cpz_, 0.0);
getSdfParam<double>(_sdf, "thrustCenterOffsetCoeff", k_cpx_, 0.0);
```

### 6.2 引入样条工具头文件

在 `#include "common.h"` 之后加入：

```cpp
#include <vector>
#include "ductedfan_plugin/spline_ppval.h"
```

### 6.3 添加样条数据成员

在 `private:` 区域中添加以下成员变量，例如放在 `bool fly;` 附近：

```cpp
std::vector<SplineSegment> spline_dt_;       // 涵道拉力系数样条
std::vector<SplineSegment> spline_dn_;       // 涵道侧向力系数样条

std::vector<SplineSegment> wing_spline_wl_;  // 机翼升力系数样条
std::vector<SplineSegment> wing_spline_wd_;  // 机翼侧向力 / 阻力相关系数样条
std::vector<SplineSegment> wing_spline_wm_;  // 机翼俯仰力矩系数样条
```

---

## 7. 修改 `ductedfan_plugin.cpp`

### 7.1 在 `Load()` 函数中加载样条文件

在 `Load()` 函数末尾部分插入以下代码，例如放在 `if (_sdf->HasElement("reversible")) { ... }` 之后。

```cpp
// ---------- 加载涵道气动样条系数（dt, dn） ----------
if (_sdf->HasElement("ductSplineDtFile")) {
    std::string dt_path = _sdf->GetElement("ductSplineDtFile")->Get<std::string>();
    if (!LoadSplineFromCsv(dt_path, spline_dt_)) {
        gzerr << "[ductedfan_plugin] 加载涵道拉力系数样条失败，文件: " << dt_path << "\n";
    }
} else {
    gzerr << "[ductedfan_plugin] 未指定 ductSplineDtFile，将不使用样条修正推力\n";
}

if (_sdf->HasElement("ductSplineDnFile")) {
    std::string dn_path = _sdf->GetElement("ductSplineDnFile")->Get<std::string>();
    if (!LoadSplineFromCsv(dn_path, spline_dn_)) {
        gzerr << "[ductedfan_plugin] 加载涵道侧向力系数样条失败，文件: " << dn_path << "\n";
    }
} else {
    gzerr << "[ductedfan_plugin] 未指定 ductSplineDnFile，侧向力将不采用样条\n";
}

// ---------- 加载机翼气动样条系数（wl, wd, wm） ----------
if (_sdf->HasElement("wingSplineWlFile")) {
    std::string wl_path = _sdf->GetElement("wingSplineWlFile")->Get<std::string>();
    if (!LoadSplineFromCsv(wl_path, wing_spline_wl_))
        gzerr << "[ductedfan_plugin] 加载机翼升力样条失败！\n";
} else {
    gzerr << "[ductedfan_plugin] 未指定 wingSplineWlFile\n";
}

if (_sdf->HasElement("wingSplineWdFile")) {
    std::string wd_path = _sdf->GetElement("wingSplineWdFile")->Get<std::string>();
    if (!LoadSplineFromCsv(wd_path, wing_spline_wd_))
        gzerr << "[ductedfan_plugin] 加载机翼阻力样条失败！\n";
} else {
    gzerr << "[ductedfan_plugin] 未指定 wingSplineWdFile\n";
}

if (_sdf->HasElement("wingSplineWmFile")) {
    std::string wm_path = _sdf->GetElement("wingSplineWmFile")->Get<std::string>();
    if (!LoadSplineFromCsv(wm_path, wing_spline_wm_))
        gzerr << "[ductedfan_plugin] 加载机翼俯仰力矩样条失败！\n";
} else {
    gzerr << "[ductedfan_plugin] 未指定 wingSplineWmFile\n";
}
```

### 7.2 修改 `UpdateForcesAndMoments()` 函数

`UpdateForcesAndMoments()` 是本次修改的核心函数。该函数主要完成以下步骤：

1. 获取电机角速度，并通过 `rotor_velocity_slowdown_sim_` 还原真实转速；
2. 计算机体系下的相对风速；
3. 将 Gazebo 坐标系下的速度分量转换为 MATLAB 坐标系；
4. 计算机翼迎角与涵道风扇迎角；
5. 通过样条插值得到 `dt`、`dn`、`wl`、`wd`、`wm`；
6. 计算涵道推力、侧向力、俯仰力矩、反扭矩；
7. 计算机翼相关气动力与俯仰力矩；
8. 打印调试信息，用于验证计算结果。

```cpp
void DuctedFanModel::UpdateForcesAndMoments() {
  // 获取当前关节角速度（rad/s）
  motor_rot_vel_ = joint_->GetVelocity(0);

  // 检查混叠风险：如果仿真步长过大，高转速可能无法被准确捕捉。
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_
          << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }

  // 将仿真中的低速旋转映射回物理真实转速。
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double O = real_motor_velocity;

  // 获取旋翼刚体在世界坐标系下的线速度。
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d body_velocity = link_->WorldLinearVel();
  ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
  ignition::math::Pose3d link_pose = link_->WorldPose();
#else
  ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(link_->GetWorldLinearVel());
  ignition::math::Vector3d joint_axis = ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
  ignition::math::Pose3d link_pose = ignitionFromGazeboMath(link_->GetWorldPose());
#endif

  // 相对风速 = 机体速度 - 世界风速。
  ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;

  // 转换到机体系，即旋翼坐标系下的风速。
  ignition::math::Vector3d wind_body = link_pose.Rot().RotateVectorReverse(relative_wind_velocity);

  double u_enu = wind_body.X();   // Gazebo 机体前向 x
  double v_enu = wind_body.Y();   // Gazebo 机体侧向 y
  double w_enu = wind_body.Z();   // Gazebo 机体轴向 z，向上为正

  // 转换为 MATLAB 坐标系：x→v，y→u，z→-w。
  double u_mat = v_enu;   // MATLAB 侧向 x
  double v_mat = u_enu;   // MATLAB 前向 y
  double w_mat = -w_enu;  // MATLAB 轴向 z，向下为正

  // ---------- 机翼迎角计算 ----------
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

  // ================= 涵道风扇力与力矩计算 =================
  // 输入：转速 O，来流速度分量 u_mat、v_mat、w_mat。
  // 输出：推力 Ducted_T、侧向力 Ducted_N、俯仰力矩 Ducted_M、反扭矩 Ducted_Q。

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

  // 通过样条插值获取涵道气动系数。
  double dt = 0.0;
  double dn = 0.0;
  if (!spline_dt_.empty()) dt = PpvalSpline(spline_dt_, Ducted_AOA, true);
  if (!spline_dn_.empty()) dn = PpvalSpline(spline_dn_, Ducted_AOA, true);

  double Ducted_O2 = O * O;
  double Ducted_VaO = Ducted_Va_body * O;

  // 涵道推力 T。
  double Ducted_T = motor_constant_ * Ducted_O2
                  + Ducted_VaO * (k_Th_ + k_Ts_ * Ducted_csAOA)
                  + Ducted_Va_body * Ducted_Va_body * dt;

  if (!reversible_) {
    Ducted_T = std::abs(Ducted_T);
  }

  // 涵道侧向力 N。
  double Ducted_N = Ducted_VaO * k_Ns_ * Ducted_sAOA
                  + Ducted_Va_body * Ducted_Va_body * dn;

  // 涵道俯仰力矩 M。
  double Ducted_M = 0.0;
  if (O > 0.0) {
    Ducted_M = Ducted_N * l_cpz_
             + Ducted_T * k_cpx_ * Ducted_Va_body / O * Ducted_sAOA;
  } else {
    Ducted_M = Ducted_N * l_cpz_;
  }

  // 反扭矩 Q。
  double Ducted_Q = moment_constant_ * Ducted_O2;

  // ================= 机翼力与力矩计算 =================
  double wl = 0.0;
  double wd = 0.0;
  double wm = 0.0;

  if (!wing_spline_wl_.empty()) wl = PpvalSpline(wing_spline_wl_, AOA, true);
  if (!wing_spline_wd_.empty()) wd = PpvalSpline(wing_spline_wd_, AOA, true);
  if (!wing_spline_wm_.empty()) wm = PpvalSpline(wing_spline_wm_, AOA, true);

  double Vvw2 = Vvw_mat * Vvw_mat;
  double Wing_Fy = Vvw2 * wl;   // MATLAB 前向 y 轴分量
  double Wing_Fz = Vvw2 * wd;   // MATLAB 轴向 z 轴分量，向下为正
  double Wing_My = Vvw2 * wm;   // 绕 MATLAB 侧向 x 轴的俯仰力矩

  // ================= 调试打印 =================
  const bool DEBUG_PRINT = true;

  if (DEBUG_PRINT) {
    static std::chrono::steady_clock::time_point last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();

    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count();

    if (elapsed_ms >= 1000) {
      std::cout << "--- Debug Info [Motor " << motor_number_ << "] ---" << std::endl;
      std::cout << "motor_rot_vel_: " << motor_rot_vel_ << "\n"
                << "real_motor_velocity: " << real_motor_velocity << "\n"
                << "AOA(deg): " << AOA * 180.0 / M_PI
                << " wl:" << wl << " wd:" << wd << " wm:" << wm << "\n"
                << "Ducted_AOA(deg): " << Ducted_AOA * 180.0 / M_PI
                << " dt:" << dt << " dn:" << dn << "\n"
                << "Ducted_T: " << Ducted_T << " Ducted_N: " << Ducted_N
                << " Ducted_M: " << Ducted_M << " Ducted_Q: " << Ducted_Q << "\n"
                << "Wing_Fy: " << Wing_Fy << " Wing_Fz: " << Wing_Fz
                << " Wing_My: " << Wing_My << std::endl;
      std::cout << "----------------------------------" << std::endl;

      last_print_time = current_time;
    }
  }
}
```

---

## 8. 修改 `CMakeLists.txt`

### 8.1 原始写法

```cmake
add_library(DuctedFanPlugin SHARED src/ductedfan_plugin/ductedfan_plugin.cpp)
```

### 8.2 修改后写法

将新增的 `spline_ppval.cpp` 加入编译：

```cmake
add_library(DuctedFanPlugin SHARED
  src/ductedfan_plugin/ductedfan_plugin.cpp
  src/ductedfan_plugin/spline_ppval.cpp
)
```

---

## 9. 调试与验证要点

### 9.1 样条文件读取验证

启动 Gazebo 后，应在终端中检查是否出现以下类似信息：

```text
[SplinePPVal] 成功读取样条区间数量: ...
```

如果出现文件无法打开或样条为空的报错，优先检查：

1. `.csv` 文件是否已经放入 `Tools/sitl_gazebo/models/SHC09/`；
2. SDF 中的 `model://SHC09/...` 路径是否正确；
3. CSV 是否为纯数值格式，且每行包含 6 列：`x_left, x_right, a, b, c, d`。

### 9.2 力与力矩计算验证

当前版本通过 `DEBUG_PRINT` 每隔约 1 秒输出一次计算结果。重点检查以下变量：

| 变量 | 检查目的 |
|---|---|
| `real_motor_velocity` | 判断真实转速是否由 `rotor_velocity_slowdown_sim_` 正确还原 |
| `AOA(deg)` | 判断机翼迎角计算是否合理 |
| `Ducted_AOA(deg)` | 判断涵道风扇迎角计算是否合理 |
| `dt`, `dn` | 判断涵道样条插值是否正常 |
| `wl`, `wd`, `wm` | 判断机翼样条插值是否正常 |
| `Ducted_T` | 判断涵道推力是否与转速、来流速度变化一致 |
| `Ducted_N` | 判断侧向力是否随横向来流变化 |
| `Ducted_M` | 判断俯仰力矩是否受侧向力与拉力中心偏移影响 |
| `Ducted_Q` | 判断反扭矩是否随转速平方变化 |
| `Wing_Fy`, `Wing_Fz`, `Wing_My` | 判断机翼气动力和俯仰力矩计算是否正常 |

### 9.3 后续需要完成的工作

当前代码仅计算并打印气动力和气动力矩。后续若要使其真正作用于模型，需要进一步完成：

1. 明确涵道推力、侧向力、机翼力在 Gazebo 坐标系下的方向；
2. 将 MATLAB 坐标系下的力与力矩转换回 Gazebo / link 坐标系；
3. 使用 Gazebo 接口将力和力矩施加到对应 link 上；
4. 在静态悬停、前飞、侧风等工况下分别验证力和姿态响应；
5. 关闭或降低 `DEBUG_PRINT` 频率，避免长时间仿真时终端输出过多。

---

## 10. 本次修改总结

本版本完成了从 MATLAB 样条数据到 Gazebo 插件内部气动力计算的完整链路。核心改动包括：

- 使用 MATLAB 导出涵道与机翼气动样条系数；
- 在 SDF 中新增涵道风扇气动参数和样条文件路径；
- 新增 `spline_ppval` 工具模块，实现 CSV 读取与 `ppval()` 等价计算；
- 在 `DuctedFanPlugin` 中加载气动样条，并计算涵道风扇推力、侧向力、俯仰力矩和反扭矩；
- 增加机翼气动力与俯仰力矩计算；
- 将新增源文件加入 CMake 编译流程。

整体上，该版本已经具备涵道风扇气动计算能力，下一步重点是完成力和力矩到 Gazebo 模型的实际施加与仿真验证。
