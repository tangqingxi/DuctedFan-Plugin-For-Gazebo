# Gazebo 插件开发记录：DuctedFanPlugin_05

## 1. 版本定位与前序工作衔接

`DuctedFanPlugin_05` 是在 `DuctedFanPlugin_04` 基础上的进一步扩展版本。前一版本已经完成涵道风扇、机翼气动力、非轴流阻尼、涵道反扭矩和风扇陀螺力矩的统一计算，并将合力与合力矩通过 `AddRelativeForce()` 和 `AddRelativeTorque()` 施加到 `base_link` 上。因此，`05` 版本的核心工作不再是重新构建整体气动力模型，而是在已有总力矩框架中补充控制舵面力矩项 `M_cs`，使插件进一步替代原先由多个 `LiftDragPlugin` 分散实现的控制舵面气动作用。

本版本的主要目标如下：

1. 删除原 SDF 文件中用于六个控制舵面的 `LiftDragPlugin` 配置，避免控制舵面力矩被重复计算；
2. 在 `DuctedFanPlugin` 中集中读取六个控制舵面关节角度；
3. 在 SDF 中配置控制舵面力矩模型参数和控制效能矩阵 `B_cs`；
4. 根据 MATLAB / 论文模型计算控制舵面力矩 `M_cs`；
5. 将 `M_cs` 合入 `DuctedFanPlugin_04` 已建立的总力矩计算流程，并统一施加到 `base_link`。

本版本完成后，`DuctedFanPlugin` 已经形成第一版较完整的涵道风扇综合动力学插件：电机转速响应、涵道风扇气动力、机翼气动力、风扇反扭矩、涵道反扭矩、陀螺力矩以及控制舵面力矩均在同一插件内完成计算和施加。

---

## 2. SDF 配置修改

### 2.1 删除原控制舵面 `LiftDragPlugin`

在 `SHC09.sdf.jinja` 中，删除原先分别作用于六个控制舵面的 `LiftDragPlugin` 配置。删除该部分的原因是：本版本已经将控制舵面力矩统一纳入 `DuctedFanPlugin` 中计算，如果继续保留原 `LiftDragPlugin`，会导致控制舵面的气动力或力矩被重复施加，从而破坏动力学模型的一致性。

### 2.2 增加控制舵面关节名称

在 `rotor_ductedfan_plugin` 插件块中，在风扇转动惯量参数之后增加六个控制舵面关节名称。该配置用于插件在 `Load()` 阶段获取对应关节指针，并在每个仿真步读取当前舵面偏转角。

```bash
<!-- 风扇转动惯量 I_fan，单位 kg·m^2 -->
<fanInertia>3.7e-05</fanInertia>

<!-- 控制舵面关节名称 -->
<control_joint_name_1>CS1_joint</control_joint_name_1>
<control_joint_name_2>CS2_joint</control_joint_name_2>
<control_joint_name_3>CS3_joint</control_joint_name_3>
<control_joint_name_4>CS4_joint</control_joint_name_4>
<control_joint_name_5>CS5_joint</control_joint_name_5>
<control_joint_name_6>CS6_joint</control_joint_name_6>
```

### 2.3 增加控制舵面力矩模型参数

控制舵面力矩模型采用如下形式：

$$
\bm M_{cs}=d_{cs} V_e^2 \operatorname{diag}(l_1,l_1,l_2)\bm B_{cs}\bm c_{cs}
$$

其中，`d_cs` 为单个控制舵面的力系数，`l1` 和 `l2` 为力臂参数，`V_e` 为前一版本中已经计算得到的诱导速度相关量，`B_cs` 为控制效能矩阵，`c_cs` 为控制舵面偏转角向量。

对应的 SDF 配置如下：

```bash
<!-- 控制舵面力矩模型参数 -->
<control_surface_force_coeff>0.0032</control_surface_force_coeff>
<control_surface_arm_l1>0.267</control_surface_arm_l1>
<control_surface_arm_l2>0.066</control_surface_arm_l2>
```

### 2.4 配置控制效能矩阵 `B_cs`

控制效能矩阵 `B_cs` 按照 MATLAB 模型中的坐标系和舵面编号定义写入 SDF 文件。后续在 C++ 中不修改矩阵本身，而是通过重新排列 Gazebo 中读取到的舵面角度，使输入向量 `c_cs` 与 MATLAB 模型保持一致。

```bash
<!-- 控制效能矩阵 B_cs，用于计算 M_cs -->
<control_effectiveness_matrix>
  0.0   -0.8660254  -0.8660254   0.0   0.8660254   0.8660254
 -1.0   -0.5         0.5         1.0   0.5        -0.5
  1.0    1.0         1.0         1.0   1.0         1.0
</control_effectiveness_matrix>
```

需要注意的是，`B_cs` 的三行分别对应 MATLAB 坐标系下的三个力矩分量。因此，后续计算得到的 `M_cs` 也应首先作为 MATLAB 坐标系下的力矩项，再随总力矩一起转换到 Gazebo 机体系。

---

## 3. 头文件与数据结构修改

### 3.1 增加头文件依赖

由于本版本需要保存控制舵面关节名称，因此在 `ductedfan_plugin.h` 中增加标准库字符串头文件。

```cpp
#include <string>
```

如果当前头文件中尚未包含 Eigen 相关头文件，则需要确保已经引入：

```cpp
#include <Eigen/Dense>
```

### 3.2 构造函数初始化

在 `DuctedFanModel()` 构造函数中，对控制舵面相关容器和控制效能矩阵进行初始化。这里保留 1 到 6 的舵面编号习惯，因此向量长度设置为 `kControlJointCount + 1`，下标 0 不参与实际使用。

```cpp
DuctedFanModel()
    : reversible_(false),
      control_joint_names_(kControlJointCount + 1),
      control_joints_(kControlJointCount + 1),
      control_joint_angles_(kControlJointCount + 1, 0.0),
      B_cs_(Eigen::Matrix<double, 3, 6>::Zero()) {
}
```

### 3.3 增加成员变量与成员函数声明

在 `private` 区域增加控制舵面相关成员变量。该部分用于保存舵面关节名称、关节指针、实时偏转角、控制舵面力矩参数以及控制效能矩阵。

```cpp
// 控制舵面数量，编号采用 1 到 6
static const int kControlJointCount = 6;

// 控制舵面关节名称，使用下标 1 到 6
std::vector<std::string> control_joint_names_;

// 控制舵面关节指针，使用下标 1 到 6
std::vector<physics::JointPtr> control_joints_;

// 控制舵面当前偏转角，单位 rad，使用下标 1 到 6
std::vector<double> control_joint_angles_;

// 从 SDF 中读取控制舵面关节名称，并获取关节指针
void LoadControlJoints(sdf::ElementPtr _sdf);

// 读取当前控制舵面角度
void ReadControlJointAngles();

// 从 SDF 中读取控制效能矩阵 B_cs_
void LoadControlEffectivenessMatrix(sdf::ElementPtr _sdf);

// 单个控制舵面的力系数
double d_cs_;

// 控制舵面力臂参数 l1
double l1_cs_;

// 控制舵面力臂参数 l2
double l2_cs_;

// 控制效能矩阵，维度为 3 x 6
Eigen::Matrix<double, 3, 6> B_cs_;
```

该设计的主要优点是将控制舵面相关逻辑集中管理，避免在 `UpdateForcesAndMoments()` 中直接写大量 SDF 读取代码。同时，后续如果需要增加舵面诊断打印、舵面限幅检查或舵面角度符号修正，也可以在现有结构上继续扩展。

---

## 4. 源文件实现与控制舵面力矩计算

### 4.1 在 `Load()` 函数中加载控制舵面信息

在 `ductedfan_plugin.cpp` 的 `Load()` 函数中，增加控制舵面关节、控制效能矩阵和控制舵面力矩参数的读取逻辑。

```cpp
// 读取 1 到 6 号控制舵面关节
LoadControlJoints(_sdf);

// 读取控制舵面效能矩阵 B_cs_
LoadControlEffectivenessMatrix(_sdf);

// 读取控制舵面力矩模型参数
getSdfParam<double>(_sdf, "control_surface_force_coeff", d_cs_, 0.0);
getSdfParam<double>(_sdf, "control_surface_arm_l1", l1_cs_, 0.0);
getSdfParam<double>(_sdf, "control_surface_arm_l2", l2_cs_, 0.0);
```

其中，`getSdfParam` 的默认值设置为 `0.0`。这样即使 SDF 中暂时缺少某一参数，插件也不会直接崩溃，但对应的控制舵面力矩项会自然退化为零，便于调试定位。

### 4.2 读取控制舵面关节

`LoadControlJoints()` 用于从 SDF 文件中读取 `control_joint_name_1` 到 `control_joint_name_6`，并通过 `model_->GetJoint()` 获取对应的 Gazebo 关节指针。

```cpp
void DuctedFanModel::LoadControlJoints(sdf::ElementPtr _sdf)
{
  for (int i = 1; i <= kControlJointCount; ++i) {
    std::string sdf_tag = "control_joint_name_" + std::to_string(i);

    if (!_sdf->HasElement(sdf_tag)) {
      gzerr << "[ductedfan_plugin] 未设置 " << sdf_tag
            << "，CS" << i << " 角度默认为 0。\n";
      continue;
    }

    std::string joint_name = _sdf->Get<std::string>(sdf_tag);
    physics::JointPtr joint = model_->GetJoint(joint_name);

    if (!joint) {
      gzerr << "[ductedfan_plugin] 无法找到控制舵面关节: "
            << joint_name << "\n";
      continue;
    }

    control_joint_names_[i] = joint_name;
    control_joints_[i] = joint;
    control_joint_angles_[i] = 0.0;

    gzmsg << "[ductedfan_plugin] 已加载控制舵面关节 CS"
          << i << ": " << joint_name << "\n";
  }
}
```

### 4.3 读取控制舵面角度

`ReadControlJointAngles()` 用于在每个仿真步读取六个舵面的当前偏转角。Gazebo 9 及以上版本使用 `Position(0)`，早期版本使用 `GetAngle(0).Radian()`。两种方式得到的角度单位均为弧度。

```cpp
void DuctedFanModel::ReadControlJointAngles()
{
  for (int i = 1; i <= kControlJointCount; ++i) {
    if (!control_joints_[i]) {
      control_joint_angles_[i] = 0.0;
      continue;
    }

#if GAZEBO_MAJOR_VERSION >= 9
    control_joint_angles_[i] = control_joints_[i]->Position(0);
#else
    control_joint_angles_[i] = control_joints_[i]->GetAngle(0).Radian();
#endif
  }
}
```

### 4.4 读取控制效能矩阵

`LoadControlEffectivenessMatrix()` 从 SDF 中读取 `control_effectiveness_matrix` 字符串，并解析为 18 个浮点数，再按行填充到 `3 x 6` 的 Eigen 矩阵中。

```cpp
void DuctedFanModel::LoadControlEffectivenessMatrix(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("control_effectiveness_matrix")) {
    gzerr << "[ductedfan_plugin] 未设置 control_effectiveness_matrix，B_cs_ 使用零矩阵。\n";
    B_cs_.setZero();
    return;
  }

  std::string matrix_str =
      _sdf->GetElement("control_effectiveness_matrix")->Get<std::string>();
  std::stringstream ss(matrix_str);

  std::vector<double> values;
  double value = 0.0;

  while (ss >> value) {
    values.push_back(value);
  }

  if (values.size() != 18) {
    gzerr << "[ductedfan_plugin] control_effectiveness_matrix 参数数量错误，应为 18 个，实际为 "
          << values.size() << " 个。B_cs_ 使用零矩阵。\n";
    B_cs_.setZero();
    return;
  }

  int index = 0;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 6; ++col) {
      B_cs_(row, col) = values[index];
      ++index;
    }
  }

  gzmsg << "[ductedfan_plugin] 控制效能矩阵 B_cs_ 读取完成：\n"
        << B_cs_ << "\n";
}
```

### 4.5 在 `UpdateForcesAndMoments()` 中计算 `M_cs`

在每个仿真步中，首先读取六个控制舵面的当前角度。

```cpp
// 读取 1 到 6 号控制舵面当前角度，单位 rad
ReadControlJointAngles();
```

随后将 Gazebo 中的舵面编号映射到 MATLAB 模型中的舵面编号。当前模型中，Gazebo 俯视图下的 CS1 到 CS6 按顺时针排列，而 MATLAB 模型中以相同机头方向为参考时，CS1 到 CS6 按逆时针排列。因此，需要对舵面角度向量进行重排，而不直接修改 SDF 中的 `B_cs`。

```cpp
// Gazebo 中读取的控制舵面角度，单位 rad
double CS1_gazebo = control_joint_angles_[1];
double CS2_gazebo = control_joint_angles_[2];
double CS3_gazebo = control_joint_angles_[3];
double CS4_gazebo = control_joint_angles_[4];
double CS5_gazebo = control_joint_angles_[5];
double CS6_gazebo = control_joint_angles_[6];

// 将 Gazebo 舵面编号重排为 MATLAB 模型中的舵面编号
double CS1_mat = CS1_gazebo;
double CS2_mat = CS6_gazebo;
double CS3_mat = CS5_gazebo;
double CS4_mat = CS4_gazebo;
double CS5_mat = CS3_gazebo;
double CS6_mat = CS2_gazebo;

// 构造 MATLAB 坐标系下的控制舵面偏转角向量
Eigen::Matrix<double, 6, 1> c_cs;
c_cs << CS1_mat,
        CS2_mat,
        CS3_mat,
        CS4_mat,
        CS5_mat,
        CS6_mat;
```

然后构造力臂矩阵，并按照控制舵面力矩模型计算 `M_cs`。

```cpp
// 控制舵面力臂矩阵 diag([l1, l1, l2])
Eigen::Matrix3d L_cs = Eigen::Matrix3d::Zero();
L_cs(0, 0) = l1_cs_;
L_cs(1, 1) = l1_cs_;
L_cs(2, 2) = l2_cs_;

// 控制舵面力矩项：M_cs = d_cs * Ve^2 * diag([l1,l1,l2]) * B_cs * c_cs
Eigen::Matrix<double, 3, 1> M_cs =
    d_cs_ * Ve * Ve * L_cs * B_cs_ * c_cs;
```

最后，将 `M_cs` 合入前一版本已经建立的 MATLAB 坐标系总力矩中。

```cpp
// MATLAB 坐标系下的总力矩
double M1 = -Ducted_M * Cduct - Wing_My + M_my + M_gyro_x + M_cs(0);
double M2 =  Ducted_M * Sduct + M_gyro_y + M_cs(1);
double M3 =  M_fan + M_sta + M_gyro_z + M_cs(2);

ignition::math::Vector3d M_mat(M1, M2, M3);
```

由于 `M_mat` 仍然是 MATLAB 坐标系下的力矩，因此继续沿用 `04` 版本中的坐标转换关系，将其转换为 Gazebo 机体系下的相对力矩。

```cpp
// 转换到 Gazebo body 坐标系
ignition::math::Vector3d F_gazebo_body(F_mat.Y(), F_mat.X(), -F_mat.Z());
ignition::math::Vector3d M_gazebo_body(M_mat.Y(), M_mat.X(), -M_mat.Z());

if (base_link_) {
  base_link_->AddRelativeForce(F_gazebo_body);
  base_link_->AddRelativeTorque(M_gazebo_body);
}
```

这一处理保持了 `DuctedFanPlugin_04` 中“先在 MATLAB 坐标系下完成模型计算，再统一转换到 Gazebo 机体系施加”的总体架构，避免在每个子模型内部单独处理坐标转换。

---

## 5. 验证状态、技术意义与后续工作

### 5.1 当前验证状态

完成上述修改后，`DuctedFanPlugin` 已经在 Gazebo 中通过基础运行验证。当前版本可以正常读取控制舵面角度，并将控制舵面力矩项 `M_cs` 合入总力矩后施加到 `base_link`。这说明插件已经具备集中式控制舵面力矩建模能力，可以在后续仿真中替代原先分散布置的六个 `LiftDragPlugin`。

### 5.2 本版本的技术意义

本版本的关键意义在于：将控制舵面的气动力矩从“多个插件分别施加”改为“统一模型集中计算”。这种结构更适合后续进行动力学模型校核、控制分配验证和 MATLAB / Gazebo 模型一致性检查。

具体体现在以下方面：

1. 控制舵面力矩与涵道风扇、机翼、反扭矩和陀螺力矩在同一坐标框架下合成；
2. 控制效能矩阵 `B_cs` 直接沿用 MATLAB 模型定义，减少重复推导和矩阵改写；
3. 通过舵面角度重排解决 Gazebo 与 MATLAB 编号方向不一致的问题；
4. 所有气动力和气动力矩最终统一施加到 `base_link`，便于分析整机动力学响应；
5. 插件结构更清晰，为后续进一步拆分函数、封装子模型和验证控制分配算法提供基础。

### 5.3 后续需要重点检查的问题

虽然 `M_cs` 已经完成接入，但仍需围绕模型方向、数值量级和舵面映射关系继续验证：

1. 检查 `CS1` 到 `CS6` 的 Gazebo 实际转角正方向是否与 MATLAB 控制输入符号一致；
2. 检查 `B_cs` 三行对应的力矩方向是否与 `M_mat(M1, M2, M3)` 的定义一致；
3. 检查 `M_cs(2)` 对偏航响应的影响，避免由于符号或量级错误导致机体自旋；
4. 检查 `Ve^2` 项在低速、悬停和大转速工况下是否存在异常放大；
5. 在单舵面偏转、对称舵面偏转和组合舵面偏转工况下分别验证力矩方向；
6. 在悬停、姿态保持和指令跟踪仿真中观察 `M_cs` 对控制响应的影响。

### 5.4 建议的调试输出

为便于后续验证，建议在调试阶段增加以下变量打印：

```cpp
std::cout << "CS_mat: "
          << CS1_mat << ", "
          << CS2_mat << ", "
          << CS3_mat << ", "
          << CS4_mat << ", "
          << CS5_mat << ", "
          << CS6_mat << "\n"
          << "M_cs: " << M_cs.transpose() << "\n"
          << "M_gazebo_body: " << M_gazebo_body << std::endl;
```

其中，`CS_mat` 用于检查舵面编号重排是否正确，`M_cs` 用于检查控制舵面力矩量级，`M_gazebo_body` 用于检查最终施加到 Gazebo 机体系下的力矩方向。

### 5.5 本版本总结

`DuctedFanPlugin_05` 在 `04` 版本已经完成整机气动力和气动力矩施加的基础上，进一步补充了控制舵面力矩 `M_cs`。本版本通过 SDF 参数扩展、控制舵面关节读取、控制效能矩阵解析、舵面编号重排和力矩合成，实现了控制舵面力矩从 MATLAB 模型到 Gazebo 插件的接入。

至此，`DuctedFanPlugin` 已经具备第一版完整建模能力，可以作为 SHC09 涵道风扇飞行器在 Gazebo 中的综合动力学插件使用。后续工作应重点围绕舵面符号、力矩方向、控制分配响应和典型飞行工况验证继续开展。
