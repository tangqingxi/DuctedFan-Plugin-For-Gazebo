# 关于我们的Ducted-FAN插件
原生 `PX4 + Gazebo` 仿真环境中，固定翼飞机的左右机翼、水平尾翼、垂直尾翼均采用 `libLiftDragPlugin.so` 插件，螺旋桨则使用 `libgazebo_motor_model.so` 插件，两者分别作用于对应的 link 上。

我们计划开发一个新的插件，将上述两个插件的功能整合在一起，并对其数学计算公式进行修正与补充。这些公式来源于本实验室的论文。

`DuctedFanPlugin` 是用于 PX4-Gazebo 仿真的涵道风扇动力学插件，主要面向 SHC09 / SHW09 类涵道风扇无人机模型。插件用于替代原有 `gazebo_motor_model` 与部分 `LiftDragPlugin` 功能，在同一插件中完成电机转速响应、涵道风扇气动力、机翼气动力、反扭矩、陀螺力矩以及控制舵面力矩的统一计算与施加。

更详细的开发过程、模型推导和代码修改记录见：

```text
ductedfan_plugin_01.md
ductedfan_plugin_02.md
ductedfan_plugin_03.md
ductedfan_plugin_04.md
ductedfan_plugin_05.md
```

---

## 1. 环境要求(跳转到https://github.com/mengchaoheng/DuctedFanUAV-Autopilot搭建环境)
- 对于Ubuntu 20.04，安装模拟环境相当简单：
- 安装git：
```bash
sudo apt install git
```

- 克隆代码(注意分支切换到DF-1.12.3)：
```bash
git clone https://github.com/mengchaoheng/DuctedFanUAV-Autopilot --recursive
```

- 请前往代码路径：
```bash
cd DuctedFanUAV-Autopilot
```

- 运行无参数的 ubuntu.sh 来安装所有东西
```bash
# For arm64-based ubuntu. See https://github.com/PX4/PX4-Autopilot/issues/21117
bash ./Tools/setup/ubuntu.sh
```
- 测试本项目的模拟：
```bash
make px4_sitl gazebo_SHC09
```
- 注：在Ubuntu 22.04及更高版本中，Gazebo Classic不再支持arm64 Ubuntu。如果Gazebo是用脚本在amd64 Ubuntu上安装的，需要卸载并重新安装：
```bash
sudo apt remove gz-harmonic
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```
- 一定要确保`make px4_sitl gazebo_SHC09`是有效的。
---

## 2. 下载与放置

下载项目源码：

```bash
git clone https://github.com/tqx2023/DuctedFan-Plugin-For-Gazebo.git
cd DuctedFan-Plugin-For-Gazebo
```
`DuctedFan-Plugin-For-Gazebo`内有Tools文件夹，按照以下路径，放置在`DuctedFanUAV-Autopilot`的对应文件夹内。
插件源码`ductedfan_plugin.cpp`，`spline_ppval.cpp`应放置在：

```text
DuctedFanUAV-Autopilot/Tools/sitl_gazebo/src/ductedfan_plugin/
```

头文件`ductedfan_plugin.h`，`spline_ppval.h`应放置在：

```text
DuctedFanUAV-Autopilot/Tools/sitl_gazebo/include/ductedfan_plugin/
```

气动样条系数文件应放置在对应模型目录下，例如：

```text
DuctedFanUAV-Autopilot/Tools/sitl_gazebo/models/SHC09/
```
气动样条系数,涵道样条系数文件包括：

```text
duct_spline_dt.csv
duct_spline_dn.csv
wing_spline_wl.csv
wing_spline_wd.csv
wing_spline_wm.csv
```
---

## 3. 编译插件

在 `Tools/sitl_gazebo/CMakeLists.txt` 中加入插件编译目标：

```cmake
add_library(DuctedFanPlugin SHARED
  src/ductedfan_plugin/ductedfan_plugin.cpp
  src/ductedfan_plugin/spline_ppval.cpp
)

list(APPEND plugins DuctedFanPlugin)
```

然后在 PX4 工程根目录重新编译仿真环境：

```bash
cd ~/DuctedFanUAV-Autopilot
make px4_sitl gazebo_SHC09
```
编译成功后会生成：

```text
libDuctedFanPlugin.so
```
- (也可以选择)
- `DuctedFan-Plugin-For-Gazebo\Tools\sitl_gazebo`内有一个版对应的CMakeLists.txt放置在对应目录下
```text
DuctedFanUAV-Autopilot\Tools\sitl_gazebo
```
---

## 4. SDF 配置

在模型的 `.sdf.jinja` 文件中加载插件。插件必须放在 `<model>` 标签内部，不要放入 `<link>` 标签内部。

```bash
    <plugin name='rotor_ductedfan_plugin' filename='libDuctedFanPlugin.so'>
      <robotNamespace></robotNamespace>
      <jointName>rotor_joint</jointName>
      <linkName>rotor</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>2450</maxRotVelocity>
      
      <!-- k_T0 悬停时的拉力系数 -->
      <motorConstant>2.8964e-05</motorConstant>
      <!-- k_Th 推力‑速度耦合项  跟C_T_J0相关 -->
      <thrustVelocityCoupling>-3.7614e-04</thrustVelocityCoupling>
      <!-- k_Ts 推力‑速度‑迎角项 跟C_T_J相关  拉力随a_df的衰减系数 -->
      <thrustAoaCoupling>-2.6733e-04</thrustAoaCoupling>
      <!-- k_Ns 侧向力‑速度‑迎角项 %跟C_N_J相关  侧向力力随a_df的衰减系数 -->
      <sideForceVelocityCoupling>5.0699e-04</sideForceVelocityCoupling>
      <!-- k_Q0 跟C_Q相关    风扇扭矩系数 -->
      <momentConstant>5.7792e-07</momentConstant>
      <!-- l_cpz 跟l_N相关    侧向力N_df对机体中心的力臂 -->
      <sideForceArmZ>-0.15</sideForceArmZ>
      <!-- k_cpx 跟l_T相关    拉力中心的法向偏移 -->
      <thrustCenterOffsetCoeff>2.6619</thrustCenterOffsetCoeff>
      <!-- 涵道扩压比 sd -->
      <ductExpansionRatio>0.7</ductExpansionRatio>
      <!-- 桨盘面积 S (m^2)，示例值根据半径 0.114m 计算 -->
      <ductDiskArea>0.0408</ductDiskArea>
      <!-- 空气密度 den (kg/m^3) -->
      <airDensity>1.225</airDensity>
      <!-- k_my0 俯仰阻尼-->
      <pitchDampingConstant>0.0198</pitchDampingConstant>
      <!-- k_myv -->      
      <pitchDampingVelocityCoeff>0.0015</pitchDampingVelocityCoeff> 
      <!-- 涵道反扭矩系数 k_sta -->
      <ductTorqueCoeff>5.3688e-04</ductTorqueCoeff>   
      <!-- 风扇转动惯量 I_fan (kg·m²) -->
      <fanInertia>3.7e-05</fanInertia>

      <!-- 控制舵面 -->
      <control_joint_name_1>CS1_joint</control_joint_name_1>
      <control_joint_name_2>CS2_joint</control_joint_name_2>
      <control_joint_name_3>CS3_joint</control_joint_name_3>
      <control_joint_name_4>CS4_joint</control_joint_name_4>
      <control_joint_name_5>CS5_joint</control_joint_name_5>
      <control_joint_name_6>CS6_joint</control_joint_name_6>

      <!-- 控制舵面力矩模型参数 -->
      <control_surface_force_coeff>0.0032</control_surface_force_coeff>
      <control_surface_arm_l1>0.267</control_surface_arm_l1>
      <control_surface_arm_l2>0.066</control_surface_arm_l2>

      <!-- 控制效能矩阵B for M_cs -->
      <control_effectiveness_matrix>
        0.0   -0.8660254  -0.8660254   0.0   0.8660254  0.8660254
       -1.0     -0.5        0.5        1.0   0.5        -0.5
        1.0      1.0        1.0        1.0   1.0        1.0
      </control_effectiveness_matrix>                   

      <!-- The Drag moment generated by the rotation of the rotor (the same as T) -->
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <!-- Air Drag Force H acting on rotor due to relative wind vel-->
      <rotorDragCoefficient>0</rotorDragCoefficient>
      <!-- Rolling moment due to air drag Force H -->
      <rollingMomentCoefficient>0</rollingMomentCoefficient>

      <!-- 新增：涵道气动系数样条文件路径 -->
      <ductSplineDtFile>model://SHC09/duct_spline_dt.csv</ductSplineDtFile>
      <ductSplineDnFile>model://SHC09/duct_spline_dn.csv</ductSplineDnFile>

      <!-- 新增：机翼气动系数样条文件路径 -->
      <wingSplineWlFile>model://SHC09/wing_spline_wl.csv</wingSplineWlFile>
      <wingSplineWdFile>model://SHC09/wing_spline_wd.csv</wingSplineWdFile>
      <wingSplineWmFile>model://SHC09/wing_spline_wm.csv</wingSplineWmFile>


      <motorSpeedPubTopic>/motor_speed/0</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <robotNamespace></robotNamespace>
    </plugin>
```

如果模型原来已经给六个控制舵面配置了 `LiftDragPlugin`，需要删除或禁用，避免控制舵面力矩重复施加。

修改 `.sdf.jinja` 后，建议删除旧的自动生成 `.sdf` 文件，再重新编译运行。

---

## 5. 运行与验证

启动仿真：

```bash
cd ~/DuctedFanUAV-Autopilot
make px4_sitl gazebo_SHC09
```

启动后检查终端输出。正常情况下应能看到插件加载、样条文件读取、控制舵面关节读取等信息。

重点检查：

```text
libDuctedFanPlugin.so 是否成功加载
样条 CSV 是否成功读取
base_link 是否成功获取
CS1_joint 到 CS6_joint 是否成功读取
飞行器是否能完成基本悬停或姿态响应
```

若需要调试内部变量，可在源码中打开 `DEBUG_PRINT`，观察转速、迎角、推力、力矩、控制舵面角度和 `M_cs` 等变量。

---

## 6. 常见问题

### 插件没有加载

检查 SDF 中的文件名是否正确：

```xml
filename="libDuctedFanPlugin.so"
```

同时确认 `CMakeLists.txt` 中已经添加 `DuctedFanPlugin` 编译目标。

### 修改 `.sdf.jinja` 后没有生效

删除旧的 `.sdf` 文件后重新编译：

```bash
make px4_sitl gazebo_SHC09
```

### 样条文件读取失败

检查 CSV 是否位于模型目录下，并确认 SDF 路径是否使用正确的 `model://` 格式。

### 飞行器出现异常自旋

优先检查：

```text
control_effectiveness_matrix
CS1 到 CS6 的舵面编号映射
舵面转角正方向
M_cs 的符号和量级
AddRelativeTorque() 的坐标方向
```

---

## 7. 开发记录索引

| 文件 | 内容 |
|---|---|
| `ductedfan_plugin_01.md` | HelloWorld 插件创建、CMake 配置、SDF 加载与最小闭环验证 |
| `ductedfan_plugin_02.md` | 替代 `gazebo_motor_model`，完成基础电机模型、推力、反扭矩和悬停验证 |
| `ductedfan_plugin_03.md` | 引入 MATLAB 样条系数，完成涵道和机翼气动力 / 力矩计算 |
| `ductedfan_plugin_04.md` | 将合力与合力矩统一施加到 `base_link`，补充诱导速度、阻尼、反扭矩和陀螺力矩 |
| `ductedfan_plugin_05.md` | 接入控制舵面力矩 `M_cs`，完成第一版综合动力学插件 |
