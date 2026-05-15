# 🚀学习Gazebo插件写法 
## Ducted-FAN_plugin ---HelloWorld版本
### 新建插件源码文件
- 要先在项目文件下建立我们的`ductedfan_plugin`文件夹
```bash
cd ~/DuctedFanUAV-Autopilot/Tools/sitl_gazebo/src
mkdir ductedfan_plugin
code ductedfan_plugin/ductedfan_plugin.cpp
```
- 目录结构变成：
```bash
DuctedFanUAV-Autopilot/
└── Tools/
    └── sitl_gazebo/
        └── src/
            └── ductedfan_plugin/
                └── ductedfan_plugin.cpp
```
### ductedfan_plugin.cpp 内容 (HELLOWORLD代码作为示例代码。实际代码在本项目，plugins/ductedfan_plugin.cpp)
```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class DuctedFanPlugin : public ModelPlugin
  {
    public: DuctedFanPlugin() : ModelPlugin()
    {
      printf("DuctedFanPlugin constructor called.\n");
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      printf("Hello World from DuctedFanPlugin!\n");

      this->model = _model;
    }

    private: physics::ModelPtr model;
  };

  GZ_REGISTER_MODEL_PLUGIN(DuctedFanPlugin)
}
```

### 修改CMakeLists.txt
- 打开
```bash
code ~/DuctedFanUAV-Autopilot/Tools/sitl_gazebo/CMakeLists.txt
```
![代码插入位置说明](/image/ductedfan_plugin_01.png)
- 真正创建 `.so` 的是 `add_library(...)` ，编译后生成的库文件是：`libDuctedFanPlugin.so`。后面 SDF 里必须写这个名字
```bash
foreach(plugin ${plugins})
  target_link_libraries(${plugin} ...)
endforeach()
```
- 给已经存在`plugin`的 `target` 链接库
- 在`target`链接库上面加入
```bash
add_library(DuctedFanPlugin SHARED src/ductedfan_plugin/ductedfan_plugin.cpp)
list(APPEND plugins DuctedFanPlugin)

foreach(plugin ${plugins})
  target_link_libraries(${plugin} ...)
endforeach()
```

### 在模型 SDF 中加载插件
- 找到你要测试的飞机模型 SDF。
- 例如
```bash
cd ~/DuctedFanUAV-Autopilot/Tools/sitl_gazebo/models/SHW09/SHW09.sdf.jinjia
```
更改了`sdf.jinjia`文件 , 原先编译出来的`.sdf`文件要删除掉。
- 在 `<model>` 标签内部加入：
```bash
<model name="ductedfan">
  ...

  <plugin name="ductedfan_plugin" filename="libDuctedFanPlugin.so">
  </plugin>

</model>
```
![HELLOWORLD成功打印](/image/ductedfan_plugin_02.png)
- 注意：
```bash
<plugin> 要放在 <model> 里面
不要放进 <link> 里面
```

### 重新编译
```bash
cd ~/DuctedFanUAV-Autopilot
make px4_sitl gazebo_SHW09
```

### 最小闭环验证成功
![HELLOWORLD成功打印](/image/ductedfan_plugin_03.png)