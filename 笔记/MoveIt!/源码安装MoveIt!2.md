# 源码安装 MoveIt!2

### 1. 参考教程 [Humble, Jazzy -stable]

- [鱼香ROS社区《动手学MoveIt2》](https://fishros.org.cn/forum/topic/395/%E5%8A%A8%E6%89%8B%E5%AD%A6moveit2-3-%E5%AE%89%E8%A3%85moveit2%E6%B5%8B%E8%AF%95)
- [MoveIt2官方教程](https://moveit.ai/install-moveit2/source/)
- [TRAC-IK安装](https://blog.csdn.net/joyopirate/article/details/130880705?spm=1001.2014.3001.5502)

### 2. 源码安装Moveit!2

- 创建 colcon 工作区

  ```bash
  export COLCON_WS=~/ws_moveit2/
  mkdir -p $COLCON_WS/src
  cd $COLCON_WS/src
  ```
- 下载源码到工作区

  - Humble, Jazzy-stable

    ```bash
    git clone https://github.com/moveit/moveit2.git -b $ROS_DISTRO
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```
  - Rolling, Jazzy, Humble-unstable

    ```bash
    git clone https://github.com/moveit/moveit2.git -b main
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- 从源码构建`MoveIt2`

  ```bash
  cd $COLCON_WS
  colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

  - `--event-handlers`: colcon的一个选项，用于指定**事件处理器（event handlers）**
  - `desktop_notification-`: 表示**禁用桌面通知**（默认在构建完成时会弹出桌面通知）
  - `status-`: 表示**禁用终端构建显示**（默认会显示每个包的构建进度，建议打开）

    ⚠️注意：`desktop_notification-`和`status-`后的`-`表示**禁用**，`+`表示**启用**
  - `--cmake-args`: 表示把后面的参数直接传给**CMake**
  - `-DCMAKE_BUILD_TYPE=Release`: **CMake** 参数，用来指定**构建类型为 Release**

    - `Release`: 优化过的可执行文件，运行速度快，但没有调试信息
    - `Debug`: 带有调试信息，方便调试，但是运行速度慢
    - `RelWithDebInfo`: 带优化同时保留调试信息
    - `MinSizeRel`: 优化体积最小

- `source`工作空间

  ```bash
  # bash
  source $COLCON_WS/install/setup.bash

  # zsh
  source $COLCON_WS/install/setup.zsh
  ```

### 3. 源码安装TRAC-IK

- 下载并安装`NLopt library`

  - [NLopt Documentation](https://nlopt.readthedocs.io/en/latest/#download-and-installation)

- 下载`TRAC-IK`源码

  ```bash
  git clone -b ros2_plugin https://github.com/ravnicas/trac_ik.git
  ```

- 将下载的源码放到项目所在的工作空间中的`src`文件夹

  ```bash
  cd colcon_ws/src
  ```

- 编译源码

  ```bash
  colcon build
  ```

  - 报错问题

    ```bash
    --- stderr: trac_ik_kinematics trac_ik_kinematics: You did not request a specific build type: Choosing 'Release' for maximum performance In file included from /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/include/trac_ik_kinematics_plugin/trac_ik_kinematics_plugin.h:45, from /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:47: /home/tong/ws_moveit2/install/moveit_core/include/moveit_core/moveit/kinematics_base/kinematics_base.h:50:77: note: ‘#pragma message: .h header is obsolete. Please use the .hpp header instead.’ 50 | ".h header is obsolete. Please use the .hpp header instead.") | ^ In file included from /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/include/trac_ik_kinematics_plugin/trac_ik_kinematics_plugin.h:47, from /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:47: /home/tong/ws_moveit2/install/moveit_core/include/moveit_core/moveit/robot_model/robot_model.h:51:77: note: ‘#pragma message: .h header is obsolete. Please use the .hpp header instead.’ 51 | ".h header is obsolete. Please use the .hpp header instead.") | ^ In file included from /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/include/trac_ik_kinematics_plugin/trac_ik_kinematics_plugin.h:48, from /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:47: /home/tong/ws_moveit2/install/moveit_core/include/moveit_core/moveit/robot_state/robot_state.h:51:77: note: ‘#pragma message: .h header is obsolete. Please use the .hpp header instead.’ 51 | ".h header is obsolete. Please use the .hpp header instead.") | ^ /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp: In member function ‘virtual bool trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin::initialize(const SharedPtr&, const moveit::core::RobotModel&, const string&, const string&, const std::vector<std::__cxx11::basic_string<char> >&, double)’: /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:151:5: error: ‘lookupParam’ was not declared in this scope 151 | lookupParam(node_, "position_only_ik", position_ik_, false); | ^~~~~~~~~~~ /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp: In member function ‘int trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin::getKDLSegmentIndex(const string&) const’: /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:161:43: warning: use of old-style cast to ‘int’ [-Wold-style-cast] 161 | while (i < (int)chain.getNrOfSegments()) | ^ | ----- | static_cast<int> ( ) In file included from /opt/ros/humble/include/rclcpp/rclcpp/logging.hpp:24, from /opt/ros/humble/include/rclcpp/rclcpp/client.hpp:40, from /opt/ros/humble/include/rclcpp/rclcpp/callback_group.hpp:24, from /opt/ros/humble/include/rclcpp/rclcpp/any_executable.hpp:20, from /opt/ros/humble/include/rclcpp/rclcpp/memory_strategy.hpp:25, from /opt/ros/humble/include/rclcpp/rclcpp/memory_strategies.hpp:18, from /opt/ros/humble/include/rclcpp/rclcpp/executor_options.hpp:20, from /opt/ros/humble/include/rclcpp/rclcpp/executor.hpp:37, from /opt/ros/humble/include/rclcpp/rclcpp/executors/multi_threaded_executor.hpp:25, from /opt/ros/humble/include/rclcpp/rclcpp/executors.hpp:21, from /opt/ros/humble/include/rclcpp/rclcpp/rclcpp.hpp:155, from /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:32: /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp: In member function ‘bool trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin::searchPositionIK(const Pose&, const std::vector<double, std::allocator<double> >&, double, std::vector<double, std::allocator<double> >&, const IKCallbackFn&, moveit_msgs::msg::MoveItErrorCodes&, const std::vector<double, std::allocator<double> >&, const kinematics::KinematicsQueryOptions&) const’: /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:397:32: warning: format ‘%i’ expects argument of type ‘int’, but argument 5 has type ‘moveit_msgs::msg::MoveItErrorCodes’ {aka ‘moveit_msgs::msg::MoveItErrorCodes_<std::allocator<void> >’} [-Wformat=] 397 | RCLCPP_DEBUG(LOGGER, "Solution has error code %i", error_code); | ^~~~~~~~~~~~~~~~~~~~~~~~~~~~ /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:397:58: note: format string is defined here 397 | RCLCPP_DEBUG(LOGGER, "Solution has error code %i", error_code); | ~^ | | | int /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:320:77: warning: unused parameter ‘consistency_limits’ [-Wunused-parameter] 320 | const std::vector<double> &consistency_limits, | ~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~ /home/tong/colcon_ws/src/trac_ik_kinematics/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp:321:92: warning: unused parameter ‘options’ [-Wunused-parameter] 321 | const kinematics::KinematicsQueryOptions &options) const | ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~ gmake[2]: *** [trac_ik_kinematics_plugin/CMakeFiles/moveit_trac_ik_kinematics_plugin.dir/build.make:76：trac_ik_kinematics_plugin/CMakeFiles/moveit_trac_ik_kinematics_plugin.dir/src/trac_ik_kinematics_plugin.cpp.o] 错误 1 gmake[1]: *** [CMakeFiles/Makefile2:152：trac_ik_kinematics_plugin/CMakeFiles/moveit_trac_ik_kinematics_plugin.dir/all] 错误 2 gmake: *** [Makefile:146：all] 错误 2 --- Failed <<< trac_ik_kinematics [5.16s, exited with code 2]
    ```

    - 解决方法

      修改`trac_ik_kinematics_plugin.cpp`文件：

      找到：

      ```cpp
      lookupParam(node_, "position_only_ik", position_ik_, false);
      lookupParam(node_, "solve_type", position_ik_, false);
      ```

      修改为：

      ```cpp
      node_->get_parameter_or("position_only_ik", position_ik_, false);
      node_->get_parameter_or("solve_type", position_ik_, false);
      ```

      ⚙️检查插件声明文件

      确保`trac_ik_kinematics_plugin_description.xml`中插件名称与配置一致

      ```xml
      <library path="moveit_trac_ik_kinematics_plugin">
        <class name="trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin" type="trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin" base_class_type="kinematics::KinematicsBase">
          <description>
            A implementation of kinematics as a plugin based on TRAC-IK.
          </description>
        </class>
      </library>
      ```
  - 再次编译即可

    ```bash
    colcon build
    ```
- 修改`kinematics.yaml`文件

  ```yaml
  manipulator:
    kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.005
    goal_joint_tolerance: 0.0001
    goal_position_tolerance: 0.0001
    goal_orientation_tolerance: 0.001
  ```