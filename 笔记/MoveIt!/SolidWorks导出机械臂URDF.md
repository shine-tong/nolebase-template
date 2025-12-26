# SolidWorks导出机械臂URDF

### 1. 插件和工具

- 插件

  - [SolidWorks to URDF Exporter](https://wiki.ros.org/sw_urdf_exporter)
- 工具

  - [网页版模型预览工具](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/)
  - [VScode 预览拓展](https://marketplace.visualstudio.com/items?itemName=morningfrog.urdf-visualizer)
  - [VScode URDF拓展](https://marketplace.visualstudio.com/items?itemName=smilerobotics.urdf)
  - [在线单位换算器](https://cn.unithelper.com/)

### 2. 配置基准轴和基准点

- 配置基准轴

  - `基准轴1`

    - ![image](../../images/tools/image9.png)
  - `基准轴2`

    - ![image](../../images/tools/image10.png)
  - `基准轴3`

    - ![image](../../images/tools/image11.png)
  - `基准轴4`

    - ![image](../../images/tools/image12.png)
  - `基准轴5`

    - ![image](../../images/tools/image13.png)
  - `基准轴6`

    - ![image](../../images/tools/image14.png)
- 配置基准点

  - 基准点`base_point`

    - 注意：每个机器人的`base_point`需要根据厂家提供的信息来确定
    - 辅助面`基准面1`

      ![image](../../images/tools/image15.png)
    - `base_point`

      ![image](../../images/tools/image16.png)
  - 基准点`point1`

    - 辅助面`基准面2`

      ![image](../../images/tools/image17.png)
    - `point1`

      ![image](../../images/tools/image18.png)
  - 基准点`point2`

    - 辅助面`基准面3`

      ![image](../../images/tools/image19.png)
    - `point2`

      ![image](../../images/tools/image20.png)
  - 基准点`point3`

    - ![image](../../images/tools/image21.png)
  - 基准点`point456`

    - 因`J4`、`J5`和`J6`的基准点重合，故只创建一个共用基准点`point456`
    - `point456`

      ![image](../../images/tools/image22.png)

### 3. 配置参考坐标系

- 在配置参考坐标系时统一要求`+Z`沿各关节基准轴
- 矩阵`R=[x,y,z]`表示与`SolidWorks`自动生成坐标系的旋转关系
- 配置参考坐标系

  - 参考坐标系`base_point`

    `R=[-90, 0, 90]`

    ![image](../../images/tools/image23.png)
  - 参考坐标系`frame1`

    `R=[-90, 0, 90]`

    ![image](../../images/tools/image24.png)
  - 参考坐标系`frame2`

    `R=[90, 90, 0]`

    ![image](../../images/tools/image25.png)
  - 参考坐标系`frame3`

    `R=[90, 90, 0]`

    ![image](../../images/tools/image26.png)
  - 参考坐标系`frame4`

    `R=[180, 0, -90]`

    ![image](../../images/tools/image27.png)
  - 参考坐标系`frame5`

    `R=[0, -90, 0]`

    ![image](../../images/tools/image28.png)
  - 参考坐标系`frame6`

    `R=[180, 0, 90]`

    ![image](../../images/tools/image29.png)

### 4. `Link`和`Joint`配置

- `Link`配置

  - `Link`配置需要选择对应的坐标系、关节名称、参考旋转轴、关节类型和对应零件，以便生成正确的`urdf`文件用于路径规划和碰撞检测
  - 对于`base_link`只需要配置旋转坐标系和对应零件即可，其他`link`则需要配置上述所有参数
  - `base_link`配置

    ![image](../../images/tools/image30.png)
  - `link1`配置

    ![image](../../images/tools/image31.png)
  - `link2`配置

    ![image](../../images/tools/image32.png)
  - `link3`配置

    ![image](../../images/tools/image33.png)
  - `link4`配置

    ![image](../../images/tools/image34.png)
  - `link5`配置

    ![image](../../images/tools/image35.png)
  - `link6`配置

    ![image](../../images/tools/image36.png)
- `Joint`配置

  - ![image](../../images/tools/image37.png)
  - `1`：检查关节名称和顺序是否正确
  - `2`：检查关节对应的`Parent Link`和`Child Link`是否正确
  - `3`：检查关节类型`Joint Type`和参考坐标系、参考旋转轴是否正确

    - 一般情况下机械臂只用到两种关节类型
    - `revolute`类型：带有关节限制的旋转类型
    - `fixed`类型：固定关节类型
  - `4`：检查坐标变换是否正确
  - `5`：检查旋转方向是否正确（需要根据实际机器人旋转方向确定，可最后直接修改urdf文件）

    - `1`：正方向
    - `-1`：负方向
  - `6`：检查关节限制、力矩和速度设置是否正确 (可在机械臂说明书或是介绍单页中查看)

### 5. 修改`urdf`文件

- 在使用插件生成`urdf`文件时，只配置了关节`J1-J6`，接下来需要配置法兰盘和工具手
- 配置法兰盘

  - 创建`point_flange`

    ![image](../../images/tools/image38.png)
  - 测量`point456`到法兰盘参考点`point_flange`的距离

    ![image](../../images/tools/image39.png)
  - 在`urdf`文件中添加`link_flange`标签

    ```xml
      <link name="link_flange">
        <collision>
          <geometry>
            <box size="0.0001 0.0001 0.0001"/>
          </geometry>
          <origin xyz="0 0 0" rpy="0 0 0"/>
        </collision>
      </link>
    ```
  - 接着在`urdf`文件中添加`joint_flange`标签，将测得的距离转换为`m`后填入对应位置

    ```xml
      <joint name="joint_flange" type="fixed">
        <origin xyz="0.0 0.0 <测得的距离>" rpy="0.0 0.0 3.1415926"/>
        <parent link="link6"/>
        <child link="link_flange"/>
      </joint>
    ```
- 配置工具手

  - 首先需要获取`TCP`标定数据

  - 在`urdf`文件中创建`link_tool`标签

    ```xml
      <link name="link_tool"/>
    ```
  - 接着在`urdf`文件中创建`joint_tool`标签；注意将位置单位为`m`，角度单位为`rad`；此外旋转`rpy`不一定对应`ABC`，需要多次尝试或咨询机器人售后解决

    ```xml
      <joint name="joint_tool" type="fixed">
        <origin xyz="-0.0795825 0.000209429 0.428486" rpy="0.0000283791 0.547625 3.14159"/>
        <parent link="link_flange"/>
        <child link="link_tool"/>
      </joint>
    ```

### 6. 检查生成的`urdf`文件和`tf`树是否正确

- `Windows`下可借助本教程提供的[工具](#20250117144439-k69ouf7)来可视化检查`urdf`文件

  ![image](../../images/tools/image40.png)
- 检查`tf`树是否正确

  ![image](../../images/tools/image41.png)