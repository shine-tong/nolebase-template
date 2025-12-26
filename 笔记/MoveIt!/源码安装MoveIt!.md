# 源码安装 MoveIt!


### 源码安装注意事项
- 基本步骤参照[官方教程](https://moveit.ai/install/source/)
- **构建旧版本moveit！时，需要按需修改官方给出的链接以构建所需的版本，具体步骤如下**：

- 下载.rosinstall文件

  - 先将`moveit.rosinstall`文件下载到本地(注意：不要立即执行`wstool merge`)

```shell
wget https://raw.githubusercontent.com/moveit/moveit/master/moveit.rosinstall -O moveit.rosinstall
```

- 修改`moveit.rosinstall`文件，并找到执行moveit仓库的部分，如下所示：

```xml
- git:
    local-name: moveit
    uri: https://github.com/ros-planning/moveit.git
    version: master
```

- 通过修改`version`字段，可以构建不同的moveit!版本

  - 如果想使用`noetic-devel`分支

    - `version: noetic-devel`
  - 如果想使用`1.1.14 tag`

    - `version: 1.1.14`
- 执行`wstool merge`命令

  - 使用修改后的`moveit.rosinstall`文件运行`wstoll merge`

```shell
wstool merge -t src moveit.rosinstall
```

- 更新工作空间，将源码下载至工作空间

```shell
wstool update -t src
```

- 构建源码

```shell
catkin build
```

- 注意：在构建构成中可能会报错，根据提示进行处理即可，一般为缺少某些依赖。
- **最后source工作空间**

```bash
source .bashrc
```