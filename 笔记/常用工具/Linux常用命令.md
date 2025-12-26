### 常用Shell命令

```bash
1. 查看可更新软件列表
sudo apt list --upgradable

2. 更新所有软件
sudo apt upgradable

3. 更新指定软件
sudo apt install --only-upgrade <package_name>

4. 命令行配置Ubuntu软件包地址
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

5. 下载并合并指定的ROS workspace的依赖项配置文件到当前ROS workspace的src文件夹下
wstool merge -t src https://raw.githubusercontent.com/moveit/moveit/1.1.14/moveit.rosinstall

6. 添加Python环境变量
export PYTHONPATH=$PYTHONPATH:~/moveir_ws/devel/lib/python3/dist-packages

7. 删除Python环境变量
export PYTHONPATH=$(echo $PYTHONPATH | sed -e 's|:/home/tong/home/tong/moveir_ws/devel/lib/python3/dist-packages||' -e 's|/home/tong/home/tong/moveir_ws/devel/lib/python3/dist-packages:||' -e 's|/home/tong/home/tong/moveir_ws/devel/lib/python3/dist-packages||')

8. 列出系统中所有安装包
sudo dpkg-query -l
```

### ubuntu开机时进入tty界面

    - Ctrl+Alt+F2