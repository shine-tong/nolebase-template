# VS Code相关配置
### 1 扩展（参照开发语言）

#### 1.1 ROS必备扩展

- ROS
- catkin-tools
- Chinese(Simplified)

#### 1.2 Python扩展

- Python
- Pylance
- Jupyter(可选)

#### 1.3 C++扩展

- C/C++
- C/C++ Extension Pack
- CMake
- CMake Tools

#### 1.4 可选扩展

- XML
- YAML
- Docker
- Msg Language Support
- URDF

### 2 .vscode  json文件配置

#### 2.1 c_cpp_properties.json

- 一般不需要修改，自动包含ROS的头文件目录，一般在需要添加自定义头文件目录时修改

```json
{
	“configurations”:[
		{
			"browse":{
				"databaseFilename": "${default}",
				"limitSymbolsToIncludeHeaders": false
			},
			"includePath":[
				"/home/robot/ROS/catkin_ws/devel/include/**",
				"/opt/ros/noetic/include/**",
				"/home/robot/ROS/COMP_URDF/src/moveit_demo/include/**",
				"/usr/include/**"
			],
			"name": "ROS",
			"intelliSenseMode": "gcc-x64",
			"compilerPath": "usr/bin/gcc",
			"cStandard": "gnu11",
			"cppStandard": "c++14"
		}
	],
	"version": 4
}
```

#### 2.2 settings.json

- Python相关配置

```json
{
	"python.autoComplete.extraPaths":[
		"/home/tong/moveir_ws/devel/lib/python3/dist-packages",
		"/opt/ros/noetic/lib/python3/dist-packages"
	],
	"python.analysis.extraPaths":[
		"/home/tong/moveir_ws/devel/lib/python3/dist-packages",
		"/opt/ros/noetic/lib/python3/dist-packages"
	],
	"python.envFile": "${workspaceFolder}/.env",
}
```

- `python.autoComplete.extraPaths`用于Python自动补全搜索的包路径
- `python.analysis.extraPaths`用于Python静态分析搜索的包路径，用于代码检查
- `python.envFile`用于添加自定义Python环境变量

  - 注意：需要先在工作空间的根目录下创建一个`.env`文件，并将需要添加的环境变量写入该文件

```bash
//cd 到工作空间根目录
cd /path/to/work_space
//创建.env文件
touch .env
//直接将需要添加的环境变量粘贴到.env文件中，保存退出
```