# ROS2 基础

### 1. 创建功能包

- 创建功能包

  ```bash
  /*
  创建ros2功能包
  pkg_name: 功能包名称
  --build-type: 构建类型 ament_python
  --license: 声明开源协议 Apace2.0
  */
  ros2 pkg create <pkg_name> --build-type ament_python --license Apace-2.0
  ```

  功能包结构如下
  ![image](../../images/tools/image4.png)
- 在功能包中编写ROS2节点

  - 节点文件存放在`demo_python_pkg`目录下
  - 编写第一个ROS2节点

    ```python
    from rclpy.node import Node

    def run():
        rclpy.init()								#初始化ROS2
        py_node = Node("py_node")					#定义节点名称
        py_node.get_logger().info("Hello ros2!")	#调用节点的get_logger()方法
        rclpy.spin(py_node)							#以阻塞的方式运行节点
        rclpy.shutdown()							#关闭节点
    ```
  - 运行节点时需要对`setup.py`文件进行修改，使得编译器能够找到正确的函数运行入口

    ```python
    from setuptools import find_packages, setup

    package_name = 'demo_python_pkg'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='tong',
        maintainer_email='tong@todo.todo',
        description='TODO: Package description',
        license='Apace-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'python_node = demo_python_pkg.python_node:run',	#添加函数入口信息
            ],
        },
    )
    ```
  - 修改`package.xml`文件，添加运行依赖`<depend>rclpy<depend>` 注意：依赖需要根据节点所使用的库来对应添加

    ```xml
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>demo_python_pkg</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="tong@todo.todo">tong</maintainer>
      <license>Apace-2.0</license>

      <depend>rclpy</depend>

      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
    ```
- 构建功能包

  - 安装`colcon`工具（可选）

    ```bash
    sudo apt install python3-colcon-common-extensions   
    ```
  - 构建

    ```bash
    colcon build

    ---
    Starting >>> demo_python_pkg
    Finished <<< demo_python_pkg [0.62s]      

    Summary: 1 package finished [0.93s]
    ---
    ```
  - 运行节点

    ```bash
    source install/setup.bash

    ros2 run demo_python_pkg python_node
    ```
- 查看ROS环境变量

  ```bash
  printenv | grep -i ROS  
  ```
- 查看功能包是否安装

  ```bash
  ros2 pkg executables [pkg_name]
  ```

### 2. 创建工作空间

- 创建工作空间

  ```bash
  mkdir -p colcon_ws/src
  ```
- 构建工作空间中的功能包

  ```bash
  //1.构建所有功能包
  colcon build

  //2.构建指定功能包
  colcon build --packages-select <package_name>
  ```
- 注意

  - 使用python文件定义节点，如果使用`ros2 run`命令运行`node`文件，在对python文件进行修改后需要重新`colcon build- `

### 3. Python面相对象的编程

- 节点类与类的继承（Python）

  ```python
  from rclpy.node import Node

  class PersonNode(Node):
      def __init__(self, node_name:str ,name:str, age:int):
          print("PersonNode has been init!")
          super().__init__(node_name)		#调用super().init(arg)，初始化父类
          self.name = name
          self.age = age

      def eat(self, food_name:str):
          """
          method: eat food
          :food_name food name
          """
          # print(f"{self.name},{self.age}years old, and he like eat {food_name}")
          self.get_logger().info(f"{self.name}, {self.age} years old, and he like eat {food_name}")	#这里调用父类Node的get_logger()方法

  def main():
      rclpy.init()
      person_node = PersonNode("person_node", "Bob", 18)
      person_node.eat("tomato")
      rclpy.spin(person_node)		#传入要运行的节点
      rclpy.shutdown()
  ```

### 4. 话题与通信

- 查看节点信息

  ```bash
  //列出当前运行节点
  ros2 node list

  //查看节点信息
  ros2 node info /[node_name]
  ```
- 查看话题信息

  ```bash
  1. 查看当前话题列表
  ros2 topic list

  2. 输出话题信息到终端
  ros2 topic echo /[topic_name]

  3. 查看话题的具体信息
  ros2 topic info /[topic_name] -v

  -----------
  Type: turtlesim/msg/Pose		//消息接口

  Publisher count: 1

  Node name: turtlesim
  Node namespace: /
  Topic type: turtlesim/msg/Pose
  Endpoint type: PUBLISHER
  GID: 01.0f.c2.bb.71.13.12.ea.00.00.00.00.00.00.1c.03.00.00.00.00.00.00.00.00
  QoS profile:
    Reliability: RELIABLE
    History (Depth): UNKNOWN
    Durability: VOLATILE
    Lifespan: Infinite
    Deadline: Infinite
    Liveliness: AUTOMATIC
    Liveliness lease duration: Infinite

  Subscription count: 0
  -----------

  4. 查看消息接口的详细定义
  ros2 interface show [msg]

  -----------
  float32 x
  float32 y
  float32 theta

  float32 linear_velocity
  float32 angular_velocity
  -----------

  5. 使用命令行发布话消息
  #注意消息格式：大括号用来区分消息结构层级 冒号后需要添加空格
  ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0}}"
  ```
- Python话题订阅与发布

  - 话题发布（Publisher）

    ```python
    import rclpy
    import requests
    from rclpy.node import Node
    from example_interfaces.msg import String
    from queue import Queue

    class NovelPubNode(Node):
        def __init__(self, node_name):
            super().__init__(node_name)     #初始化父类

            self.novels_queue = Queue()     #初始化队列

            #创建发布者，发布小说
            self.novel_publisher = self.create_publisher(String, 'novel', 10)
            self.timer = self.create_timer(5, self.timer_callback)  #设置定时器


        def download_novel(self, url):
            response = requests.get(url)    #获取下载链接
            response.encoding = 'utf-8'		#设置编码格式
            self.get_logger().info(f"下载完成:{url}")

            for line in response.text.splitlines():   #按行分割，放入队列
                self.novels_queue.put(line)			#put方法，将每行内容放入队列

    	#回调函数
        def timer_callback(self):
            if self.novels_queue.qsize() > 0:
                msg = String()						#初始化消息对象，type：string
                msg.data = self.novels_queue.get()	#获取队列内容
                self.novel_publisher.publish(msg)	#发送消息
                self.get_logger().info(f"发布了一行小说：{msg.data}")

    def main():
        rclpy.init()
        node = NovelPubNode('novel_pub')	#节点名称，最好与setup.py中保持一致
        node.download_novel('http://localhost:8000/novel.txt')	#从本地服务器读取小说内容
        rclpy.spin(node)
        rclpy.shutdown()
    ```
  - 话题订阅（Subscriber）

    ```python
    import espeakng
    import rclpy
    from rclpy.node import Node
    from example_interfaces.msg import String
    from queue import Queue
    import threading
    import time

    class NovelSubNode(Node):
        def __init__(self, node_name):
            super().__init__(node_name)
            self.sub_queue = Queue()

    		#注意：发布者和订阅者传入的topic_name需要保持一致
            self.nover_subscriber = self.create_subscription(String, 'novel', self.novel_sub_callback, 10)
            self.sub_thread = threading.Thread(target=self.speaker_novel)	#指定线程
            self.sub_thread.start()		#开启线程

        def novel_sub_callback(self, msg):
            self.sub_queue.put(msg.data)	#将收到的消息内容放入队列中

        def speaker_novel(self):
            self.speaker = espeakng.Speaker()	#实例化espeakng对象
            self.speaker.voice = 'zh'   		#设置中文
            while rclpy.ok:
                if self.sub_queue.qsize() > 0:
                    self.text = self.sub_queue.get()	#获取队列内容
                    self.get_logger().info(f'开始朗读小说...')
                    self.speaker.say(self.text)			#将读取到的内容传递给say()方法
                    self.speaker.wait()					#等待speak结束
                else:
                    time.sleep(1)   #休眠1s，防止系统资源占用过多


    def main():
        rclpy.init()
        node = NovelSubNode('novel_sub')
        rclpy.spin(node)
        rclpy.shutdown()
    ```
  - `setup.py`配置

    ```python
    from setuptools import find_packages, setup

    package_name = 'demo_python_topic'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='tong',
        maintainer_email='tong@todo.todo',
        description='TODO: Package description',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
    			#将编写好的node文件配置再此处
                'novel_pub = demo_python_topic.novel_pub_node:main',		#节点名 = 功能包名.py文件名:主函数
                'novel_sub = demo_python_topic.novel_sub_node:main',   
            ],
        },
    )
    ```
  - 运行节点

    ```bash
    1. 启动python本地服务器
    python3 -m http.server

    2. 启动订阅者，等待发布者发布消息
    ros2 run demo_python_topic novel_sub

    3. 启动发布者，发布消息
    ros2 run demo_python_topic novel_pub
    ```

### 5. 话题通信实践

- 创建自定义消息

  - 创建相关消息接口功能包，因为ROS2底层为`C++`代码编写，因此这里构建类型要选择`Cmake`

    ```bash
    ros2 pkg create <pkg_name> --build-type ament_cmake --dependencies rosidl_default_generators builtin_interfaces --license Apache-2.0
    ```
  - 自定义消息文件必须放在`/package/msg`目录下，并且文件后缀必须为`.msg`，自定义消息文件示例如下：

    ```ROS
    builtin_interfaces/Time stamp   #记录时间戳，builtin_interfaces接口功能包下的Time类型
    string host_name            #系统名称
    float32 cpu_percent         #CPU使用率
    float32 memory_percent      #内存使用率
    float32 memory_total        #内存总量
    float32 memory_available    #剩余内存
    float64 net_sent            #网络发送数据总量
    float64 net_recv            #网络接收数据总量
    ```

    > `ROS2`消息接口支持的9种数据类型
    >
    > ```txt
    > bool
    > byte
    > char
    > float32, float64
    > int8, uint8
    > int16, uint16
    > int32, uint32
    > int64, uint64
    > string
    > ```
    >
  - 修改功能包`CMakeList.txt`和`package.xml`，添加相关依赖和声明

    ```cmake
    cmake_minimum_required(VERSION 3.8)
    project(status_interfaces)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # find dependencies
    find_package(ament_cmake REQUIRED)
    #添加相应依赖-----------------
    find_package(rosidl_default_generators REQUIRED)
    find_package(builtin_interfaces REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/SystermStatus.msg"
        DEPENDENCIES builtin_interfaces
    )
    ----------------------------

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      # the following line skips the linter which checks for copyrights
      # comment the line when a copyright and license is added to all source files
      set(ament_cmake_copyright_FOUND TRUE)
      # the following line skips cpplint (only works in a git repo)
      # comment the line when this package is in a git repo and when
      # a copyright and license is added to all source files
      set(ament_cmake_cpplint_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
    ```

    ```xml
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>status_interfaces</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="tong@todo.todo">tong</maintainer>
      <license>Apache-2.0</license>
      <!--添加声明，声明该功能包为消息接口功能包-->
      <member_of_group>rosidl_interface_packages</member_of_group>

      <buildtool_depend>ament_cmake</buildtool_depend>

      <depend>rosidl_default_generators</depend>
      <depend>builtin_interfaces</depend>

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>
    ```

### 6. 服务与参数通信

- 查看服务列表和对应接口

  ```bash
  ros2 service list -t

  ---
  /clear [std_srvs/srv/Empty]
  /kill [turtlesim/srv/Kill]
  /reset [std_srvs/srv/Empty]
  /spawn [turtlesim/srv/Spawn]
  /turtle1/set_pen [turtlesim/srv/SetPen]
  /turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
  ```

  > `/spawn`等表示服务名称
  >
  > `-t`参数表示显示服务的接口类型
  >
  > `[]`内则是服务的接口类型
  >
- 调用服务

  ```bash
  ros2 service call /spawn turtlesim/srv/Spawn "{x: 1, y: 1}"
  ```

  > 参数`/spawn`为服务名字
  >
  > 参数`turtlesim/srv/Spawn`为服务的接口类型
  >
  > 参数`"{x: 1, y: 1}"`为Request数据
  >
- 查看参数列表

  ```bash
  ros2 param list

  ---
  /turtlesim:
    background_b
    background_g
    background_r
    qos_overrides./parameter_events.publisher.depth
    qos_overrides./parameter_events.publisher.durability
    qos_overrides./parameter_events.publisher.history
    qos_overrides./parameter_events.publisher.reliability
    use_sim_time
  ```

  - 获取和设置节点参数的值

    ```bash
    1. 获取参数值
    ros2 param get /turtlesim background_b

    2. 设置参数的值
    ros2 param set /turtlesim background_b
    ```
  - 将参数导出到文件中

    ```bash
    ros2 param dump /turtlesim > turtlesim_param.yaml 
    ```
  - 运行节点时指定参数文件

    ```bash
    ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_param.yaml
    ```
- 创建自定义服务接口

  - 创建服务接口功能包

    ```bash
    ros2 pkg create service_interfaces --build-type ament_cmake --dependencies rosidl_default_generators sensor_msgs --license Apache-2.0
    ```
  - 自定义服务接口文件必须放在`/package/srv`目录下，并且文件后缀必须为`.srv`，自定义消息文件示例如下：

    ```ROS
    sensor_msgs/Image image # 原始图像
    ---
    int16 number            # 人脸数
    float32 use_time        # 识别耗时
    int32[] top             #人脸在图像中的位置
    int32[] right
    int32[] bottom
    int32[] left
    ```

    > `---`上部为`Request`部分；定义一个`sensor_msgs/Image`类型用来表示图像
    >
    > `---`下部为`Response`部分，定义了服务相关参数
    >
  - 修改`CMakeLists.txt`文件，添加相关依赖

    ```cmake
    ...
    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(sensor_msgs REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/FaceDetector.srv"
        DEPENDENCIES sensor_msgs
    )
    ...
    ```
  - 修改`package.xml`，添加相应声明

    ```xml
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>service_interfaces</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="tong@todo.todo">tong</maintainer>
      <license>Apache-2.0</license>

      <buildtool_depend>ament_cmake</buildtool_depend>
      <!--ROS2接口功能包声明-->
      <member_of_group>rosidl_interface_packages</member_of_group>
      <depend>rosidl_default_generators</depend>
      <depend>sensor_msgs</depend>

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>
    ```
- 服务与参数通信实践

  - 编写服务端节点

    ```python
    import rclpy
    from rclpy.node import Node
    from service_interfaces.srv import FaceDetector
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge
    import cv2
    import face_recognition
    import time
    from rcl_interfaces.msg import SetParametersResult

    class FaceDetecTorService(Node):
        def __init__(self, node_name):
            super().__init__(node_name)
            self.bridge = CvBridge()    # 实例化CvBridge类
            self.default_image_path = get_package_share_directory('demo_python_service') + '/resource/image2.png'

            # 创建服务
            self.service = self.create_service(FaceDetector, '/face_detector', self.face_detector_callback)

            self.upsample_times = 1
            self.model = 'hog'

            # 声明和获取参数
            self.declare_parameter('face_locations_upsample_times', 1)
            self.declare_parameter('face_locations_model', 'hog')
            self.upsample_times = self.get_parameter("face_locations_upsample_times").value
            self.model = self.get_parameter('face_locations_model').value
            self.set_parameters([rclpy.Parameter('face_locations_model', rclpy.Parameter.Type.STRING, 'cnn')])
            self.add_on_set_parameters_callback(self.parameter_callback)

        # 定义参数回调函数
        def parameter_callback(self, parameters):
            for parameter in parameters:
                self.get_logger().info(f'参数{parameter.name}设置为{parameter.value}')
                if parameter.name == 'face_locations_upsample_times':
                    self.upsample_times = parameter.value
                if parameter.name == 'face_locations_model':
                    self.model = parameter.value

            return SetParametersResult(successful=True)

        # 定义人脸识别服务回调函数
        def face_detector_callback(self, request, response):
            if request.image.data:
                cv_image = self.bridge.imgmsg_to_cv2(request.image)
            else:
                cv_image = cv2.imread(self.default_image_path)

            start_time =time.time()
            self.get_logger().info(f'图像加载完成，开始人脸检测...')
            face_locations = face_recognition.face_locations(
                cv_image, 
                number_of_times_to_upsample = self.upsample_times,
                model = self.model
            )
            end_time = time.time()
            self.get_logger().info(f'检测完成，耗时{end_time-start_time}')
            response.number = len(face_locations)
            response.use_time = end_time - start_time

            for top, right, bottom, left in face_locations:
                response.top.append(top)
                response.right.append(right)
                response.bottom.append(bottom)
                response.left.append(left)

            return response

    def main():
        rclpy.init()
        node = FaceDetecTorService('face_detector_service')
        rclpy.spin(node)
        rclpy.shutdown()
    ```
  - 编写客户端节点

    ```python
    import rclpy
    from rclpy.node import Node
    from service_interfaces.srv import FaceDetector
    from sensor_msgs.msg import Image
    from ament_index_python.packages import get_package_share_directory
    import cv2
    from cv_bridge import CvBridge
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

    class FaceDetectorClient(Node):
        def __init__(self, node_name):
            super().__init__(node_name)
            self.client = self.create_client(FaceDetector, '/face_detector')
            self.bridge = CvBridge()
            self.test_image_path = get_package_share_directory('demo_python_service') + '/resource/image2.png'
            self.image = cv2.imread(self.test_image_path)

        def send_request(self):
            # 判断服务是否在线
            while self.client.wait_for_service(timeout_sec=1.0) is False:
                self.get_logger().info(f'等待服务上线...')

            # 构造Request
            self.request = FaceDetector.Request()
            self.request.image = self.bridge.cv2_to_imgmsg(self.image)

            # 发送请求并spin等待服务处理完成
            self.future = self.client.call_async(self.request)
            rclpy.spin_until_future_complete(self, self.future)

            # 处理结果信息
            self.response = self.future.result()
            self.get_logger().info(f'识别结果：图像中共识别{self.response.number}张脸，耗时{self.response.use_time}')

            # 显示识别结果
            self.show_face_locations(self.response)

        def call_set_parameters(self, parameters):
            # 创建一个参数设置客户端，并等待服务上线
            self.client_param = self.create_client(SetParameters, '/face_detector_service/set_parameters')
            while not self.client_param.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'等待参数设置服务端上线...')

            # 创建请求对象
            self.request_param = SetParameters.Request()
            self.request_param.parameters = parameters

            # 异步调用，等待并返回响应结果
            self.future_param = self.client_param.call_async(self.request_param)
            rclpy.spin_until_future_complete(self, self.future_param)
            self.response_param = self.future_param.result()

            return self.response_param

        def update_detector_model(self, model):
            # 创建一个参数对象
            self.param = Parameter()
            self.param.name = 'face_locations_model'

            # 创建参数值对象并赋值
            self.new_model_value = ParameterValue()
            self.new_model_value_type = ParameterType.PARAMETER_STRING
            self.new_model_value.string_value = model
            self.param.value = self.new_model_value

            # 请求更新参数并处理
            self.response_model = self.call_set_parameters([self.param])
            for result in self.response_model.results:
                if result.successful:
                    self.get_logger().info(f'参数{self.param.name}设置为{self.param.value}')
                else:
                    self.get_logger().info(f'参数{self.param.name}设置失败，原因为：{result.reason}')

        def show_face_locations(self, response):
            for i in range(response.number):
                top = response.top[i]
                right = response.right[i]
                bottom = response.bottom[i]
                left = response.left[i]
                cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 2)

            cv2.imshow('Face Detector Result', self.image)
            cv2.waitKey(0)


    def main():
        rclpy.init()
        node = FaceDetectorClient('face_detector_client')
        node.update_detector_model('hog')
        node.send_request()
        # node.update_detector_model('hog')
        # node.send_request()
        rclpy.spin(node)
        rclpy.shutdown()
    ```

### 7. launch启动脚本

- ROS2支持使用`Python`、`XML`、`YAML`语言编写`launch`文件
- `launch`文件统一放在功能包的`/launch`文件夹下，使用`Python`编写时文件后缀为`.launch.py`
- `launch`启动文件

  - 使用`Python`编写启动文件

    ```python
    import launch
    import launch_ros

    def generate_launch_description():
    	#使用launch传递参数
        action_declare_arg_max_spped = launch.actions.DeclareLaunchArgument('launch_max_speed', default_value='2.0')

    	# 定义action对象
        action_node_turtle_control = launch_ros.actions.Node(
            package='demo_cpp_service',
            executable="turtle_control",
            output='screen',
            parameters=[{'max_speed': launch.substitutions.LaunchConfiguration(
      'launch_max_speed', default='2.0')}],

        )
        action_node_patrol_client = launch_ros.actions.Node(
            package='demo_cpp_service',
            executable="patrol_client",
            output='log',
        )
        action_node_turtlesim_node = launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='both',
        )

       	# 合成所有action对象启动描述并返回
        launch_description = launch.LaunchDescription([
            action_declare_arg_max_spped,
            action_node_turtle_control,
            action_node_patrol_client,
            action_node_turtlesim_node
        ])

        return launch_description
    ```
  - 修改`setup.py`配置文件

    ```python
    from glob import glob

    	...
    		data_files=[
    			...
    			('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    		]
    	...
    ```
- `launch`使用进阶

  - 动作的更多使用办法

    ```python
    import launch
    import launch_ros
    from ament_index_python.packages import get_package_share_directory

    def generate_launch_description():
        # 利用IncludeLaunchDescription动作包含其他launch文件
        action_include_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_source.PythonLaunchDescriptionSource(
                [get_package_share_directory("turtlesim"), "/launch","/multisim.launch.py"]
            )
        )

        # 利用ExecuteProcess动作执行命令行
        action_executeprocess = launch.actions.ExecuteProcess(
            cmd = ['ros2', 'service', 'call', '/turtlesim1/Spawn', 'turtlesim/srv/Spawn', '{x: 1, y: 1}']
        )

        # 利用Loginfo动作输出日志
        action_log_info = launch.actions.LogInfo(msg='使用launch命令来调用服务生成海龟')

        # 利用定时器动作实现依次启动日志输出和进程执行，并使用GruopAction封装成组合
        action_group = launch.actions.GroupAction([
            launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
            launch.actions.TimerAction(period=3.0, actions=[action_executeprocess]),
        ])

        # 合成启动描述并返回
        launch_description = launch.LaunchDescription([action_include_launch, action_group])

        return launch_description
    ```
### 8. Zsh终端无法自动补全

- 修改`/opt/ros/humble/setup.zsh`，在文件最后添加如下内容

  ```bash
  # argcomplete for ros2 & colcon
  eval "$(register-python-argcomplete3 ros2)"
  eval "$(register-python-argcomplete3 colcon)"

  ```

  ```bash
  sudo gedit /opt/ros/humble/setup.zsh
  ```
- 在`.zshrc`中，将`source /opt/ros/humble/setup.zsh`放在最后一行