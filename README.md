# Ros2-foxy
<span id="1"></span>
[Ros2官网文档](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html)
[rclpy官方文档](https://docs.ros2.org/latest/api/rclpy/index.html)
[我的GitHub项目](https://github.com/xianzhou127/Ros2Learning?tab=readme-ov-file#1)

- [Ros2-foxy](#ros2-foxy)
  - [1.创建工作空间](#1创建工作空间)
  - [2.编译代码(colcon)](#2编译代码colcon)
    - [编译命令](#编译命令)
  - [3.功能包](#3功能包)
    - [获取功能包](#获取功能包)
    - [创建功能包](#创建功能包)
    - [列出可执行文件](#列出可执行文件)
  - [4.节点](#4节点)
    - [节点命令](#节点命令)
  - [5. 通讯](#5-通讯)
    - [通信方式](#通信方式)
    - [RQT工具rqu\_graph](#rqt工具rqu_graph)
    - [话题相关命令](#话题相关命令)
    - [服务相关命令](#服务相关命令)
    - [参数相关命令](#参数相关命令)
    - [动作相关命令](#动作相关命令)
    - [接口命令](#接口命令)
- [代码编写](#代码编写)
  - [1.节点创建](#1节点创建)
  - [2. 话题与服务](#2-话题与服务)
    - [topic发布者](#topic发布者)
    - [topic订阅者](#topic订阅者)
    - [自定义接口](#自定义接口)
    - [server服务端](#server服务端)
    - [server客户端](#server客户端)
    - [param参数](#param参数)
    - [action服务端](#action服务端)
  - [3. launch文件编写](#3-launch文件编写)


## 1.创建工作空间
`(base) xianzhou@xianzhou:~$ mkdir -p ~/dev_ws/src`

[返回目录](#1)

## 2.编译代码(colcon)
```
sudo apt install python3-colcon-ros
colcon build
```
若已经安装conda并激活，在base环境下编译后报错：
`ModuleNotFoundError: No module named 'catkin_pkg'`
则在base环境下重新安装后再编译即可
`pip install catkin_pkg`

### 编译命令
1. 只编译一个包
`colcon build --packages-select <pkg>`
2. 不编译测试单元
`colcon build --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=0`
3. 运行编译的包的测试
`colcon test`
4. 允许通过更改src下的部分文件来改变install（直接用这个）
每次调整 python 脚本时都不必重新build了
`colcon build --symlink-instal`

[colcon官方文档](https://colcon.readthedocs.io/en/released/user/installation.html)

[返回目录](#1)

## 3.功能包

### 获取功能包
`sudo apt install ros-<version>-package_name`

### 创建功能包
`ros2 pkg create <package-name>  --build-type  {cmake,ament_cmake,ament_python}  --dependencies <依赖名字>`
- --build-type 指定编译类型 ament_cmake(默认) ament_python
- --dependencies 功能包的依赖 rclcpp rclpy

`ros2 pkg create example_py  --build-type ament_python --dependencies rclpy`
### 列出可执行文件
1. 所有可执行包及文件
`ros2 pkg executables`
2. 某包的所有可执行文件
`ros2 pkg executables turtlesim`
3. 所有包
`ros2 pkg list`
4. 查看包的信息
`ros2 pkg xml <pkg>`

[返回目录](#1)

## 4.节点

### 节点命令
1. 启动节点
`ros2 run <package_name> <executable_name>`
2. 查看节点列表
`ros2 node list`
3. 查看节点信息
`ros2 node info <node>`
4. 重映射节点名称
`ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle`
5. 运行节点时设置参数
`ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10`

[返回目录](#1)

## 5. 通讯

### 通信方式
- 话题-topics
- 服务-services
- 动作-Action
- 参数-parameters
  
### RQT工具rqu_graph
查看消息树
`rqt_graph`

### 话题相关命令
1. 帮助
`ros2 topic -h`
2. 所有话题
`ros2 topic list`
3. 额外输出消息类型
`ros2 topic list -t`
4. 打印实时话题内容
`ros2 topic echo <topic_name>`
5. 查看话题信息
`ros2 topic info <topic_name>`
6. 查看消息类型
`ros2 interface show std_msgs/msg/String`
7. 手动发布话题
`ros2 topic pub <topic_name> <massage_type> <massage_content>`
e.g.
`ros2 topic pub /chatter std_msgs/msg/String '{data: "hello world"}`
在最后多加一个-可以查看其他参数
`ros2 topic pub /chatter std_msgs/msg/String '{data: "hello world"} -`

### 服务相关命令
1. 查看服务列表
`ros2 service list`
2. 手动调用服务
`ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"`
3. 查看服务接口类型
`ros2 service type /add_two_ints`
4. 查找使用某一接口的服务
`ros2 service find example_interfaces/srv/AddTwoInts`

### 参数相关命令
1. 查看所有节点的参数列表
`ros2 param list`
2. 详细查看某节点某参数
`ros2 param describe /turtlesim background_b`
3. 查看参数值
`ros2 param describe /turtlesim background_b`
4. 动态设置参数值
`ros2 param set /turtlesim background_r 44`
5. 参数保存,保存为.yaml
`ros2 param dump /turtlesim`
6. 参数读取
`ros2 param load  /turtlesim ./turtlesim.yaml`
7. 启动节点同时参数读取
`ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml `

### 动作相关命令
1. 获取目前系统中的action列表。
`ros2 action list`
2. 同时查看节点类型
`ros2 action list -t`
3. 查看action信息
`ros2 action info /turtle1/rotate_absolute `
4. 发送action请求到服务端
`ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5}" --feedback`

### 接口命令
通讯接口就是各种信息包，查看包可以直接上ros index
1. 查看接口列表
`ros2 interface list`
1. 查看所有接口包
`ros2 interface package`
1. 查看某一包的接口
`ros2 interface std_msgs`
1. 查看某一接口详细内容
`ros2 interface show std_msgs/msg/String`
1. 输出某一接口所有属性
`ros2 interface proto sensor_msgs/msg/Image`

[返回目录](#1)

----

# 代码编写

## 1.节点创建
1. /src下创建节点包
`ros2 pkg create ssr_pkg --build-type ament_python --dependencies rclpy`
2. /ssr_pkg/ssr_pkg下创建脚本文件Ultrasonic_node.py(路径不能变)
3. 编写脚本代码
```
#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node

def main(args=None):
    #入口函数
    rclpy.init(args=args)   #初始化节点
    Ultrasonic_node = Node("Ultrasonic")  #新建节点
    Ultrasonic_node.get_logger().info("hello world")  
    rclpy.spin(Ultrasonic_node)    #保持节点运行
    rclpy.shudown()     #关闭rclpy
```
新建节点还能这么写
`Ulterasonic_node = rclpy.create_node("Ultrasonic"))`
4. 修改/ssr_pkg/setup.py
```
'console_scripts': [
            "Ultrasonic = ssr_pkg.Ultrasonic_node:main"
        ],
```
- Ultrasonic 节点名称，ros2 run的时候用的就是这个
- ssr_pkg 包名
- Ultrasonic_node 脚本py文件名
- main 入口函数
5. 编译一下
```
colcon build --symlink-instal
source install/setup.bash
```
6. 运行节点
`ros2 run ssr_pkg Ultrasonic.py`

[返回目录](#1)

## 2. 话题与服务

### topic发布者
1. 导入消息包，与ROS1不同，ROS2不需要在创建包时添加std_msgs依赖项
`from std_msgs.msg import String`
2. 创建发布者
`pub_pos = Node.create_publisher(String,'Position',qos_profile=10)`
- String 消息类型
- 'Position' topic名称
- qos_profile=10 类似ROS1的queue_size=10
类内写法
`self.pub_pos = self.create_publisher(String,'Position',qos_profile=10)`
3. 设置定时发布
`timer = Node.create_timer(timer_period_sec=period_sec,callback=self.__pub_callback)`
类内写法
`self.timer = self.create_timer(timer_period_sec=period_sec,callback=self.__pub_callback)`
4. 定时发布回调函数
```
def __pub_callback(self):
  self.pub_pos.publish(self.msg)
```
5. 创建消息
```
  msg = String()
  msg.data = "I am Ultrasonic"
```
6. 发布消息
`pub_pos.publish(msg)`
类内写法
`self.pub_pos.publish(self.msg)`
7. 全部代码
```
#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  #与ros1不同，在创建pkg时不需要写std_msgs的依赖项

#OOP编程
class SsrNode(Node):
    '''
    define a class of ssr nodes
    '''
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Publisher Ultrasonic_node online")
        self.pub_pos = self.create_publisher(String,'Position',qos_profile=10)

    def publish(self,period_sec,msg):
        self.msg = msg
        self.timer = self.create_timer(timer_period_sec=period_sec,callback=self.__pub_callback)  #回调函数这里不能传参

    #两个下划线表示private
    def __pub_callback(self):
        self.pub_pos.publish(self.msg)

def main(args=None):
    '''
    入口函数
    '''
    rclpy.init(args=args)   #初始化节点
    Ultrasonic_node = SsrNode("Ultrasonic_node")  #新建节点

    msg = String()
    msg.data = "I am Ultrasonic"
    Ultrasonic_node.publish(1,msg)
    rclpy.spin(Ultrasonic_node)    #保持节点运行
    rclpy.shudown()     #关闭rclpy
```

[返回目录](#1)

### topic订阅者
1. 导入消息包
`from std_msgs.msg import String`
2. 创建订阅者
`sub_pos = Node.create_subscription(String,"Position",self.__sub_callback,qos_profile=10)`
3. 订阅回调函数
```
def __sub_callback(self,msg):
  self.get_logger().info(msg.data)
```
4. 全部代码，我是重新创建了一个节点，步骤看节点创建，别忘了修改setup\.py
```
#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AtrNode(Node):
    '''
    define a class of atr node
    '''
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("subscriber motor_node online")
        self.sub_pos = self.create_subscription(String,"Position",self.__sub_callback,qos_profile=10)

    def __sub_callback(self,msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    Motor_node = AtrNode("Motor_node")

    rclpy.spin(Motor_node)    #保持节点运行
    rclpy.shudown()     #关闭rclpy  
```

[返回目录](#1)

### 自定义接口
1. 创建接口包
`ros2 pkg create robot_msgs --build-type ament_cmake --dependencies rosidl_default_generators geometry_msgs`
2. 包内创建msg(话题接口)和srv(服务接口)和action(动作接口)文件夹，并创建接口文件(.msg和.srv .action)
├── msg
│   ├── RobotPose.msg
│   └── RobotStatus.msg
├── package.xml
├── srv
│ └──MoveRobot.srv
├── action
│ └──ControlRobot.action
1. 添加接口代码
   
RobotStatus.msg
```
uint32 STATUS_MOVEING = 1
uint32 STATUS_STOP = 1
uint32  status
float32 pose
```
RobotPose.msg
```
uint32 STATUS_MOVEING = 1
uint32 STATUS_STOP = 2
uint32  status
geometry_msgs/Pose pose
```
MoveRobot.srv
```
# 前进后退的距离
float32 distance
---
# 当前的位置
float32 pose
```
ControlRobot.action
```
# Goal: 要移动的距离
float32 distance
---
# Result: 最终的位置
float32 pose
---
# Feedback: 中间反馈的位置和状态
float32 pose
uint32 status
uint32 STATUS_MOVEING = 3
uint32 STATUS_STOP = 4
```
4. 修改CMakeLists.txt文件
```
# 添加下面的内容
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/RobotStatus.msg"
  "srv/MoveRobot.srv"
  "action/ControlRobot.action"
  DEPENDENCIES geometry_msgs
)
```
5. 修改package.xml文件
`<member_of_group>rosidl_interface_packages</member_of_group> #添加这一行`
6. 编译
```
colcon build --packages-select robot_msgs
source install/setup.bash
```
7. 可能遇到的问题
- 报错：Could not find a package configuration file provided by
  "rosidl_default_generator" with any of the following names:
  打错包名了，在CMakeLists.txt文件和package.xml中修改相应包名即可
- 报错：ModuleNotFoundError: No module named 'em'
  `pip install empy==3.3.2`
- 报错：AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
  empy版本太高，使用上面的命令重新安装
- 报错：ModuleNotFoundError: No module named 'lark'
  `pip install lark`
- 调用自定义接口包时报错:ModuleNotFoundError: No module named 'robot_msgs.robot_msgs_s__rosidl_typesupport_c'
  我是因为在anaconda环境下编译的massage，解决方法就是在工作空间的build和install文件夹中删除message所属部分，然后退出anaconda环境重新编译(就是在/home/.bashrc文件中把source ~/anaconda3/bin/activate注释掉再重新打开一个终端)。
  经过测试，使用上述办法编译后在anaconda环境中也可以正常运行。

[返回目录](#1)

### server服务端
1. 定义接口类型
`from robot_msgs.srv import MoveRobot`
2. 创建服务端
`self.moverobot_srv_ = self.create_service(MoveRobot,"move_robot",self.__moverobot_callback)`
- MoveRobot 消息接口
- “move_robot” 服务名称
- self.__moverobot_callback 服务端回调函数
3. 编写服务端回调函数
```
def __moverobot_callback(self,request,response):
  '''
  服务端回调函数,获得前进距离，并返回现在位置
  request和response为固定参数，必须有。不过名字可以更换
  request\response 都是MoveRobot类型
  '''
  self.get_logger().info("get requestions,moving forward %f meters"%request.distance)
  self.pose += request.distance
  response.pose = self.pose
  return response
```

### server客户端
1. 定义接口类型
`from robot_msgs.srv import MoveRobot`
2. 创建客户端
`self.moverobot_client_ = self.create_client(MoveRobot,"move_robot")`
- MoveRobot 接口类型
- "move_robot" 服务名称
3. 编写服务调用函数
```
def move_robot(self,distance):
    '''
    客户端发送请求函数
    distance 输入移动距离
    '''
    while rclpy.ok() and self.moverobot_client_.wait_for_service(1)==False:
        self.get_logger().info(f"等待服务端上线....")

    #创建请求对象,后面的.Request()是自带的不是写在massage里的 ，可能是把消息打包？
    request = MoveRobot.Request()
    request.distance = distance
    self.get_logger().info(f"Requesting robot to move forward {distance} meters")
    #调用服务函数，并设置接收回调函数
    self.moverobot_client_.call_async(request).add_done_callback(self.move_robot_callback__)
```
4. 编写接收回调函数
```
def move_robot_callback__(self,response):
    '''
    客户端回调函数
    response为服务端返回值，就是客户端回调函数return的呢个
    ''' 
    # .result()可能是把消息解包？
    result = response.result()
    self.get_logger().info(f"now located {result.pose} meters away")
```
[返回目录](#1)

### param参数
1. 定义参数
`Node.declare_parameter(Ultrasonic_node,"period",5)`
- Ultrasonic_node 定义的节点对象
- "period" 参数名称
- 5 参数值
类内写法，不需要写对象了
`self.declare_parameter("period",5)`
2. 获取参数
`period = Node.get_parameter(Ultrasonic_node,"period").value`
- Ultrasonic_node 定义的节点对象
- "period" 参数名称
- .value 返回值的属性,可以单独写个点.看看补全的其他方法和属性
还能这样写
`period = Node.get_parameter(Ultrasonic_node,"period").get_parameter_value().integer_value`
- .get_parameter_value() 获取参数数值的方法
- .inter_value 获取整数数值
类内写法
`period = self.get_parameter("period").value`

[返回目录](#1)

### action服务端
有点复杂，用到的时候再写

## 3. launch文件编写
1. 随便选择一个包，创建一个launch文件夹，文件夹内创建一个.launch.py文件
```
├── atr_pkg
│   ├── launch  #放launch文件
|       └── Motor.launch.py
│   └── atr_pkg #放的节点.py文件夹,别弄错了
|   ...
```
2. 编写代码
```
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess # ExecuteProcess用于在launch文件中启动一个进程

def generate_launch_description():
    # 函数名固定，由ros2 launch 扫描调用

    #创建两个节点的启动对象，其他启动项写法不同
    motor_node = Node(
        package="atr_pkg",
        executable="Motor_node"
    )

    Ultrasonic_node = Node(
        package="ssr_pkg",
        executable="Ultrasonic" #这里的节点名为你在节点.py中的节点对象
                                #也就是setup.py中entry_points里添加的最前面的呢个名字
    )

    # teleop_key = ExecuteProcess(
    #         cmd=['xterm', '-e', 'ros2', 'run', 'turtlesim', 'turtle_teleop_key'],
    #         name='teleop_key',
    #     )
    #     ExecuteProcess()中最重要的参数是cmd，它是一个包含要执行的命令及其参数的列表。例如，cmd=['ls', '-l']将执行ls -l命令。
    #     在上面的launch示例文件中，需要在你的系统中安装xterm才能使用这个功能。
    #     sudo apt-get update
    #     sudo apt-get install xterm
    #     为什么没有将turtle_teleop_key以Node的方式进行启动？大概问题是，launch启动文件无法从终端获取用户输入，
    #     所以使用ExecuteProcess替代，这将会自动运行一个xterm终端模拟器来获取键盘输入。
    #     取自https://blog.csdn.net/xijuezhu8128/article/details/131818608

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [motor_node, Ultrasonic_node])
    
    return launch_description
```
launch启动Rviz2
```
from ament_index_python.packages import get_package_share_directory #启动rviz用
import os   #启动rviz用

def generate_launch_description():
    rviz_name = 'display.rviz'  #rviz的配置文件
    # 获取功能包路径（注意，这个路径是在工作空间的install文件夹里
    package_name = 'atr_pkg'
    pkg_description = get_package_share_directory(package_name)
    # 声明文件路径，os.path.join将口号内的str用\连接，组成路径
    rviz_config_path = os.path.join(pkg_description,'rviz',rviz_name)

    # 创建Rviz项
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path]
    )

    return  launch_description = LaunchDescription(
        [rviz])

# setup.py中添加这个(先看步骤3)
# (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),    #启动Rviz添加这一行
```
launch启动Gazebo
```
from ament_index_python.packages import get_package_share_directory #启动rviz和Gazebo用
import os   #启动rviz和Gazebo用

def generate_launch_description():

    # 获取world路径
    world_file_name = 'empty_world.world'
    package_name = 'atr_pkg'
    pkg_description = get_package_share_directory(package_name)
    world = os.path.join(pkg_description,'world',world_file_name)

    # 创建Gazebo项
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',world],  #若没有world则删掉'--verbose',world
        output='screen'
        )

    return  launch_description = LaunchDescription(
            [gazebo])

# 使用保存的world时，在setup.py中添加这个(先看步骤3)
# (os.path.join('share', package_name, 'world'), glob('world/*.world')),    #使用保存的地图添加这一行
```
3. setup.py文件中添加额外代码
```
from glob import glob   #添加这一行
import os               #添加这一行

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  #添加这一行
    ],
```
4. 编译并启动
```
colcon build --packages-select atr_pkg
source install/setup.bash
ros2 launch atr_pkg Motor.launch.py
```

[返回目录](#1)
