# Ros2-foxy
<span id="1"></span>
[Ros2官网文档](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html)

- [Ros2-foxy](#ros2-foxy)
  - [1.创建工作空间](#1创建工作空间)
  - [2.编译代码(colcon)](#2编译代码colcon)
    - [只编译一个包](#只编译一个包)
    - [不编译测试单元](#不编译测试单元)
    - [运行编译的包的测试](#运行编译的包的测试)
    - [允许通过更改src下的部分文件来改变install（直接用这个）](#允许通过更改src下的部分文件来改变install直接用这个)
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
- [代码编写](#代码编写)
  - [1.节点创建](#1节点创建)
  - [2. 话题与服务](#2-话题与服务)
    - [topic发布者](#topic发布者)
    - [topic订阅者](#topic订阅者)


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

### 只编译一个包
`colcon build --packages-select <pkg>`

### 不编译测试单元
`colcon build --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=0`

### 运行编译的包的测试
`colcon test`

### 允许通过更改src下的部分文件来改变install（直接用这个）
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
    Ultrasonic_node = Node("Ultrasonic_node")  #新建节点
    Ultrasonic_node.get_logger().info("hello world")  
    rclpy.spin(Ultrasonic_node)    #保持节点运行
    rclpy.shudown()     #关闭rclpy
```
4. 修改/ssr_pkg/setup.py
```
'console_scripts': [
            "Ultrasonic_node = ssr_pkg.Ultrasonic_node:main"
        ],
```
- ssr_pkg 包名
- Ultrasonic_node 脚本文件名
- main 入口函数
5. 编译一下
`colcon build --symlink-instal`
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

    Attributes:
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
    description:入口函数

    Args:

    Returns:
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
