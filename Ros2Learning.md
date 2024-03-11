# Ros2-foxy
[Ros2官网文档](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html)

[toc]

## 1.创建工作空间
`(base) xianzhou@xianzhou:~$ mkdir -p ~/dev_ws/src`

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

## 4.节点
### 通信方式
- 话题-topics
- 服务-services
- 动作-Action
- 参数-parameters
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
            "Ultrasonic_noede = ssr_pkg.Ultrasonic_node:main"
        ],
```
- ssr_pkg 包名
- Ultrasonic_node 脚本文件名
- main 入口函数
5. 编译一下
`colcon build --symlink-instal`
6. 运行节点
`ros2 run ssr_pkg Ultrasonic.py`