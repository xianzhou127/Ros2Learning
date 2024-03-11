#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node

def main(args=None):
    '''
    1. 导入库文件
    2. 初始化客户端库
    3. 新建节点
    4. spin循环节点
    5. 关闭客户端库
    '''
    rclpy.init(args=args)   #初始化节点
    Ultrasonic_node = Node("Ultrasonic_node")  #新建节点
    Ultrasonic_node.get_logger().info("hello world")  
    rclpy.spin(Ultrasonic_node)    #保持节点运行
    rclpy.shudown()     #关闭rclpy