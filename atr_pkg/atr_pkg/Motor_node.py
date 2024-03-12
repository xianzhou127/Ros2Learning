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
