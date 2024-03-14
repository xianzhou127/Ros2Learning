#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_msgs.srv import MoveRobot

class AtrNode(Node):
    '''
    define a class of atr node
    '''
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("subscriber motor_node online")
        self.pose = 0
        self.sub_pos = self.create_subscription(String,"Position",self.__sub_callback,qos_profile=10)
        self.moverobot_srv_ = self.create_service(MoveRobot,"move_robot",self.__moverobot_callback)
    def __sub_callback(self,msg):
        self.get_logger().info(msg.data)

    def __moverobot_callback(self,request,response):
        '''
        服务端回调函数
        获得前进距离，并返回现在位置
        request\response-MoveRobot类型
        '''
        self.get_logger().info("get requestions,moving forward %f meters"%request.distance)
        self.pose += request.distance
        response.pose = self.pose
        return response
    
def main(args=None):
    rclpy.init(args=args)
    Motor_node = AtrNode("Motor_node")

    rclpy.spin(Motor_node)    #保持节点运行
    rclpy.shudown()     #关闭rclpy  
