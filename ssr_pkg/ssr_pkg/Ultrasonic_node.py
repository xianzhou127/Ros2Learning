#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  #与ros1不同，在创建pkg时不需要写std_msgs的依赖项
from robot_msgs.msg import RobotPose
from robot_msgs.srv import MoveRobot

#OOP编程
class SsrNode(Node):
    '''
    define a class of ssr nodes

    Attributes:
    '''
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("This is %s,now the robots is located 0 meters away"%(name))
        self.pub_pos = self.create_publisher(String,'Position',qos_profile=10)
        # 客户端
        self.moverobot_client_ = self.create_client(MoveRobot,"move_robot")

    def publish(self,period_sec,msg):
        self.msg = msg
        self.timer = self.create_timer(timer_period_sec=period_sec,callback=self.__pub_callback)  #回调函数这里不能传参

    def move_robot(self,distance):
        '''
        客户端发送请求函数
        '''
        while rclpy.ok() and self.moverobot_client_.wait_for_service(1)==False:
            self.get_logger().info(f"Waiting for service....")

        #创建请求对象,后面的.Request()意思是调用这个函数，
        request = MoveRobot.Request()
        request.distance = distance
        self.get_logger().info(f"Requesting robot to move forward {distance} meters")
        self.moverobot_client_.call_async(request).add_done_callback(self.move_robot_callback__)

    def move_robot_callback__(self,response):
        '''
        客户端回调函数
        response为服务端返回值
        ''' 
        result = response.result()
        self.get_logger().info(f"now located {result.pose} meters away")
        # 调用

    # while rclpy.ok():
    #     # 非阻塞处理回调函数
    #     rclpy.spin_once(minimal_client)

    #     # 检测是否收到应答完成服务
    #     if minimal_client.future.done():
    #         try:
    #             # 应答结果
    #             response = minimal_client.future.result()
    #         except Exception as e:
    #             # 显示异常
    #             minimal_client.get_logger().info('Service call failed %r' % (e,))
    #         else:
    #             # 显示请求-应答状态
    #             minimal_client.get_logger().info(
    #                 'Result of add_three_ints: for %d + %d + %d = %d' %                                # CHANGE
    #                 (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum))  # CHANGE
    #         break

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

    Pose = RobotPose()
    Pose.pose.position.x = 10.0    #前进10m
    Ultrasonic_node.move_robot(Pose.pose.position.x)
    rclpy.spin(Ultrasonic_node)    #保持节点运行
    rclpy.shudown()     #关闭rclpy


    '''
    1. 导入库文件
    2. 初始化客户端库
    3. 新建节点
    4. spin循环节点
    5. 关闭客户端库
    '''