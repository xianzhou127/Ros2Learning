# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess # ExecuteProcess用于在launch文件中启动一个进程
# from ament_index_python.packages import get_package_share_directory #启动rviz,gazebo用
# import os   #启动rviz,gazebo用
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

    # rviz_name = 'display.rviz'    #rviz的配置文件
    # # 获取功能包路径（注意，这个路径是在工作空间的install文件夹里
    # package_name = 'atr_pkg'
    # pkg_description = get_package_share_directory(package_name)
    # # 声明文件路径，os.path.join将口号内的str用\连接，组成路径
    # rviz_config_path = os.path.join(pkg_description,
    #                                 'rviz',
    #                                 rviz_name)
    # # 启动rviz2
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=['-d', rviz_config_path]
    # )

    # # 定义启动参数
    # world = DeclareLaunchArgument('world', default='file://$(art_pkg)/worlds/your_world.world.tar.gz')
    # 启动Gazebo ROS节点
    # gazebo = Node(
    #     package='atr_pkg',
    #     executable='gazebo',
    #     arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    # )

    # world_file_name = 'empty_world.world'
    # package_name = 'atr_pkg'
    # pkg_description = get_package_share_directory(package_name)
    # world = os.path.join(pkg_description,'world',world_file_name)
    # gazebo = ExecuteProcess(
    #     cmd=['gazebo', '--verbose',world],
    #     output='screen'
    #     )
    # # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [motor_node, Ultrasonic_node])
    
    return launch_description