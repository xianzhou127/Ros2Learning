from setuptools import setup
from glob import glob
import os

package_name = 'atr_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),    #添加这一行
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),    #启动Rviz添加这一行
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),    #启动Gazebo自带地图添加这一行
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xianzhou',
    maintainer_email='1544453976@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "Motor_node = atr_pkg.Motor_node:main"
        ],
    },
    
)
