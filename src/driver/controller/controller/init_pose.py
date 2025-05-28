#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from servo_controller import action_group_controller
from servo_controller_msgs.msg import ServosPosition, ServoPosition

class InitPose(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)  # 允许未声明的参数
        self.get_logger().info('开始初始化节点...')
        
        namespace = self.get_namespace()
        if namespace == '/':
            namespace = ''
        self.get_logger().info(f'命名空间: {namespace}')
       
        self.get_logger().info('创建发布者...')
        self.servo_controller_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.joint_controller_pub = self.create_publisher(JointState, 'joint_controller', 1)
        self.get_logger().info('发布者创建完成')

        try:
            self.client = self.create_client(Trigger, namespace + '/controller_manager/init_finish')
            # 添加超时机制
            if not self.client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('服务不可用，但将继续执行')
            else:
                self.get_logger().info('服务连接成功')
        except Exception as e:
            self.get_logger().error(f'连接服务时出错: {str(e)}')

        # 读取配置参数(read configuration parameter)
        self.type = self.get_parameter('type').value
        self.get_logger().info(f'类型: {self.type}')
        
        if self.type == 'action':
            action = self.get_parameters_by_prefix(self.type)
            self.get_logger().info(f'动作参数: {action}')
            acg = action_group_controller.ActionGroupController(self.servo_controller_pub, '/home/ubuntu/software/arm_pc/ActionGroups')
            acg.run_action(action['action_name'].value)
            print("动作执行完成")
        elif self.type == 'servo':
            self.get_logger().info('设置伺服电机位置...')
            pulse = self.get_parameters_by_prefix(self.type)
            self.get_logger().info(f'脉冲参数: {pulse}')
            msg = ServosPosition()
            msg.duration = float(pulse['duration'].value)
            data = []   
            for i in ['id1', 'id2', 'id3', 'id4', 'id5', 'id9', 'id10']:
                servo = ServoPosition()
                servo.id = int(i[2:]) 
                servo.position = float(pulse[i].value)
                data.append(servo)
            msg.position_unit = 'pulse'
            msg.position = data
            self.get_logger().info(f'发布伺服位置消息: {msg}')
            self.servo_controller_pub.publish(msg)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.get_logger().info('初始化完成')

    def get_node_state(self, request, response):
        self.get_logger().info('收到状态查询请求')
        response.success = True
        self.get_logger().info('返回成功状态')
        return response

def main():
    print("主函数开始执行")
    node = InitPose('init_pose')
    node.get_logger().info('节点创建完成，开始spin')
    rclpy.spin(node)  # 循环等待ROS2退出(loop waiting for ROS2 to exit)
    node.get_logger().info('spin结束')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
