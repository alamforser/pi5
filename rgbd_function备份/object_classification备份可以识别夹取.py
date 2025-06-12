#!/usr/bin/python3
# coding=utf8
# @data:2023/12/25
# 通过深度图识别物体进行分类(classify objects through depth map recognition)
# 机械臂向下识别(mechanical arm recognition downwards)
# 可以识别长方体，球，圆柱体，以及他们的颜色(it can recognize cuboids, spheres, cylinders, and their respective colors)
import os
import cv2
import time
import math
import rclpy
import queue
import signal
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from sdk import common, fps
from interfaces.srv import SetStringList
from kinematics import kinematics_control
from std_srvs.srv import Trigger, SetBool
# from xf_mic_asr_offline import voice_play
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics_msgs.srv import SetJointValue, SetRobotPose
from kinematics.kinematics_control import set_joint_value_target
from servo_controller.bus_servo_control import set_servo_position
from example.rgbd_function.position_change_detect import position_reorder
from servo_controller.action_group_controller import ActionGroupController

def depth_pixel_to_camera(pixel_coords, intrinsic_matrix):
    fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    return np.array([x, y, z])


class ObjectClassificationNode(Node):
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.0],  #0.011
        [0.0, -1.0, 0.0, 0.037],  # 0.045
        [0.0, 0.0, 0.0, 1.0]
    ]
    pick_offset = [0.01, 0.01, 0.0, -0.01, 0.0]  # x1, x2, y1, y2, z
    '''
                
                 x1(+)
        y1(+)   center    y2(-)
                  x2

                  arm
                  car
    '''

    def __init__(self, name):
        print("开始初始化节点")
        try:
            rclpy.init()
            print("rclpy初始化成功")
            super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
            print("节点创建成功")
            
            # 在每个关键步骤后添加日志
            self.fps = fps.FPS()
            print("FPS初始化成功")
            
            # 检查环境变量
            try:
                self.language = os.environ['ASR_LANGUAGE']
                print(f"获取到ASR_LANGUAGE: {self.language}")
            except KeyError:
                print("错误: 未设置ASR_LANGUAGE环境变量")
                self.language = "zh"  # 设置默认值
            
            # 先定义callback_group
            timer_cb_group = ReentrantCallbackGroup()
            print("ReentrantCallbackGroup创建成功")
            
            # 继续其他初始化...
            self.moving = False
            self.count = 0
            self.running = True
            self.start = False
            self.shapes = None
            self.colors = None
            self.target_shapes = ''
            self.roi = [120, 300, 120, 520]  #self.roi = [y_min, y_max, x_min, x_max]
            self.endpoint = None
            self.last_position = 0, 0
            self.last_object_info_list = []
            signal.signal(signal.SIGINT, self.shutdown)
            print("基本变量初始化完成")
            
            self.image_queue = queue.Queue(maxsize=2)
            print("队列创建成功")
            
            print("准备获取参数")
            self.debug = self.get_parameter('debug').value
            self.plane_distance = self.get_parameter('plane_distance').value
            print(f"参数获取成功: debug={self.debug}, plane_distance={self.plane_distance}")
            
            print("准备读取YAML数据")
            self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
            print("YAML数据读取成功")
            
            print("准备创建发布者")
            self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
            self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
            print("发布者创建成功")

            print("准备创建服务")
            self.create_service(Trigger, '~/start', self.start_srv_callback)
            self.create_service(Trigger, '~/stop', self.stop_srv_callback)
            self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)
            self.create_service(SetStringList, '~/set_color', self.set_color_srv_callback)
            print("服务创建成功")
            
            print("准备创建订阅者")
            rgb_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/rgb0/image')
            depth_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/depth0/image_raw')
            info_sub = message_filters.Subscriber(self, CameraInfo, '/ascamera/camera_publisher/depth0/camera_info')
            print("订阅者创建成功")

            print("准备创建同步器")
            sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
            sync.registerCallback(self.multi_callback)
            print("同步器创建成功")
            
            print("准备等待controller_manager/init_finish服务")
            self.client = self.create_client(Trigger, '/controller_manager/init_finish')
            self.client.wait_for_service()
            print("controller_manager/init_finish服务可用")
            
            print("准备等待depth_cam/set_ldp_enable服务")
            # self.client = self.create_client(SetBool, '/depth_cam/set_ldp_enable')
            # self.client.wait_for_service()
            print("depth_cam/set_ldp_enable服务可用")
           
            print("准备创建kinematics/set_joint_value_target客户端")
            self.set_joint_value_target_client = self.create_client(SetJointValue, '/kinematics/set_joint_value_target', callback_group=timer_cb_group)
            self.set_joint_value_target_client.wait_for_service()
            print("kinematics/set_joint_value_target服务可用")
            
            print("准备创建kinematics/set_pose_target客户端")
            self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
            self.kinematics_client.wait_for_service()
            print("kinematics/set_pose_target服务可用")

            print("准备创建ActionGroupController")
            self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/arm_pc/ActionGroups')
            print("ActionGroupController创建成功")

            print("准备创建定时器")
            self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
            print("定时器创建成功")
        except Exception as e:
            print(f"初始化过程中出错: {str(e)}")
            import traceback
            traceback.print_exc()

    def init_process(self):
        self.timer.cancel()
        self.get_logger().info('\033[1;32m%s\033[0m' % "程序已执行到init_process")
        # msg = SetBool.Request()
        # msg.data = False
        # self.send_request(self.client, msg)

        self.goto_default()
        # 记录程序执行到此处的日志信息
        self.get_logger().info('\033[1;32m%s\033[0m' % "程序已执行到goto_default之后")
        

        if self.get_parameter('start').value:
            if self.get_parameter('category').value == 'shape':
                msg = SetStringList.Request()
                msg.data = ['sphere', 'cuboid', 'cylinder']
                self.set_shape_srv_callback(msg, SetStringList.Response())
            else:
                msg = SetStringList.Request()
                msg.data = ['red', 'green', 'blue']
                self.set_color_srv_callback(msg, SetStringList.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")
        # 确保程序能够正常退出
        cv2.destroyAllWindows()  # 关闭所有OpenCV窗口
        rclpy.shutdown()  # 关闭ROS2
        import sys
        sys.exit(0)  # 强制退出程序

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def set_shape_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_shape")
        self.colors = None
        self.shapes = request.data
        self.start = True
        response.success = True
        response.message = "set_shape"
        return response

    def set_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_color")
        self.shapes = None
        self.colors = request.data
        self.start = True
        response.success = True
        response.message = "set_color"
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop")
        self.start = False
        self.colors = None
        self.shapes = None
        self.moving = False
        self.count = 0
        self.target_shapes = ''
        self.last_position = 0, 0
        self.last_object_info_list = []
        response.success = True
        response.message = "stop"
        return response

    def goto_default(self):
        msg = set_joint_value_target([500.0, 470.0, 220.0, 90.0, 500.0])
        # 记录日志，表示正在前往默认位置
        self.get_logger().info('\033[1;32m%s\033[0m' % "机械臂初始化")
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        # set_servo_position(self.joints_pub, 1, ((1, 500), (2, 470), (3, 220), (4, 90), (5, 500), (10, 200)))
        # set_servo_position(self.joints_pub, 1, ((1, 500), (2, 420), (3, 216), (4, 146), (5, 500), (10, 360)))
        #set_servo_position(self.joints_pub, 1, ((1, 500), (2, 360), (3, 310), (4, 136), (5, 500), (10, 360)))  #相机水平
        #set_servo_position(self.joints_pub, 1, ((1, 500), (2, 483), (3, 335), (4, 55), (5, 500), (10, 360)))  #相机有倾斜角度
        # set_servo_position(self.joints_pub, 1, ((1, 500), (2, 522), (3, 318), (4, 51), (5, 500), (10, 360)))  #相机有倾斜角度没有垫高
        set_servo_position(self.joints_pub, 1, ((1, 500), (2, 648), (3, 183), (4, 91), (5, 500), (10, 360)))  #相机有倾斜角度没有垫高角度效果更佳。
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])

    def move(self, obejct_info):
        shape, pose_t = obejct_info[:2]
        color, angle = obejct_info[-2:]
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
        time.sleep(1)
        if 'sphere' in shape:
            offset_z = -0.015 + self.pick_offset[-1]
        elif 'cylinder' in shape:
            offset_z = 0.02 + self.pick_offset[-1]
        else:
            offset_z = 0.01 + self.pick_offset[-1]
        if pose_t[0] > 0.21:
            offset_x = self.pick_offset[0]
        else:
            offset_x = self.pick_offset[1]
        if pose_t[1] > 0:
            offset_y = self.pick_offset[2]
        else:
            offset_y = self.pick_offset[3]
        pose_t[0] += offset_x
        pose_t[1] += offset_y
        pose_t[2] += offset_z
        msg = kinematics_control.set_pose_target(pose_t, 85)
        res1 = self.send_request(self.kinematics_client, msg)
        if res1.pulse:
            servo_data = res1.pulse
            set_servo_position(self.joints_pub, 1.5, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, servo_data[4])))
            time.sleep(1.5)
        pose_t[2] -= 0.05
        msg = kinematics_control.set_pose_target(pose_t, 85)
        res2 = self.send_request(self.kinematics_client, msg)
        if angle != 0:
            if 'sphere' in shape or ('cylinder' in shape and 'cylinder_horizontal_' not in shape):
                angle = 500
            else:
                angle = angle % 180
                angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
                
                # self.get_logger().info(str([angle, shape]))
                # if angle == 90:
                    # angle = 0
                angle = 500 + int(1000 * (angle + res2.rpy[-1]) / 240)
        else:
            angle = 500
        if res2.pulse:
            servo_data = res2.pulse
            set_servo_position(self.joints_pub, 0.5, ((5, angle),))
            time.sleep(0.5)
            set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, angle)))
            time.sleep(1)
            if shape == "sphere":
                set_servo_position(self.joints_pub, 0.6, ((10, 700),))
            else:
                set_servo_position(self.joints_pub, 0.6, ((10, 550),))
            time.sleep(0.6)
        if res1.pulse:
            servo_data = res1.pulse
            set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, angle)))
            time.sleep(1)
        set_servo_position(self.joints_pub, 1, ((1, 500), (2, 640), (3, 150), (4, 130), (5, 500), (10, 650)))
        time.sleep(1)
        if self.colors is None:
            self.get_logger().info('shape: %s' % shape.split("_")[0])
            if "sphere" in shape:
                self.controller.run_action("target_1")
            if "cylinder" in shape:
                self.controller.run_action("target_2")
            if "cuboid" in shape:
                self.controller.run_action("target_3")
        else:
            color = self.color_comparison(color)
            self.get_logger().info('color: %s' % color)
            if "red" == color:
                self.controller.run_action("target_1")
            if "blue" == color:
                self.controller.run_action("target_3")
        self.goto_default()
        time.sleep(2)
        self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def cal_position(self, x, y, depth, intrinsic_matrix):
        try:
            self.get_logger().info(f"开始计算位置: x={x}, y={y}, depth={depth}")
            
            # 检查输入参数
            if depth <= 0:
                self.get_logger().warning(f"无效深度值: {depth}")
                return None
            
            # 检查intrinsic_matrix
            self.get_logger().info(f"相机内参矩阵: {intrinsic_matrix}")
            
            # 检查depth_pixel_to_camera函数输入
            self.get_logger().info(f"调用depth_pixel_to_camera，参数: {[x, y, depth / 1000]}")
            position = depth_pixel_to_camera([x, y, depth / 1000], intrinsic_matrix)
            self.get_logger().info(f"depth_pixel_to_camera返回: {position}")
            
            # 检查endpoint是否初始化
            self.get_logger().info(f"self.endpoint类型: {type(self.endpoint)}")
            if self.endpoint is None:
                self.get_logger().error("self.endpoint未初始化")
                return None
            
            # 这里可能出现NoneType错误
            self.get_logger().info(f"准备执行: position[0] -= 0.01，position[0]={position[0]}")
            position[0] -= 0.01
            self.get_logger().info(f"position[0]减去0.01后: {position[0]}")
            
            # 检查hand2cam_tf_matrix
            self.get_logger().info(f"self.hand2cam_tf_matrix类型: {type(self.hand2cam_tf_matrix)}")
            
            # 转换到机器人坐标系
            self.get_logger().info("准备执行坐标转换")
            pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
            self.get_logger().info("第一次矩阵乘法完成")
            world_pose = np.matmul(self.endpoint, pose_end)
            self.get_logger().info("第二次矩阵乘法完成")
            pose_t, pose_r = common.mat_to_xyz_euler(world_pose)
            self.get_logger().info(f"位置计算完成: {pose_t}")
            
            return pose_t
        except Exception as e:
            self.get_logger().error(f"位置计算错误: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def get_min_distance(self, depth_image):
        ih, iw = depth_image.shape[:2]
        # 屏蔽掉一些区域，降低识别条件，使识别跟可靠(mask certain areas to lower recognition conditions and enhance reliability)
        depth_image[:, :self.roi[2]] = np.array([[1000, ] * self.roi[2]] * ih)
        depth_image[:, self.roi[3]:] = np.array([[1000, ] * (iw - self.roi[3])] * ih)
        depth_image[self.roi[1]:, :] = np.array([[1000, ] * iw] * (ih - self.roi[1]))
        depth_image[:self.roi[0], :] = np.array([[1000, ] * iw] * self.roi[0])
        depth = np.copy(depth_image).reshape((-1,))
        depth[depth <= 0] = 55555  # 距离为0可能是进入死区，或者颜色问题识别不到，将距离赋一个大值(a distance of 0 may indicate entry into a dead zone or failure to recognize due to color issues. Assign a large value to the distance)

        min_index = np.argmin(depth)  # 距离最小的像素(the pixel with the minimum distance)
        min_y = min_index // iw
        min_x = min_index - min_y * iw

        min_dist = depth_image[min_y, min_x]  # 获取距离摄像头最近的物体的距离(get the distance of the object that is closest to the camera)
        return min_dist

    def get_contours(self, depth_image, min_dist):
        try:
            # 检查self.plane_distance是否为None
            if self.plane_distance is None:
                self.get_logger().error("self.plane_distance为None，无法处理")
                # 设置一个默认值
                self.plane_distance = 1000  # 使用一个合理的默认值
            
            self.get_logger().info(f"plane_distance值: {self.plane_distance}")
            
            # 将深度值大于平面距离的像素置0
            depth_image = np.where(depth_image > self.plane_distance - 10, 0, depth_image)
            self.get_logger().info("第一次深度过滤完成")
            
            # 将深度值大于最小距离+40mm的像素置0
            depth_image = np.where(depth_image > min_dist + 40, 0, depth_image)
            self.get_logger().info("第二次深度过滤完成")
            
            # 归一化深度图像
            sim_depth_image_sort = np.clip(depth_image, 0, self.plane_distance - 10).astype(np.float64) / (self.plane_distance - 10) * 255
            depth_gray = sim_depth_image_sort.astype(np.uint8)
            self.get_logger().info("深度图像归一化完成")
            
            # 二值化
            _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
            self.get_logger().info("深度图像二值化完成")
            
            # 查找轮廓
            contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            self.get_logger().info(f"找到{len(contours)}个轮廓")
            
            return contours
        except Exception as e:
            self.get_logger().error(f"获取轮廓时出错: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return []

    def shape_recognition(self, rgb_image, depth_image, depth_color_map, intrinsic_matrix, min_dist):
        self.get_logger().info(f"开始形状识别，最小距离: {min_dist}")
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        if min_dist <= 300:  # 大于这个值说明已经低于地面了，可能检测有误
            self.get_logger().info("最小距离在有效范围内，继续处理")
            sphere_index = 0
            cuboid_index = 0
            cylinder_index = 0
            cylinder_horizontal_index = 0
            
            self.get_logger().info("准备获取轮廓")
            contours = self.get_contours(depth_image, min_dist)
            self.get_logger().info(f"获取到{len(contours)}个轮廓")
            
            for i, obj in enumerate(contours):
                self.get_logger().info(f"处理轮廓 {i+1}/{len(contours)}")
                area = cv2.contourArea(obj)
                self.get_logger().info(f"轮廓面积: {area}")
                if area < 300:
                    self.get_logger().info("轮廓面积太小，跳过")
                    continue
                    
                # 计算轮廓周长
                perimeter = cv2.arcLength(obj, True)
                self.get_logger().info(f"轮廓周长: {perimeter}")
                
                # 获取轮廓角点坐标
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)
                
                # 获取角点数量
                CornerNum = len(approx)
                self.get_logger().info(f"角点数量: {CornerNum}")
                
                # 获取最小包围圆
                (cx, cy), r = cv2.minEnclosingCircle(obj)
                self.get_logger().info(f"物体中心点: ({cx}, {cy}), 半径: {r}")
                
                # 获取最小包围矩形
                center, (width, height), angle = cv2.minAreaRect(obj)
                self.get_logger().info(f"矩形中心: {center}, 宽高: ({width}, {height}), 角度: {angle}")
                
                # 角度调整
                if angle < -45:
                    angle += 89
                if width > height and width / height > 1.5:
                    angle = angle + 90
                self.get_logger().info(f"调整后角度: {angle}")
                    
                # 获取深度和位置
                try:
                    self.get_logger().info(f"准备获取深度值，坐标: ({int(cy)}, {int(cx)})")
                    if 0 <= int(cy) < depth_image.shape[0] and 0 <= int(cx) < depth_image.shape[1]:
                        depth = depth_image[int(cy), int(cx)]
                        self.get_logger().info(f"获取到深度值: {depth}")
                    else:
                        self.get_logger().warning(f"坐标超出范围: ({int(cy)}, {int(cx)}), 图像尺寸: {depth_image.shape}")
                        continue
                    
                    self.get_logger().info("准备计算3D位置")
                    position = self.cal_position(cx, cy, depth, intrinsic_matrix)
                    
                    if position is None:
                        self.get_logger().warning("位置计算返回None，跳过此物体")
                        continue
                    
                    self.get_logger().info(f"3D位置: {position}")
                    
                    # 创建掩码计算标准差
                    x, y, w, h = cv2.boundingRect(approx)
                    self.get_logger().info(f"边界矩形: x={x}, y={y}, w={w}, h={h}")
                    
                    mask = np.full((image_height, image_width), 0, dtype=np.uint8)
                    cv2.drawContours(mask, [obj], -1, (255), cv2.FILLED)
                    
                    # 计算深度标准差
                    depth_image_mask = np.where(depth_image == 0, np.nan, depth_image)
                    depth_std = np.nanstd(mask)
                    self.get_logger().info(f"深度标准差: {depth_std}")
                    
                    # 根据深度标准差和角点数量判断形状
                    objType = None
                    if depth_std > 30.0 and CornerNum > 4:
                        sphere_index += 1
                        angle = 0
                        objType = 'sphere_' + str(sphere_index)
                        self.get_logger().info(f"识别为球体: {objType}")
                    elif depth_std > 24.0:
                        cuboid_index += 1
                        objType = "cuboid_" + str(cuboid_index)
                        self.get_logger().info(f"识别为立方体: {objType}")
                    elif depth_std > 18.0:
                        if abs(width/height - 1) < 0.2:
                            cuboid_index += 1
                            objType = "cuboid_" + str(cuboid_index)
                            self.get_logger().info(f"识别为立方体: {objType}")
                        else:
                            cylinder_horizontal_index += 1
                            objType = "cylinder_horizontal_" + str(cylinder_horizontal_index)
                            self.get_logger().info(f"识别为水平圆柱体: {objType}")
                    else:
                        cylinder_index += 1
                        angle = 0
                        objType = "cylinder_" + str(cylinder_index)
                        self.get_logger().info(f"识别为圆柱体: {objType}")
                    
                    # 保存识别结果
                    if objType is not None:
                        rgb_value = rgb_image[int(center[1]), int(center[0])]
                        self.get_logger().info(f"物体RGB值: {rgb_value}")
                        object_info_list.append([objType, position, depth, [x, y, w, h, center, width, height], rgb_value, angle])
                        self.get_logger().info(f"已添加到物体列表，当前列表长度: {len(object_info_list)}")
                        
                except Exception as e:
                    self.get_logger().error(f"处理物体时出错: {str(e)}")
                    import traceback
                    self.get_logger().error(traceback.format_exc())
                    continue
        else:
            self.get_logger().info(f"最小距离 {min_dist} 超出有效范围，跳过形状识别")
                
        self.get_logger().info(f"形状识别完成，识别到 {len(object_info_list)} 个物体")
        return object_info_list

    def color_comparison(self, rgb):
        if rgb[0] > rgb[1] and rgb[0] > rgb[2]:
            return 'red'
        elif rgb[2] > rgb[1] and rgb[2] > rgb[1]:
            return 'blue'
        else:
            return None

    def main(self):
        count = 0
        while self.running:
            try:
                # 确保start为True
                self.start = True
                
                # 直接设置目标形状（临时解决方案）
                if self.target_shapes is None or self.target_shapes == '':
                    self.target_shapes = 'cuboid'  # 直接指定要夹取的形状
                    self.get_logger().info(f"直接设置目标形状: {self.target_shapes}")
                
                # 确保shapes和colors至少是空列表
                if self.shapes is None:
                    self.shapes = ["cuboid"]  # 只包含我们想要的形状
                    self.get_logger().info(f"设置指定shapes: {self.shapes}")
                if self.colors is None:
                    self.colors = []  # 空列表也是可迭代的
                    self.get_logger().info(f"设置默认colors: {self.colors}")
                
                # 添加日志，显示当前循环开始
                self.get_logger().info(f"主循环开始，self.start={self.start}, self.moving={self.moving}")
                
                try:
                    ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    if not self.running:
                        break
                    else:
                        continue
                try:
                    rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                    depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                    
                    # 详细打印图像尺寸
                    self.get_logger().info(f"RGB图像尺寸: 高={rgb_image.shape[0]}, 宽={rgb_image.shape[1]}, 通道={rgb_image.shape[2]}")
                    self.get_logger().info(f"深度图像尺寸: 高={depth_image.shape[0]}, 宽={depth_image.shape[1]}")
                    
                    # cv2.imshow('rgb', cv2.applyColorMap(depth_image.astype(np.uint8), cv2.COLORMAP_JET))
                    depth_image = depth_image.copy()
                    min_dist = self.get_min_distance(depth_image)
                    
                    # 添加日志，显示最小距离
                    self.get_logger().info(f"获取到最小距离: {min_dist}")
                    
                    if self.debug:
                        count += 1
                        self.get_logger().info(str(min_dist))
                        if count > 50:
                            count = 0
                            data = {'/**': {'ros__parameters': {'plane_distance': {}}}}
                            data['/**']['ros__parameters']['plane_distance'] = int(min_dist)
                            common.save_yaml_data(data, os.path.join(
                                os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')),
                                'config/object_classification_plane_distance.yaml'))
                            msg = BuzzerState()
                            msg.freq = 1900
                            msg.on_time = 0.2
                            msg.off_time = 0.01
                            msg.repeat = 1
                            self.buzzer_pub.publish(msg)
                            self.debug = False
                    else:
                        #像素值限制在0到350的范围内, 将深度图像的像素值限制和归一化到0到255的范围内
                        sim_depth_image = np.clip(depth_image, 0, 350).astype(np.float64) / 350 * 255
                        
                        depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                        
                        # 添加日志，显示moving状态
                        self.get_logger().info(f"准备检查moving状态: {self.moving}")
                        
                        if not self.moving:
                            # 添加日志，显示进入形状识别
                            self.get_logger().info("进入形状识别流程")
                            
                            object_info_list = self.shape_recognition(rgb_image, depth_image, depth_color_map, depth_camera_info.k, min_dist)
                            
                            # 添加日志，显示识别结果
                            self.get_logger().info(f"形状识别完成，识别到 {len(object_info_list) if object_info_list else 0} 个物体")
                            
                            # 添加日志，显示start状态
                            self.get_logger().info(f"检查self.start状态: {self.start}")
                            
                            if self.start:
                                # 添加日志，显示进入start条件
                                self.get_logger().info("进入self.start=True条件分支")
                                
                                reorder_object_info_list = object_info_list
                                if object_info_list:
                                    self.get_logger().info("object_info_list不为空")
                                    if self.last_object_info_list:
                                        # 添加日志，显示进入重排序
                                        self.get_logger().info("存在上一次物体列表，准备重排序")
                                        # 对比上一次的物体的位置来重新排序
                                        reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, 20)
                                        self.get_logger().info("重排序完成")
                                
                                # 添加日志，显示重排序结果
                                self.get_logger().info(f"重排序后物体列表长度: {len(reorder_object_info_list) if reorder_object_info_list else 0}")
                                
                                if reorder_object_info_list:
                                    self.get_logger().info("reorder_object_info_list不为空")
                                    
                                    # 添加日志，显示target_shapes状态
                                    self.get_logger().info(f"检查self.target_shapes状态: {self.target_shapes}")
                                    
                                    if not self.target_shapes:
                                        self.get_logger().info("没有目标形状，准备选择目标")
                                        
                                        # 添加日志，显示shapes和colors
                                        self.get_logger().info(f"self.shapes={self.shapes}, self.colors={self.colors}")
                                        
                                        if self.shapes is not None:
                                            indices = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] in self.shapes]
                                            self.get_logger().info(f"基于形状过滤，符合条件的索引: {indices}")
                                        else:
                                            indices = [i for i, info in enumerate(reorder_object_info_list) if self.color_comparison(info[-2]) in self.colors]
                                            self.get_logger().info(f"基于颜色过滤，符合条件的索引: {indices}")
                                        
                                        if indices:
                                            min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][2])
                                            self.target_shapes = reorder_object_info_list[min_depth_index][0].split('_')[0]
                                            self.get_logger().info(f"选择目标形状: {self.target_shapes}, 索引: {min_depth_index}")
                                        else:
                                            self.get_logger().info("没有符合条件的物体")
                                    else:
                                        self.get_logger().info(f"已有目标形状: {self.target_shapes}，寻找匹配物体")
                                        
                                        target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] == self.target_shapes]
                                        self.get_logger().info(f"匹配目标形状的索引: {target_index}")
                                        
                                        if target_index:
                                            target_index = target_index[0]
                                            obejct_info = reorder_object_info_list[target_index]
                                            x, y, w, h, center, width, height = obejct_info[3]
                                            angle = obejct_info[-1]
                                            
                                            # 添加日志，显示物体信息
                                            self.get_logger().info(f"目标物体信息: 中心={center}, 宽高=({width}, {height}), 角度={angle}")
                                            
                                            cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                        (0, 0, 0), 2, cv2.LINE_AA)
                                            cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                        (255, 255, 255), 1)
                                            cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1,
                                                             (0, 0, 255), 2, cv2.LINE_AA)
                                            position = obejct_info[1]
                                            
                                            # 添加日志，显示位置信息
                                            self.get_logger().info(f"当前位置: {position}, 上次位置: {self.last_position}")
                                            
                                            # 修正欧氏距离计算
                                            e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2) + 
                                                                        pow(self.last_position[1] - position[1], 2)), 5)
                                            
                                            # 添加日志，显示距离和计数
                                            self.get_logger().info(f"欧氏距离: {e_distance}, 阈值: 0.005, 当前计数: {self.count}")
                                            
                                            if e_distance <= 0.005:
                                                self.count += 1
                                                self.get_logger().info(f"物体稳定，计数增加到: {self.count}")
                                            else:
                                                self.count = 0
                                                self.get_logger().info(f"物体不稳定，计数重置为0")
                                            
                                            # 添加日志，显示计数状态
                                            self.get_logger().info(f"检查计数: {self.count} > 5 = {self.count > 5}")
                                            
                                            if self.count > 5:
                                                self.get_logger().info("计数超过5，准备移动")
                                                self.count = 0
                                                self.target_shapes = None
                                                self.moving = True
                                                threading.Thread(target=self.move, args=(obejct_info,)).start()
                                                self.get_logger().info("移动线程已启动")
                                            self.last_position = position
                                        else:
                                            self.get_logger().info(f"没有找到目标形状: {self.target_shapes}，重置目标")
                                            self.target_shapes = None

                                self.last_object_info_list = reorder_object_info_list
                            else:
                                self.get_logger().info("self.start为False，跳过处理")
                        else:
                            self.get_logger().info("self.moving为True，跳过形状识别")

                    # 检查原始图像格式
                    # print(f"RGB图像格式: {rgb_image.shape}")

                    # 尝试不进行颜色空间转换
                    # bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                    bgr_image = rgb_image.copy()  # 直接使用原始图像

                    # 在ROI区域绘制矩形
                    cv2.rectangle(bgr_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (255, 255, 0), 1)

                    self.fps.update()
                    result_image = np.concatenate([depth_color_map, bgr_image], axis=1)
                    cv2.imshow("depth", result_image)
                    # 检查窗口实际尺寸
                    self.check_window_size("depth")
                    # 等待按键
                    key = cv2.waitKey(1)
                    if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                        self.running = False
                except Exception as e:
                    self.get_logger().error(f"处理图像时出错: {str(e)}")
                    import traceback
                    self.get_logger().error(traceback.format_exc())
            except Exception as e:
                self.get_logger().error(f"主循环出错: {str(e)}")
                import traceback
                self.get_logger().error(traceback.format_exc())
            
            # 添加日志，显示一次循环结束
            self.get_logger().info("主循环一次迭代结束")
            
        rclpy.shutdown()

    def check_window_size(self, window_name="depth"):
        """检查OpenCV窗口的实际尺寸"""
        try:
            # 获取窗口属性
            x, y, w, h = cv2.getWindowImageRect(window_name)
            self.get_logger().info(f"窗口'{window_name}'的实际尺寸: 位置=({x},{y}), 尺寸={w}x{h}")
            return w, h
        except Exception as e:
            self.get_logger().error(f"获取窗口尺寸时出错: {str(e)}")
            return None, None

def main():
    print("开始创建ObjectClassificationNode节点")
    node = ObjectClassificationNode('object_classification')
    print("节点创建完成，准备创建执行器")
    executor = MultiThreadedExecutor()
    print("执行器创建完成，准备添加节点")
    executor.add_node(node)
    print("节点添加完成，准备开始spin")
    executor.spin()
    print("spin结束，准备销毁节点")
    node.destroy_node()
    print("节点销毁完成")

if __name__ == "__main__":
    main()

