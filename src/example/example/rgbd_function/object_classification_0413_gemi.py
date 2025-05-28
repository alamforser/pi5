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
    # 防止深度值为0导致除零错误
    if pz <= 0:
       # print("Warning: Zero or negative depth value encountered in depth_pixel_to_camera.")
       return np.array([0.0, 0.0, 0.0]) # 返回原点或根据需要处理
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
    pick_offset = [0.0, 0.0, 0.0, -0.03, 0.0]  # x1, x2, y1, y2, z
    '''
                 x1(+)
        y1(+)   center    y2(-)
                  x2

                  arm
                  car
    '''

    def __init__(self, name):
        try:
            rclpy.init()
            super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
            
            self.fps = fps.FPS()
            
            try:
                self.language = os.environ['ASR_LANGUAGE']
            except KeyError:
                self.language = "zh"
            
            timer_cb_group = ReentrantCallbackGroup()
            
            self.moving = False
            self.count = 0
            self.running = True
            self.start = False
            self.shapes = None
            self.colors = None
            self.target_shapes = ''
            self.roi = [150, 330, 120, 520]  # ROI区域: [y_min, y_max, x_min, x_max]

            self.endpoint = None
            self.last_position = np.array([0.0, 0.0]) # 使用numpy数组
            self.last_object_info_list = []
            signal.signal(signal.SIGINT, self.shutdown)
            
            self.image_queue = queue.Queue(maxsize=2)
            
            self.debug = self.get_parameter('debug').value
            # 不再从参数获取固定的 plane_distance，将在 get_contours 中动态计算
            # self.plane_distance = self.get_parameter('plane_distance').value 
            self.plane_distance = None # 标记为 None

            # --- 添加用于动态背景估计和分割的参数 ---
            self.min_valid_depth = 100 # mm, 最小有效深度
            self.max_valid_depth = 300 # mm, 最大有效深度 (用于背景估计)
            self.object_height_threshold = 25 # mm, 物体需要比背景中位数"高"(深度值小)多少才被认为是物体 (关键参数，需要调试!)
            self.morph_open_kernel_size = (3, 3) # 开运算核大小
            self.morph_close_kernel_size = (5, 5) # 闭运算核大小
            self.min_contour_area = 300 # 最小轮廓面积阈值
            self.min_valid_contour_points = 10 # 计算代表性深度所需的最少有效点数

            # --- 添加用于形状分类的参数 (需要调试!) ---
            self.sphere_std_threshold = 45.0 # 球体深度标准差下限 (可能需要调高)
            self.cuboid_aspect_ratio_threshold = 1.5 # 立方体长宽比上限
            self.horizontal_cylinder_aspect_ratio_threshold = 1.8 # 水平圆柱长宽比下限
            self.cylinder_circularity_threshold = 0.7 # 竖直圆柱圆形度下限
            self.cylinder_default_std_threshold = 15.0 # 默认圆柱的标准差下限 (可能需要调整)

            self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
            
            self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
            self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
            
            self.create_service(Trigger, '~/start', self.start_srv_callback)
            self.create_service(Trigger, '~/stop', self.stop_srv_callback)
            self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)
            self.create_service(SetStringList, '~/set_color', self.set_color_srv_callback)
            
            rgb_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/rgb0/image')
            depth_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/depth0/image_raw')
            info_sub = message_filters.Subscriber(self, CameraInfo, '/ascamera/camera_publisher/depth0/camera_info')
            
            sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
            sync.registerCallback(self.multi_callback)
            
            self.client = self.create_client(Trigger, '/controller_manager/init_finish')
            self.client.wait_for_service()
            
            self.set_joint_value_target_client = self.create_client(SetJointValue, '/kinematics/set_joint_value_target', callback_group=timer_cb_group)
            self.set_joint_value_target_client.wait_for_service()
            
            self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
            self.kinematics_client.wait_for_service()

            self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/arm_pc/ActionGroups')
            
            self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
            self.get_logger().info("ObjectClassificationNode initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Error during initialization: {str(e)}")
            import traceback
            traceback.print_exc()

    def init_process(self):
        self.timer.cancel()
        self.goto_default()
        
        if self.get_parameter('start').value:
            if self.get_parameter('category').value == 'shape':
                msg = SetStringList.Request()
                msg.data = ['sphere', 'cuboid', 'cylinder']
                self.set_shape_srv_callback(msg, SetStringList.Response())
            else:
                msg = SetStringList.Request()
                msg.data = ['red', 'green', 'blue'] # 注意：你的color_comparison只识别红和蓝
                self.set_color_srv_callback(msg, SetStringList.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Node started and main loop running.')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "Shutdown signal received.")
        time.sleep(0.5) # 给其他线程一点时间退出
        cv2.destroyAllWindows()
        # rclpy.shutdown() # 会在main函数末尾调用
        import sys
        sys.exit(0)

    def send_request(self, client, msg):
        if not client.service_is_ready():
            self.get_logger().error(f"Service client {client.srv_name} is not available.")
            return None
        future = client.call_async(msg)
        # 使用executor来spin_until_future_complete可能更健壮，但在回调中可能复杂
        # 这里简化处理，增加超时
        timeout_sec = 5.0 
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout_sec:
            if future.done():
                if future.result() is not None:
                    return future.result()
                else:
                    self.get_logger().error(f"Service call to {client.srv_name} failed: Future returned None.")
                    return None
            time.sleep(0.01) # 短暂等待
        
        self.get_logger().error(f"Service call to {client.srv_name} timed out after {timeout_sec} seconds.")
        return None


    def set_shape_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % f"Setting target shapes: {request.data}")
        self.colors = None
        self.shapes = request.data
        self.target_shapes = '' # 重置当前目标
        self.start = True
        response.success = True
        response.message = "set_shape"
        return response

    def set_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % f"Setting target colors: {request.data}")
        self.shapes = None
        self.colors = request.data
        self.target_shapes = '' # 重置当前目标
        self.start = True
        response.success = True
        response.message = "set_color"
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "Start command received.")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "Stop command received.")
        self.start = False
        self.colors = None
        self.shapes = None
        self.moving = False
        self.count = 0
        self.target_shapes = ''
        self.last_position = np.array([0.0, 0.0])
        self.last_object_info_list = []
        response.success = True
        response.message = "stop"
        return response

    def goto_default(self):
        self.get_logger().info("Moving arm to default position...")
        msg = set_joint_value_target([500.0, 700.0, 151.0, 70.0, 500.0]) # 使用当前倾斜位置
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        if endpoint:
            pose_t = endpoint.pose.position
            pose_r = endpoint.pose.orientation
            set_servo_position(self.joints_pub, 1.5, ((1, 500), (2, 700), (3, 151), (4, 70), (5, 500), (10, 600))) # 10号是爪子
            time.sleep(1.5)
            self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
            self.get_logger().info("Arm reached default position.")
        else:
            self.get_logger().error("Failed to get endpoint for default position.")
            # 可能需要添加错误处理逻辑

    def move(self, object_info):
        self.moving = True # 确保状态被设置
        self.get_logger().info(f"Starting move sequence for object: {object_info[0]}")
        shape, pose_t = object_info[:2] 
        color, angle = object_info[-2:] 
        
        # 播放提示音
        msg_buzzer = BuzzerState()
        msg_buzzer.freq = 1900
        msg_buzzer.on_time = 0.2
        msg_buzzer.off_time = 0.01
        msg_buzzer.repeat = 1
        self.buzzer_pub.publish(msg_buzzer)
        time.sleep(0.5) # 短暂延迟

        # --- 计算抓取偏移 ---
        if 'sphere' in shape:
            offset_z = -0.015 + self.pick_offset[-1]
        elif 'cylinder' in shape and 'horizontal' not in shape:
            offset_z = 0.02 + self.pick_offset[-1] # 竖直圆柱
        else: # 立方体 或 水平圆柱
            offset_z = 0.01 + self.pick_offset[-1]

        # X/Y 偏移 (基于物体在工作空间的位置 - 这个逻辑可能需要根据实际情况调整)
        offset_x = self.pick_offset[0] if pose_t[0] > 0.21 else self.pick_offset[1]
        offset_y = self.pick_offset[2] if pose_t[1] > 0 else self.pick_offset[3]

        target_pose_approach = pose_t.copy() # 复制一份，避免修改原始传入的位置
        target_pose_approach[0] += offset_x
        target_pose_approach[1] += offset_y
        target_pose_approach[2] += offset_z # Z偏移用于预抓取位置

        self.get_logger().info(f"Calculated approach pose: {target_pose_approach}")

        # --- 移动到预抓取位置 (上方) ---
        msg_approach = kinematics_control.set_pose_target(target_pose_approach, 85)
        res1 = self.send_request(self.kinematics_client, msg_approach)

        if res1 and res1.pulse:
            servo_data1 = res1.pulse
            self.get_logger().info("Moving to approach position...")
            # 分步移动关节，增加稳定性
            set_servo_position(self.joints_pub, 0.8, ((1, servo_data1[0]),))
            time.sleep(0.8)
            set_servo_position(self.joints_pub, 1.5, ((2, servo_data1[1]), (3, servo_data1[2]), (4, servo_data1[3]), (5, 500))) # 第5关节回中
            time.sleep(1.5)
        else:
            self.get_logger().error("Failed to get kinematics solution for approach pose or service call failed.")
            self.moving = False
            return # 无法移动，退出

        # --- 计算并移动到抓取位置 (下降) ---
        target_pose_pick = target_pose_approach.copy()
        target_pose_pick[2] -= 0.05 # 向下移动5cm (这个值也可能需要调整)
        self.get_logger().info(f"Calculated pick pose: {target_pose_pick}")
        msg_pick = kinematics_control.set_pose_target(target_pose_pick, 85)
        res2 = self.send_request(self.kinematics_client, msg_pick)

        # --- 计算爪子旋转角度 ---
        if res2 and res2.rpy: # 确保 res2 和 rpy 有效
            final_gripper_angle_pulse = 500 # 默认爪子角度
            if angle != 0: # 如果物体有角度
                if 'sphere' in shape or ('cylinder' in shape and 'horizontal' not in shape):
                    # 球体或竖直圆柱不需要精确旋转
                    pass # 保持默认角度 500
                else: # 立方体或水平圆柱
                    current_arm_roll = res2.rpy[-1] # 获取机械臂当前的 roll 角度 (可能需要确认这是否是第5关节的角度)
                    # 角度归一化和计算 (保持原有逻辑，但需要验证其正确性)
                    if 'cuboid' in shape:
                        angle = angle % 90
                        if angle > 45: angle = 90 - angle
                        angle = -angle # 反转方向?
                    else: # 水平圆柱
                        angle = angle % 180
                        angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
                    
                    # 转换为舵机脉冲值 (这个转换因子 1000/240 需要标定！)
                    # 假设目标是让爪子相对于物体对齐，需要结合手臂的roll角
                    # target_angle_world = angle # 物体在世界坐标系下的角度 (来自识别)
                    # required_gripper_rotation = target_angle_world - current_arm_roll # 需要爪子相对手臂旋转的角度? (逻辑存疑)
                    # final_gripper_angle_pulse = 500 + int(1000 * required_gripper_rotation / 240)

                    # 沿用原版计算逻辑，但需注意验证 res2.rpy[-1] 的含义和转换系数
                    final_gripper_angle_pulse = 500 + int(1000 * (angle + current_arm_roll) / 240)
                    final_gripper_angle_pulse = np.clip(final_gripper_angle_pulse, 0, 1000) # 限制在0-1000
                    self.get_logger().info(f"Calculated gripper angle pulse: {final_gripper_angle_pulse} (Object angle: {angle:.1f}, Arm roll: {current_arm_roll:.1f})")
            else:
                 self.get_logger().info("Object angle is 0, using default gripper angle (500).")
        else:
             self.get_logger().warning("Failed to get kinematics solution for pick pose or RPY data missing. Using default gripper angle.")
             final_gripper_angle_pulse = 500 # 保底值

        # --- 执行抓取动作 ---
        if res2 and res2.pulse:
            servo_data2 = res2.pulse
            self.get_logger().info("Moving to pick position and adjusting gripper...")
            # 先旋转爪子
            set_servo_position(self.joints_pub, 0.5, ((5, final_gripper_angle_pulse),))
            time.sleep(0.5)
            # 再移动手臂到抓取位置
            set_servo_position(self.joints_pub, 1.0, ((1, servo_data2[0]), (2, servo_data2[1]), (3, servo_data2[2]), (4, servo_data2[3]), (5, final_gripper_angle_pulse)))
            time.sleep(1.0)

            # 关闭爪子
            self.get_logger().info("Closing gripper...")
            gripper_close_pulse = 700 if 'sphere' in shape else 410 # 球体抓紧一点
            set_servo_position(self.joints_pub, 0.6, ((10, gripper_close_pulse),))
            time.sleep(0.6)

            # --- 抬起物体 ---
            self.get_logger().info("Lifting object...")
            # 使用 res1 的关节数据回到预抓取高度
            set_servo_position(self.joints_pub, 1.0, ((1, servo_data1[0]), (2, servo_data1[1]), (3, servo_data1[2]), (4, servo_data1[3]), (5, final_gripper_angle_pulse)))
            time.sleep(1.0)

            # --- 移动到放置区 ---
            self.get_logger().info("Moving to drop-off area...")
            # 定义一个中间过渡位置，避免直接从高处移动到放置区导致碰撞或奇异
            set_servo_position(self.joints_pub, 1.0, ((1, 500), (2, 640), (3, 150), (4, 130), (5, 500))) # 保持爪子闭合 (10号不变)
            time.sleep(1.0)

            # 根据形状或颜色执行放置动作
            if self.colors is None: # 按形状放置
                target_action = None
                if "sphere" in shape: target_action = "target_1"
                elif "cylinder" in shape: target_action = "target_2" # 包含水平和竖直
                elif "cuboid" in shape: target_action = "target_3"
                
                if target_action:
                    self.get_logger().info(f"Running drop-off action: {target_action}")
                    self.controller.run_action(target_action) # run_action 应该包含松爪动作
                else:
                    self.get_logger().warning(f"No drop-off action defined for shape: {shape}. Releasing gripper at current location.")
                    set_servo_position(self.joints_pub, 0.6, ((10, 600),)) # 松开爪子
                    time.sleep(0.6)

            else: # 按颜色放置
                detected_color_name = self.color_comparison(color) # color 是 RGB 值
                target_action = None
                self.get_logger().info(f"Detected color RGB: {color}, classified as: {detected_color_name}")
                if detected_color_name == "red": target_action = "target_1"
                # elif detected_color_name == "green": target_action = "target_2" # 假设绿色放2号位
                elif detected_color_name == "blue": target_action = "target_3"

                if target_action:
                    self.get_logger().info(f"Running drop-off action: {target_action}")
                    self.controller.run_action(target_action) # run_action 应该包含松爪动作
                else:
                    self.get_logger().warning(f"No drop-off action defined for color: {detected_color_name}. Releasing gripper at current location.")
                    set_servo_position(self.joints_pub, 0.6, ((10, 600),)) # 松开爪子
                    time.sleep(0.6)
            
            # 等待放置动作完成
            time.sleep(2.0) # 等待action group执行完毕 (如果action group是阻塞的则不需要)

        else:
            self.get_logger().error("Failed to get kinematics solution for pick pose or service call failed. Aborting move.")
            # 可能需要先抬起来再返回默认位
            if res1 and res1.pulse:
                set_servo_position(self.joints_pub, 1.0, ((1, res1.pulse[0]), (2, res1.pulse[1]), (3, res1.pulse[2]), (4, res1.pulse[3]), (5, 500)))
                time.sleep(1.0)
            # 松开爪子以防万一
            set_servo_position(self.joints_pub, 0.6, ((10, 600),))
            time.sleep(0.6)


        # --- 返回默认位置 ---
        self.get_logger().info("Returning to default position.")
        self.goto_default()
        time.sleep(1.0) # 确保手臂稳定

        self.moving = False
        self.get_logger().info("Move sequence completed.")


    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if not self.running: return # 如果节点已停止，则不处理
        try:
            if self.image_queue.full():
                self.image_queue.get_nowait() # 非阻塞获取并丢弃旧帧
            self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info), block=False) # 非阻塞放入
        except queue.Full:
             self.get_logger().warning("Image queue is full, dropping frame.")
        except Exception as e:
             self.get_logger().error(f"Error in multi_callback: {str(e)}")


    def cal_position(self, x, y, depth, intrinsic_matrix):
        try:
            if depth <= 0:
                self.get_logger().warning(f"Invalid depth value in cal_position: {depth}")
                return None
            
            # 像素坐标转相机坐标
            position_cam = depth_pixel_to_camera([x, y, depth / 1000.0], intrinsic_matrix) # 深度单位 mm 转 m
            # self.get_logger().debug(f"Position in camera frame: {position_cam}")

            if self.endpoint is None:
                self.get_logger().error("Endpoint transform (robot base to end effector) is not initialized.")
                return None
                
            # 应用手眼标定矩阵: 相机坐标 -> 末端坐标
            # 注意：这里假设 position_cam 是 [X, Y, Z, 1] 的齐次坐标形式，需要转换
            pos_cam_h = np.append(position_cam, 1) # 转为齐次坐标 [X, Y, Z, 1]
            # pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position_cam, (0, 0, 0))) # 原有逻辑似乎是转换一个姿态？
            # 修正：直接转换点坐标
            # 目标点在末端坐标系下的齐次坐标 = hand2cam_tf_matrix * 目标点在相机坐标系下的齐次坐标
            pos_end_h = np.dot(self.hand2cam_tf_matrix, pos_cam_h)
            # self.get_logger().debug(f"Position in end-effector frame (homogeneous): {pos_end_h}")

            # 转换到世界坐标系: 世界坐标 = 末端位姿矩阵 * 点在末端坐标系下的坐标
            # 末端位姿矩阵 self.endpoint 是 base -> end_effector 的变换
            # 目标点在世界坐标系下的齐次坐标 = self.endpoint * 目标点在末端坐标系下的齐次坐标
            world_pose_h = np.dot(self.endpoint, pos_end_h)
            # self.get_logger().debug(f"Position in world frame (homogeneous): {world_pose_h}")

            # 提取世界坐标 XYZ
            world_pose_t = world_pose_h[:3] / world_pose_h[3] # 齐次坐标转笛卡尔坐标

            # --- 应用固定的抓取坐标调整 (可能需要重新评估这个调整的必要性和数值) ---
            # 这些调整的坐标系基准是什么？世界坐标系？
            # 假设是世界坐标系调整
            final_pose = world_pose_t.copy()
            # final_pose[0] += 0.09 # 这些值非常依赖标定和设置
            # final_pose[2] += 0.11
            # final_pose[0] -= 0.01 # 原有逻辑中的调整，暂时注释掉
            
            # self.get_logger().info(f"Calculated world position: Raw={world_pose_t}, Final={final_pose}")
            return final_pose

        except Exception as e:
            self.get_logger().error(f"Error calculating position: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def get_min_distance(self, depth_image_raw):
        # 不修改传入的图像，在内部复制
        depth_image = depth_image_raw.copy()
        ih, iw = depth_image.shape[:2]
        
        # --- 获取ROI区域 ---
        # 确保ROI坐标有效
        y_min, y_max = max(0, self.roi[0]), min(ih, self.roi[1])
        x_min, x_max = max(0, self.roi[2]), min(iw, self.roi[3])
        
        if y_min >= y_max or x_min >= x_max:
             self.get_logger().warning("Invalid ROI definition.")
             return self.max_valid_depth # 返回一个大的默认值

        roi_depth = depth_image[y_min:y_max, x_min:x_max]

        # --- 过滤无效深度值 ---
        valid_depths = roi_depth[(roi_depth > self.min_valid_depth) & (roi_depth < self.max_valid_depth)]

        if valid_depths.size == 0:
            # self.get_logger().warning("No valid depth points found in ROI for min_distance calculation.")
            # 保持原有逻辑，查找整个图像的最小值（但在ROI之外可能不可靠）
             min_dist = self.max_valid_depth # 返回一个默认大值
        else:
            min_dist = np.min(valid_depths)

        # self.get_logger().debug(f"Minimum valid distance in ROI: {min_dist} mm")
        return min_dist


    # ===========================================================================
    # === 修改后的 get_contours 函数 ===
    # ===========================================================================
    def get_contours(self, depth_image_raw, min_dist_unused): # min_dist 参数不再直接使用
        try:
            # 复制一份深度图用于处理，避免修改原始数据
            depth_image = depth_image_raw.copy()
            ih, iw = depth_image.shape[:2]

            # --- 1. 获取ROI内的有效深度值 (排除0和配置的无效范围) ---
            roi_y_min, roi_y_max, roi_x_min, roi_x_max = self.roi
            # 再次确保 ROI 边界在图像内
            roi_y_min, roi_y_max = max(0, roi_y_min), min(ih, roi_y_max)
            roi_x_min, roi_x_max = max(0, roi_x_min), min(iw, roi_x_max)

            if roi_y_min >= roi_y_max or roi_x_min >= roi_x_max:
                 self.get_logger().warning("Invalid ROI definition in get_contours.")
                 return []

            roi_depth = depth_image[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
            valid_depths_in_roi = roi_depth[(roi_depth > self.min_valid_depth) & (roi_depth < self.max_valid_depth)]

            if valid_depths_in_roi.size == 0:
                self.get_logger().warning("No valid depth points found in ROI to estimate background.")
                return [] # 无法估计背景，返回空

            # --- 2. 估计背景深度 (使用中位数更鲁棒) ---
            median_background_depth = np.median(valid_depths_in_roi)
            # self.get_logger().info(f"Estimated median background depth in ROI: {median_background_depth:.2f} mm")

            # --- 3. 定义物体最大深度阈值 ---
            # 物体必须比背景中位数"高"(深度值小) object_height_threshold
            object_max_depth = median_background_depth - self.object_height_threshold
            # self.get_logger().info(f"Object max depth threshold: {object_max_depth:.2f} mm (MedianBG - {self.object_height_threshold}mm)")

            # --- 4. 过滤背景和无效点 ---
            # 创建掩码：True代表可能是物体点
            # 条件1: 深度值必须小于计算出的物体最大深度阈值
            mask_object = depth_image < object_max_depth
            # 条件2: 深度值必须在有效的全局范围内
            mask_valid_range = (depth_image > self.min_valid_depth) & (depth_image < self.max_valid_depth)

            # 合并掩码，只有同时满足两个条件的点才保留
            final_mask = mask_object & mask_valid_range

            # 应用掩码，不符合条件的像素深度值置0
            depth_image_filtered = np.where(final_mask, depth_image, 0)
            # self.get_logger().debug("Background and invalid points filtered based on dynamic threshold.")
            
            # --- (可选) 移除 min_dist + 40 过滤 ---
            # 这一步在动态背景估计后可能不再需要，或者其阈值需要重新评估。
            # 如果保留，它可能会错误地移除位于背景深度附近但高于阈值的物体部分。
            # 建议先注释掉观察效果。
            # depth_image_filtered = np.where(depth_image_filtered > min_dist + 40, 0, depth_image_filtered)
            # self.get_logger().info("Applied min_dist + 40 filter (may be removed).")


            # --- 5. 归一化用于二值化 ---
            # 归一化的范围可以用估计的物体深度范围，例如 min_valid_depth 到 object_max_depth
            norm_min = self.min_valid_depth
            norm_max = object_max_depth

            # 防止除以零或负数
            norm_range = norm_max - norm_min
            if norm_range <= 0:
                # self.get_logger().warning(f"Normalization range is invalid ({norm_range}), using default range.")
                norm_range = self.max_valid_depth - self.min_valid_depth # 使用备用范围
                if norm_range <=0 : norm_range = 1 # 最后保险

            # 归一化，注意处理值为0的情况 (背景)
            # 使用 filtered image 进行归一化
            depth_normalized = np.where(depth_image_filtered > 0,
                                        np.clip((depth_image_filtered - norm_min) / norm_range, 0, 1) * 255,
                                        0)
            depth_gray = depth_normalized.astype(np.uint8)
            # self.get_logger().debug("Depth image normalized for thresholding.")

            # --- 6. 二值化 ---
            _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
            # self.get_logger().debug("Depth image thresholded.")

            # --- 7. 形态学操作 (推荐) ---
            # 使用开运算去除小的噪声点
            if self.morph_open_kernel_size[0] > 0 and self.morph_open_kernel_size[1] > 0:
                 kernel_open = np.ones(self.morph_open_kernel_size, np.uint8)
                 depth_bit = cv2.morphologyEx(depth_bit, cv2.MORPH_OPEN, kernel_open)
                 # self.get_logger().debug("Morphological opening applied.")

            # 使用闭运算填充物体内部的小洞或连接断裂部分
            if self.morph_close_kernel_size[0] > 0 and self.morph_close_kernel_size[1] > 0:
                 kernel_close = np.ones(self.morph_close_kernel_size, np.uint8)
                 depth_bit = cv2.morphologyEx(depth_bit, cv2.MORPH_CLOSE, kernel_close)
                 # self.get_logger().debug("Morphological closing applied.")

            # --- (调试) 显示二值化结果 ---
            # cv2.imshow("Binary Segmentation", depth_bit)

            # --- 8. 查找轮廓 ---
            contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # self.get_logger().info(f"Found {len(contours)} contours after filtering and morphological operations.")

            return contours
        except Exception as e:
            self.get_logger().error(f"Error in get_contours: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return []


    # ===========================================================================
    # === 修改后的 shape_recognition 函数 ===
    # ===========================================================================
    def shape_recognition(self, rgb_image, depth_image_raw, depth_color_map, intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image_raw.shape[:2]
        
        # min_dist 现在只是一个参考值，主要依赖 get_contours 中的动态分割
        # if min_dist <= 300: # 这个检查可能仍然有用，防止在没有物体时进行处理
        
        sphere_index = 0
        cuboid_index = 0
        cylinder_index = 0
        cylinder_horizontal_index = 0
        
        # --- 调用修改后的 get_contours ---
        # 传入原始深度图，min_dist不再直接用于过滤，但可以传给函数（虽然函数内部未使用）
        contours = self.get_contours(depth_image_raw, min_dist)
        
        if not contours:
             # self.get_logger().info("No contours found by get_contours.")
             return [] # 没有找到轮廓，直接返回

        for i, obj_contour in enumerate(contours):
            try:
                area = cv2.contourArea(obj_contour)
                if area < self.min_contour_area:
                    # self.get_logger().debug(f"Contour {i+1} area ({area:.1f}) too small, skipping.")
                    continue
                    
                perimeter = cv2.arcLength(obj_contour, True)
                if perimeter <= 0: continue # 避免除零错误

                # 获取轮廓角点坐标
                approx = cv2.approxPolyDP(obj_contour, 0.035 * perimeter, True)
                CornerNum = len(approx)
                
                # 获取最小包围矩形信息
                center, (width, height), angle = cv2.minAreaRect(obj_contour)
                center_x, center_y = center
                
                # --- 改进深度获取：使用轮廓内深度中位数 ---
                mask = np.zeros(depth_image_raw.shape[:2], dtype=np.uint8)
                cv2.drawContours(mask, [obj_contour], -1, 255, -1) # 填充轮廓

                # 获取轮廓内的所有有效深度值 (使用原始深度图!)
                contour_depths = depth_image_raw[mask == 255]
                # 再次过滤无效深度
                valid_contour_depths = contour_depths[(contour_depths > self.min_valid_depth) & (contour_depths < self.max_valid_depth)]

                if valid_contour_depths.size < self.min_valid_contour_points:
                    # self.get_logger().warning(f"Contour {i+1} has only {valid_contour_depths.size} valid depth points (min {self.min_valid_contour_points}), skipping.")
                    continue

                # 使用中位数作为代表深度
                representative_depth = np.median(valid_contour_depths)
                # self.get_logger().info(f"Contour {i+1}: Area={area:.1f}, Center=({center_x:.1f}, {center_y:.1f}), RepDepth={representative_depth:.2f}mm")

                # --- 使用代表深度和轮廓中心计算3D位置 ---
                if not (0 <= int(center_y) < image_height and 0 <= int(center_x) < image_width):
                     # self.get_logger().warning(f"Contour {i+1} center ({center_x:.1f}, {center_y:.1f}) is outside image bounds, skipping.")
                     continue

                position = self.cal_position(center_x, center_y, representative_depth, intrinsic_matrix)
                
                if position is None:
                    # self.get_logger().warning(f"Position calculation failed for contour {i+1} (RepDepth={representative_depth:.2f}), skipping.")
                    continue
                
                # --- 计算轮廓内深度标准差 (使用有效深度点) ---
                depth_std = np.std(valid_contour_depths)
                # self.get_logger().info(f"Contour {i+1}: Depth StdDev={depth_std:.2f}")

                # --- 调整角度 (保持原有逻辑) ---
                # 这个角度是 minAreaRect 返回的，可能需要根据抓取策略调整
                adjusted_angle = angle
                if width > 0 and height > 0: # 避免除零
                    if angle < -45:
                        adjusted_angle += 90 # OpenCV的minAreaRect角度范围问题调整
                        # 交换宽高以匹配新的角度定义
                        width, height = height, width
                    # 原有逻辑: if width > height and width / height > 1.5: adjusted_angle = angle + 90
                    # 这个逻辑似乎是为了让角度表示长轴方向？如果上面的调整已处理，可能不再需要。
                    # 保留 minAreaRect 调整后的角度 adjusted_angle (-90 到 0 之间)
                else:
                    adjusted_angle = 0 # 宽高无效则角度无意义
                
                # --- 改进的形状分类逻辑 ---
                objType = None
                shape_info = f"std={depth_std:.1f}, cn={CornerNum}" # 基础信息

                # 1. 球体: 通常深度标准差大，角点多（近似圆），圆形度高
                circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                shape_info += f", circ={circularity:.2f}"
                if depth_std > self.sphere_std_threshold and CornerNum > 6 and circularity > 0.8: # 条件更严格
                    sphere_index += 1
                    final_angle = 0 # 球体无角度
                    objType = 'sphere_' + str(sphere_index)
                    # self.get_logger().info(f"Classified as Sphere ({shape_info})")
                else:
                    # 2. 立方体或圆柱体: 区分依据主要是角点数、长宽比、圆形度
                    aspect_ratio = max(width, height) / min(width, height) if min(width, height) > 0 else 0
                    shape_info += f", asp={aspect_ratio:.2f}"
                    
                    if CornerNum >= 3 and CornerNum <= 6: # 可能是立方体或水平圆柱
                        if aspect_ratio < self.cuboid_aspect_ratio_threshold: # 长宽比接近1，认为是立方体
                            cuboid_index += 1
                            final_angle = adjusted_angle # 使用调整后的角度
                            objType = "cuboid_" + str(cuboid_index)
                            # self.get_logger().info(f"Classified as Cuboid ({shape_info})")
                        elif aspect_ratio > self.horizontal_cylinder_aspect_ratio_threshold: # 细长，认为是水平圆柱
                            cylinder_horizontal_index += 1
                            final_angle = adjusted_angle # 使用调整后的角度
                            objType = "cylinder_horizontal_" + str(cylinder_horizontal_index)
                            # self.get_logger().info(f"Classified as Horizontal Cylinder ({shape_info})")
                        else: # 中间情况，可能需要更复杂的判断，暂时归为立方体
                            cuboid_index += 1
                            final_angle = adjusted_angle
                            objType = "cuboid_" + str(cuboid_index)
                            # self.get_logger().info(f"Classified as Cuboid (intermediate aspect ratio) ({shape_info})")
                    elif circularity > self.cylinder_circularity_threshold: # 比较圆，可能是竖直圆柱
                        # 竖直圆柱的标准差通常比立方体小，但比桌面大
                         cylinder_index += 1
                         final_angle = 0 # 竖直圆柱无角度
                         objType = "cylinder_" + str(cylinder_index)
                         # self.get_logger().info(f"Classified as Cylinder ({shape_info})")
                    # 保底/其他情况：如果标准差略高于桌面但形状不明确，也可能尝试归类为圆柱
                    elif depth_std > self.cylinder_default_std_threshold:
                         cylinder_index += 1
                         final_angle = 0
                         objType = "cylinder_" + str(cylinder_index)
                         # self.get_logger().info(f"Classified as Cylinder (default/low std) ({shape_info})")
                    else:
                        # self.get_logger().warning(f"Could not classify shape for contour {i+1} ({shape_info})")
                        pass # 未能分类


                # --- 获取颜色和保存信息 ---
                if objType is not None:
                    # 获取物体中心点的颜色
                    rgb_value = (0, 0, 0) # 默认值
                    if 0 <= int(center_y) < rgb_image.shape[0] and 0 <= int(center_x) < rgb_image.shape[1]:
                         # 注意：OpenCV图像是BGR顺序，但ROS消息可能是RGB
                         # 假设传入的 rgb_image 是 BGR (来自cv_bridge?)
                         # bgr_val = rgb_image[int(center_y), int(center_x)]
                         # rgb_value = (bgr_val[2], bgr_val[1], bgr_val[0]) # 转为RGB元组

                         # 如果确认 rgb_image 是 RGB 格式
                         rgb_value = tuple(rgb_image[int(center_y), int(center_x)])
                         # self.get_logger().debug(f"Object {i+1} RGB: {rgb_value}")
                    else:
                         self.get_logger().warning(f"Center point ({center_x}, {center_y}) out of RGB image bounds.")

                    color_name = self.color_comparison(rgb_value) # 使用RGB值进行比较
                    # self.get_logger().info(f"Object {i+1} color classified as: {color_name}")
                    
                    # bounding box 信息 (使用 approx 计算更贴合物体)
                    x_br, y_br, w_br, h_br = cv2.boundingRect(approx)
                    
                    # 添加到结果列表
                    object_info_list.append([
                        objType,                    # 物体类型 (str)
                        position,                   # 3D 位置 (np.array)
                        representative_depth,       # 代表性深度 (float)
                        [x_br, y_br, w_br, h_br,    # 边界矩形 (int)
                         center,                   # 最小外接矩形中心 (tuple float)
                         width, height],           # 最小外接矩形宽高 (float)
                        rgb_value,                  # 颜色 (tuple int RGB)
                        final_angle                 # 角度 (float)
                    ])

                    # --- 在图像上绘制信息 (可选) ---
                    # 绘制轮廓
                    cv2.drawContours(depth_color_map, [obj_contour], -1, (0, 255, 0), 1)
                    # 绘制最小外接矩形
                    box_points = cv2.boxPoints(((center_x, center_y), (width, height), angle))
                    box_points = np.int0(box_points)
                    cv2.drawContours(depth_color_map, [box_points], 0, (0, 0, 255), 1)
                    # 绘制中心点
                    cv2.circle(depth_color_map, (int(center_x), int(center_y)), 3, (255, 255, 255), -1)
                    # 显示类型和ID
                    cv2.putText(depth_color_map, f"{i+1}:{objType.split('_')[0]}",
                               (x_br, y_br - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            except Exception as e:
                self.get_logger().error(f"Error processing contour {i+1}: {str(e)}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                continue
                
        # self.get_logger().info(f"Shape recognition finished. Found {len(object_info_list)} objects.")
        return object_info_list


    def color_comparison(self, rgb_tuple):
        # 确保输入是元组或列表，并且有三个元素
        if not isinstance(rgb_tuple, (tuple, list)) or len(rgb_tuple) != 3:
             # self.get_logger().warning(f"Invalid input for color comparison: {rgb_tuple}")
             return None # 或者返回 'unknown'

        r, g, b = rgb_tuple
        # 简单的颜色比较逻辑，可能需要根据实际光照和颜色调整阈值
        # 例如，增加判断条件：红色分量不仅最大，而且要显著大于其他分量
        red_threshold = 100
        blue_threshold = 100
        diff_threshold = 30 # 要求最大分量比其他分量至少大 diff_threshold

        is_red = r > g + diff_threshold and r > b + diff_threshold and r > red_threshold
        is_blue = b > r + diff_threshold and b > g + diff_threshold and b > blue_threshold
        # is_green = g > r + diff_threshold and g > b + diff_threshold # 你的原始代码没有绿色

        if is_red:
            return 'red'
        elif is_blue:
            return 'blue'
        # elif is_green:
        #     return 'green'
        else:
            return None # 或 'unknown'

    def main(self):
        loop_count = 0
        while self.running:
            try:
                # 从队列获取图像数据
                try:
                    # 使用阻塞获取，等待新图像，超时时间设为1秒
                    ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1.0)
                    # 清空队列中可能积压的旧帧，只处理最新一帧
                    while not self.image_queue.empty():
                         try:
                              self.image_queue.get_nowait()
                         except queue.Empty:
                              break # 队列已空
                except queue.Empty:
                    # 如果超时，继续下一次循环等待
                    # self.get_logger().debug("Image queue empty, waiting...")
                    continue

                loop_count += 1
                
                # --- 图像转换 ---
                try:
                    # 假设ROS图像是BGR8格式
                    bgr_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                    # 创建一个RGB副本用于颜色识别，保持BGR用于显示
                    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

                    # 假设深度图像是16UC1格式 (mm)
                    depth_image_raw = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                except Exception as e:
                    self.get_logger().error(f"Error converting ROS images to OpenCV format: {str(e)}")
                    continue # 跳过这一帧

                # 备份原始深度图，处理用副本
                depth_image_processed = depth_image_raw.copy()

                # --- 获取相机内参 ---
                intrinsic_matrix = depth_camera_info.k

                # --- 计算最小距离 (主要用于调试或参考) ---
                min_dist = self.get_min_distance(depth_image_raw) # 使用原始深度图获取

                # --- 打印ROI深度样本 (降低频率) ---
                if loop_count % 100 == 0: # 每100帧打印一次
                    roi_y_min, roi_y_max, roi_x_min, roi_x_max = self.roi
                    if roi_y_min < roi_y_max and roi_x_min < roi_x_max:
                         roi_depth_sample = depth_image_raw[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
                         sample_interval = 20
                         roi_sample_info = "ROI Depth Samples:\n"
                         for i in range(0, roi_depth_sample.shape[0], sample_interval):
                             row_vals = [str(roi_depth_sample[i, j]) for j in range(0, roi_depth_sample.shape[1], sample_interval)]
                             roi_sample_info += f"Row {roi_y_min + i}: {' '.join(row_vals)}\n"
                         self.get_logger().info(roi_sample_info.strip()) # 移除末尾换行
                    loop_count = 0 # 重置计数器

                # --- 可视化深度图 (jet colormap) ---
                # 归一化以获得更好的可视化效果，使用一个固定的或动态的最大值
                vis_max_depth = 500 # 可视化最大深度 (mm)
                depth_vis = np.clip(depth_image_raw, 0, vis_max_depth)
                depth_vis = (depth_vis / vis_max_depth * 255).astype(np.uint8)
                depth_color_map = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

                # --- 主逻辑：识别与抓取 ---
                if not self.moving: # 仅在手臂不在移动时进行识别
                    # 调用修改后的形状识别函数
                    # 传入 RGB 图像用于颜色，原始深度图用于计算，可视化深度图用于绘制
                    object_info_list = self.shape_recognition(rgb_image, depth_image_raw, depth_color_map, intrinsic_matrix, min_dist)

                    if self.start: # 只有在启动状态下才进行后续处理
                        reorder_object_info_list = object_info_list
                        if object_info_list:
                            if self.last_object_info_list:
                                # 对比上一次的物体的位置来重新排序
                                reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, 0.02) # 距离阈值设为2cm
                            
                            # --- 选择目标物体 ---
                            if not self.target_shapes: # 如果当前没有追踪目标
                                self.get_logger().info("Selecting a new target object...")
                                valid_indices = []
                                if self.shapes: # 按形状过滤
                                    valid_indices = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] in self.shapes]
                                    # self.get_logger().info(f"Shape filter ({self.shapes}) yields indices: {valid_indices}")
                                elif self.colors: # 按颜色过滤
                                    valid_indices = [i for i, info in enumerate(reorder_object_info_list) if self.color_comparison(info[-2]) in self.colors]
                                    # self.get_logger().info(f"Color filter ({self.colors}) yields indices: {valid_indices}")
                                else: # 如果形状和颜色都未指定，则考虑所有识别到的物体
                                     valid_indices = list(range(len(reorder_object_info_list)))
                                     # self.get_logger().info(f"No specific shape/color, considering all {len(valid_indices)} objects.")

                                if valid_indices:
                                    # 选择距离最近（深度最小）的有效物体
                                    # info[2] 是 representative_depth
                                    min_depth_index = min(valid_indices, key=lambda i: reorder_object_info_list[i][2])
                                    self.target_shapes = reorder_object_info_list[min_depth_index][0] # 使用完整类型名作为追踪目标，例如 'cuboid_1'
                                    self.get_logger().info(f"New target selected: {self.target_shapes} at index {min_depth_index} (Depth: {reorder_object_info_list[min_depth_index][2]:.1f}mm)")
                                else:
                                     self.get_logger().info("No suitable target object found matching criteria.")
                                     self.target_shapes = None # 明确设为 None
                            
                            # --- 追踪并检查稳定性 ---
                            if self.target_shapes: # 如果有追踪目标
                                target_index = -1
                                for i, info in enumerate(reorder_object_info_list):
                                     if info[0] == self.target_shapes: # 完全匹配类型名
                                          target_index = i
                                          break
                                
                                if target_index != -1: # 找到了当前帧的目标物体
                                    current_object_info = reorder_object_info_list[target_index]
                                    current_position = current_object_info[1][:2] # 只比较 X, Y 坐标

                                    # 计算与上一帧位置的距离
                                    e_distance = np.linalg.norm(current_position - self.last_position)
                                    # self.get_logger().info(f"Tracking {self.target_shapes}. Current Pos: ({current_position[0]:.3f}, {current_position[1]:.3f}), Last Pos: ({self.last_position[0]:.3f}, {self.last_position[1]:.3f}), Dist: {e_distance:.4f} m")

                                    stability_threshold = 0.005 # 5mm
                                    if e_distance <= stability_threshold:
                                        self.count += 1
                                        # self.get_logger().info(f"Object stable. Count: {self.count}")
                                    else:
                                        # self.get_logger().info(f"Object moved. Resetting stability count.")
                                        self.count = 0
                                    
                                    # --- 触发抓取 ---
                                    stability_count_threshold = 5 
                                    if self.count > stability_count_threshold:
                                        self.get_logger().info(f"Object '{self.target_shapes}' stable for {self.count} frames. Initiating grab.")
                                        self.moving = True # 设置移动标志
                                        self.count = 0      # 重置计数器
                                        target_info_to_move = current_object_info # 传递当前帧的目标信息
                                        self.last_object_info_list = [] # 清空上一帧列表，避免移动过程中误判
                                        self.last_position = np.array([0.0, 0.0]) # 重置上次位置
                                        self.target_shapes = None # 清除追踪目标
                                        
                                        # 启动移动线程
                                        threading.Thread(target=self.move, args=(target_info_to_move,), daemon=True).start()
                                        self.get_logger().info("Move thread started.")
                                    
                                    # 更新上一帧位置 (只在找到目标时更新)
                                    self.last_position = current_position

                                    # --- 在图像上高亮目标物体 ---
                                    x_br, y_br, w_br, h_br, center, _, _ = current_object_info[3]
                                    cv2.rectangle(depth_color_map, (x_br, y_br), (x_br + w_br, y_br + h_br), (0, 255, 255), 2) # 黄色框
                                    cv2.putText(depth_color_map, f"Tracking: {self.target_shapes}", (10, 30),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                                    
                                else: # 当前帧没找到追踪的目标
                                     self.get_logger().warning(f"Target '{self.target_shapes}' lost in current frame. Resetting target.")
                                     self.target_shapes = None
                                     self.count = 0
                                     self.last_position = np.array([0.0, 0.0]) # 也重置上次位置

                        # 更新上一帧物体列表 (仅在未开始移动时更新)
                        if not self.moving:
                            self.last_object_info_list = reorder_object_info_list

                    else: # self.start == False
                        # 如果不在启动状态，清除目标和计数
                        self.target_shapes = None
                        self.count = 0
                        self.last_object_info_list = []
                        self.last_position = np.array([0.0, 0.0])
                        # 可以在图像上显示 "Stopped"
                        cv2.putText(depth_color_map, "Stopped", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)


                # --- 显示结果图像 ---
                # 在 BGR 图像上绘制 ROI
                cv2.rectangle(bgr_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (255, 255, 0), 2) # ROI 框用蓝色

                # FPS 计算和显示
                fps_value = self.fps.update()
                fps_text = f"FPS: {fps_value:.1f}"
                cv2.putText(depth_color_map, fps_text, (depth_color_map.shape[1] - 100, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

                # 拼接图像 (确保尺寸匹配，如果需要可以调整大小)
                # 假设 depth_color_map 和 bgr_image 尺寸相同
                result_image = np.concatenate([depth_color_map, bgr_image], axis=1)
                cv2.imshow("Object Classification (Depth | RGB)", result_image)

                # --- 按键处理 ---
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # ESC
                    self.running = False
                    self.get_logger().info("'q' or ESC pressed, shutting down.")
                    break # 退出主循环
                elif key == ord('s'): # 手动切换 Start/Stop 状态
                     self.start = not self.start
                     self.get_logger().info(f"Toggled start state to: {self.start}")
                     # 如果停止，则重置状态
                     if not self.start:
                          self.stop_srv_callback(None, Trigger.Response()) # 调用stop逻辑


            except Exception as e:
                self.get_logger().error(f"Error in main loop: {str(e)}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                time.sleep(0.1) # 发生错误时稍作等待

        # --- 循环结束后清理 ---
        self.get_logger().info("Main loop exited.")
        cv2.destroyAllWindows()
        # rclpy.shutdown() 在外部 main 函数中调用


def main(args=None):
    print("Initializing Object Classification Node...")
    # rclpy.init(args=args) # 在Node的__init__中已经调用
    node = None
    executor = None
    try:
        node = ObjectClassificationNode('object_classification')
        print("Node created. Creating executor...")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        print("Executor created and node added. Spinning...")
        executor.spin()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down.")
    except Exception as e:
        print(f"An error occurred during node execution: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Node and RCLPY shut down.")

if __name__ == "__main__":
    main()