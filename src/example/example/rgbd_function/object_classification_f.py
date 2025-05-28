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
    # pick_offset = [0.01, 0.01, 0.0, -0.01, 0.0]  # x1, x2, y1, y2, z
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
            
            # 初始化FPS计时器
            self.fps = fps.FPS()
            
            try:
                self.language = os.environ['ASR_LANGUAGE']
            except KeyError:
                self.language = "zh"  # 默认语言
            
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
            self.last_position = (0, 0)
            self.last_object_info_list = []
            signal.signal(signal.SIGINT, self.shutdown)
            
            self.image_queue = queue.Queue(maxsize=2)
            
            self.debug = self.get_parameter('debug').value
            self.plane_distance = self.get_parameter('plane_distance').value
            
            print(f"参数获取到: debug={self.debug}, plane_distance={self.plane_distance}")
            
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
            
            ##【新增加】设置相机倾斜补偿参数（单位：度）  
            self.camera_roll_offset = -20.0    # 绕X轴
            self.camera_pitch_offset = 0.0     # 绕Y轴
            self.camera_yaw_offset = 0.0       # 绕Z轴

        except Exception as e:
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
        cv2.destroyAllWindows()
        rclpy.shutdown()
        import sys
        sys.exit(0)

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
        self.last_position = (0, 0)
        self.last_object_info_list = []
        response.success = True
        response.message = "stop"
        return response

    def goto_default(self):
        msg = set_joint_value_target([500.0, 470.0, 220.0, 90.0, 500.0])
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        # set_servo_position(self.joints_pub, 1, ((1, 500), (2, 522), (3, 218), (4, 51), (5, 500), (10, 550)))
        set_servo_position(self.joints_pub, 1, ((1, 500), (2, 700), (3, 151), (4, 70), (5, 500), (10, 600)))  #相机倾斜
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])

    # 修改后的 cal_position 函数：加入了相机倾斜补偿处理
    def cal_position(self, x, y, depth, intrinsic_matrix):
        try:
            if depth <= 0:
                return None
            
            # 1. 使用depth_pixel_to_camera将像素坐标转换到相机坐标系（单位：米）
            position = depth_pixel_to_camera([x, y, depth / 1000.0], intrinsic_matrix)
            
            # 2. 构造齐次坐标
            camera_position = np.array([position[0], position[1], position[2], 1])
            
            # 3. 构造用于消除相机倾斜的旋转矩阵
            roll_offset_rad = math.radians(self.camera_roll_offset)
            pitch_offset_rad = math.radians(self.camera_pitch_offset)
            yaw_offset_rad = math.radians(self.camera_yaw_offset)

            rotation_x = np.array([
                [1, 0, 0],
                [0, math.cos(roll_offset_rad), -math.sin(roll_offset_rad)],
                [0, math.sin(roll_offset_rad), math.cos(roll_offset_rad)]
            ])
            rotation_y = np.array([
                [math.cos(pitch_offset_rad), 0, math.sin(pitch_offset_rad)],
                [0, 1, 0],
                [-math.sin(pitch_offset_rad), 0, math.cos(pitch_offset_rad)]
            ])
            rotation_z = np.array([
                [math.cos(yaw_offset_rad), -math.sin(yaw_offset_rad), 0],
                [math.sin(yaw_offset_rad), math.cos(yaw_offset_rad), 0],
                [0, 0, 1]
            ])

            correction_matrix = np.eye(4)
            correction_matrix[:3, :3] = np.dot(rotation_z, np.dot(rotation_y, rotation_x))
            
            # 4. 应用校正矩阵消除倾斜
            corrected_camera_position = np.dot(correction_matrix, camera_position)
            corrected_position = corrected_camera_position[:3]
            
            # 若需要可在此处调整位置偏移（原代码中有 position[0] -= 0.01），这里沿用原来的偏移
            corrected_position[0] -= 0.01
            
            # 5. 将校正后的相机坐标转换到机器人坐标系
            pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(corrected_position, (0, 0, 0)))
            world_pose = np.matmul(self.endpoint, pose_end)
            pose_t, pose_r = common.mat_to_xyz_euler(world_pose)
            
            return pose_t
        except Exception as e:
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

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
            set_servo_position(self.joints_pub, 0.8, ((1, servo_data[0]),))
            time.sleep(0.8)           
            set_servo_position(self.joints_pub, 1.5, ((2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, 500)))
            time.sleep(1.5)
        pose_t[2] -= 0.05
        msg = kinematics_control.set_pose_target(pose_t, 85)
        res2 = self.send_request(self.kinematics_client, msg)
        
        if angle != 0:
            if 'sphere' in shape or ('cylinder' in shape and 'cylinder_horizontal_' not in shape):
                angle = 500
            else:
                if 'cuboid' in shape:
                    angle = angle % 90
                    if angle > 45:
                        angle = 90 - angle
                    angle = -angle
                else:
                    angle = angle % 180
                    angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
                angle = 500 + int(1000 * (angle + res2.rpy[-1]) / 240)
        else:
            angle = 500
        
        if res2.pulse:
            servo_data = res2.pulse
            set_servo_position(self.joints_pub, 0.5, ((5, angle),))
            time.sleep(0.5)
            set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, angle)))
            time.sleep(1)
        if res1.pulse:
            servo_data = res1.pulse
            set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, angle)))
            time.sleep(1)
        
        set_servo_position(self.joints_pub, 1, ((1, 500), (2, 640), (3, 150), (4, 130), (5, 500), (10, 410)))
        time.sleep(1)
        if self.colors is None:
            if "sphere" in shape:
                self.controller.run_action("target_1")
            if "cylinder" in shape:
                self.controller.run_action("target_2")
            if "cuboid" in shape:
                self.controller.run_action("target_3")
        else:
            color = self.color_comparison(color)
            if "red" == color:
                self.controller.run_action("target_1")
            if "blue" == color:
                self.controller.run_action("target_3")
        self.goto_default()
        time.sleep(2)
        self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def cal_position_without_tilt(self, x, y, depth, intrinsic_matrix):
        # 原来的 cal_position 逻辑（不做倾斜校正），仅供参考
        try:
            if depth <= 0:
                return None
            position = depth_pixel_to_camera([x, y, depth / 1000], intrinsic_matrix)
            position[0] -= 0.01
            pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
            world_pose = np.matmul(self.endpoint, pose_end)
            pose_t, pose_r = common.mat_to_xyz_euler(world_pose)
            return pose_t
        except Exception as e:
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def get_min_distance(self, depth_image):
        ih, iw = depth_image.shape[:2]
        depth_image[:, :self.roi[2]] = np.array([[1000, ] * self.roi[2]] * ih)
        depth_image[:, self.roi[3]:] = np.array([[1000, ] * (iw - self.roi[3])] * ih)
        depth_image[self.roi[1]:, :] = np.array([[1000, ] * iw] * (ih - self.roi[1]))
        depth_image[:self.roi[0], :] = np.array([[1000, ] * iw] * self.roi[0])
        depth = np.copy(depth_image).reshape((-1,))
        depth[depth <= 0] = 55555
        min_index = np.argmin(depth)
        min_y = min_index // iw
        min_x = min_index - min_y * iw
        min_dist = depth_image[min_y, min_x]
        return min_dist

    def get_contours(self, depth_image, min_dist):
        try:
            if self.plane_distance is None:
                self.plane_distance = 1000
            depth_image = np.where(depth_image > self.plane_distance - 10, 0, depth_image)
            depth_image = np.where(depth_image > min_dist + 40, 0, depth_image)
            sim_depth_image_sort = np.clip(depth_image, 0, self.plane_distance - 10).astype(np.float64) / (self.plane_distance - 10) * 255
            depth_gray = sim_depth_image_sort.astype(np.uint8)
            _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            return contours
        except Exception as e:
            import traceback
            self.get_logger().error(traceback.format_exc())
            return []

    def shape_recognition(self, rgb_image, depth_image, depth_color_map, intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        if min_dist <= 300:
            sphere_index = 0
            cuboid_index = 0
            cylinder_index = 0
            cylinder_horizontal_index = 0
            contours = self.get_contours(depth_image, min_dist)
            for i, obj in enumerate(contours):
                area = cv2.contourArea(obj)
                if area < 300:
                    continue
                perimeter = cv2.arcLength(obj, True)
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)
                CornerNum = len(approx)
                (cx, cy), r = cv2.minEnclosingCircle(obj)
                center, (width, height), angle = cv2.minAreaRect(obj)
                if angle < -45:
                    angle += 89
                if width > height and width / height > 1.5:
                    angle = angle + 90
                try:
                    if 0 <= int(cy) < depth_image.shape[0] and 0 <= int(cx) < depth_image.shape[1]:
                        depth = depth_image[int(cy), int(cx)]
                    else:
                        self.get_logger().warning(f"坐标超出范围: ({int(cy)}, {int(cx)}), 图像尺寸: {depth_image.shape}")
                        continue
                    
                    # 使用修改后的 cal_position 计算3D位置（带有相机倾斜补偿）
                    position = self.cal_position(cx, cy, depth, intrinsic_matrix)
                    
                    if position is None:
                        self.get_logger().warning("位置计算返回None，跳过此物体")
                        continue
                    
                    position[0] += 0.13
                    position[2] += 0.07
                    
                    self.get_logger().info(f"3D位置: {position}")
                    
                    x, y, w, h = cv2.boundingRect(approx)
                    self.get_logger().info(f"边界矩形: x={x}, y={y}, w={w}, h={h}")
                    
                    mask = np.full((image_height, image_width), 0, dtype=np.uint8)
                    cv2.drawContours(mask, [obj], -1, (255), cv2.FILLED)
                    
                    depth_image_mask = np.where(depth_image == 0, np.nan, depth_image)
                    depth_std = np.nanstd(mask)
                    self.get_logger().info(f"深度标准差: {depth_std}")
                    
                    objType = None
                    if depth_std > 40.0 and CornerNum > 4:
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
                    
                    rgb_value = rgb_image[int(center[1]), int(center[0])]
                    color_name = self.color_comparison(rgb_value)
                    
                    h_img, w_img = depth_color_map.shape[:2]
                    info_text = f"Object {i+1}: Type={objType.split('_')[0]}, std={depth_std:.1f}, corners={CornerNum}"
                    text_size = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                    text_x = w_img - text_size[0] - 10
                    text_y = h_img - 10 - (len(contours) - i - 1) * 30
                    cv2.putText(depth_color_map, info_text, 
                               (text_x, text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(depth_color_map, info_text, 
                               (text_x, text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(depth_color_map, f"{i+1}", 
                               (int(center[0]), int(center[1])), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(depth_color_map, f"{i+1}", 
                               (int(center[0]), int(center[1])), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, cv2.LINE_AA)
                    
                    object_info_list.append([objType, position, depth, [x, y, w, h, center, width, height], rgb_value, angle])
                    
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
                self.start = True
                
                if self.shapes is None and self.colors is None:
                    self.shapes = ["sphere", "cuboid", "cylinder"]
                
                try:
                    ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    if not self.running:
                        break
                    else:
                        continue
                try:
                    bgr_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                    rgb_image = bgr_image.copy()
                    depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                    
                    depth_image = depth_image.copy()
                    min_dist = self.get_min_distance(depth_image)
                    
                    if count % 50 == 0:
                        roi_y_min, roi_y_max, roi_x_min, roi_x_max = self.roi
                        roi_depth = depth_image[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
                        sample_interval = 20
                        roi_sample_info = "ROI Depth Samples:\n"
                        for i in range(0, roi_depth.shape[0], sample_interval):
                            row_vals = []
                            for j in range(0, roi_depth.shape[1], sample_interval):
                                row_vals.append(str(roi_depth[i, j]))
                            roi_sample_info += f"Row {roi_y_min + i}: " + " ".join(row_vals) + "\n"
                        self.get_logger().info(roi_sample_info)
                    
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
                        sim_depth_image = np.clip(depth_image, 0, 350).astype(np.float64) / 350 * 255
                        depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                        
                        if not self.moving:
                            object_info_list = self.shape_recognition(rgb_image, depth_image, depth_color_map, depth_camera_info.k, min_dist)
                            
                            if self.start:
                                reorder_object_info_list = object_info_list
                                if object_info_list:
                                    if self.last_object_info_list:
                                        reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, 20)
                                
                                if reorder_object_info_list:
                                    if not self.target_shapes:
                                        self.get_logger().info("没有目标形状，准备选择目标")
                                        if self.shapes is not None:
                                            indices = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] in self.shapes]
                                        else:
                                            indices = [i for i, info in enumerate(reorder_object_info_list) if self.color_comparison(info[-2]) in self.colors]
                                        
                                        if indices:
                                            min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][2])
                                            self.target_shapes = reorder_object_info_list[min_depth_index][0].split('_')[0]
                                        else:
                                            self.get_logger().info("没有符合条件的物体")
                                    else:
                                        self.get_logger().info(f"已有目标形状: {self.target_shapes}，寻找匹配物体")
                                        target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] == self.target_shapes]
                                        if target_index:
                                            target_index = target_index[0]
                                            obejct_info = reorder_object_info_list[target_index]
                                            x, y, w, h, center, width, height = obejct_info[3]
                                            angle = obejct_info[-1]
                                            
                                            center_x, center_y = int(center[0]), int(center[1])
                                            cv2.circle(depth_color_map, (center_x, center_y), 5, (255, 255, 255), -1)
                                            cv2.circle(bgr_image, (center_x, center_y), 5, (255, 255, 255), -1)
                                            
                                            coord_text = f"({obejct_info[1][0]:.3f}, {obejct_info[1][1]:.3f}, {obejct_info[1][2]:.3f})"
                                            cv2.putText(depth_color_map, coord_text, (center_x + 10, center_y), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                                            
                                            position_text = f"Position: X={obejct_info[1][0]:.3f}, Y={obejct_info[1][1]:.3f}, Z={obejct_info[1][2]:.3f}"
                                            angle_text = f"Angle: {angle:.2f} degrees"
                                            
                                            cv2.putText(depth_color_map, position_text, (10, 30), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
                                            cv2.putText(depth_color_map, position_text, (10, 30), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
                                            cv2.putText(depth_color_map, angle_text, (10, 60), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
                                            cv2.putText(depth_color_map, angle_text, (10, 60), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
                                            
                                            cv2.putText(bgr_image, position_text, (10, 30), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
                                            cv2.putText(bgr_image, position_text, (10, 30), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
                                            cv2.putText(bgr_image, angle_text, (10, 60), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
                                            cv2.putText(bgr_image, angle_text, (10, 60), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
                                            
                                            cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                        (0, 0, 0), 2, cv2.LINE_AA)
                                            cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1,
                                                             (0, 0, 255), 2, cv2.LINE_AA)
                                            
                                            self.get_logger().info(f"当前位置: {obejct_info[1]}, 上次位置: {self.last_position}")
                                            
                                            e_distance = round(math.sqrt(pow(self.last_position[0] - obejct_info[1][0], 2) + 
                                                                        pow(self.last_position[1] - obejct_info[1][1], 2)), 5)
                                            
                                            self.get_logger().info(f"欧氏距离: {e_distance}, 阈值: 0.005, 当前计数: {self.count}")
                                            
                                            if e_distance <= 0.005:
                                                self.count += 1
                                                self.get_logger().info(f"物体稳定，计数增加到: {self.count}")
                                            else:
                                                self.count = 0
                                                self.get_logger().info(f"物体不稳定，计数重置为0")
                                            
                                            self.get_logger().info(f"检查计数: {self.count} > 5 = {self.count > 5}")
                                            
                                            if self.count > 5:
                                                self.get_logger().info("计数超过5，准备移动")
                                                self.count = 0
                                                self.target_shapes = None
                                                self.moving = True
                                                
                                                self.get_logger().info(f"obejct_info里面的内容: {obejct_info}")
                                                
                                                threading.Thread(target=self.move, args=(obejct_info,)).start()
                                                self.get_logger().info("移动线程已启动")
                                            self.last_position = obejct_info[1]
                                        else:
                                            self.get_logger().info(f"没有找到目标形状: {self.target_shapes}，重置目标")
                                            self.target_shapes = None

                                self.last_object_info_list = reorder_object_info_list
                            else:
                                self.get_logger().info("self.start为False，跳过处理")
                        else:
                            self.get_logger().info("self.moving为True，跳过形状识别")

                    bgr_image = rgb_image.copy()
                    cv2.rectangle(bgr_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (255, 255, 0), 1)

                    self.fps.update()
                    result_image = np.concatenate([depth_color_map, bgr_image], axis=1)
                    cv2.imshow("depth", result_image)
                    key = cv2.waitKey(1)
                    if key == ord('q') or key == 27:
                        self.running = False
                except Exception as e:
                    import traceback
                    self.get_logger().error(traceback.format_exc())
            except Exception as e:
                import traceback
                self.get_logger().error(traceback.format_exc())
                time.sleep(1)
            
        rclpy.shutdown()

def main():
    node = ObjectClassificationNode('object_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
