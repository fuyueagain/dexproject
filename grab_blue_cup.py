#!/usr/bin/env python3
"""
XLeRobot 抓取蓝色包装杯子

功能:
1. 检测画面中的蓝色物体
2. 控制机械臂抓取

依赖: conda activate new-lerobot
"""

import cv2
import sys
import time
import numpy as np
sys.path.insert(0, '/home/makermods/lerobot-MakerMods/src')

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig


class BlueCupGrabber:
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 {camera_index}")
        
        ret, frame = self.cap.read()
        if ret:
            print(f"摄像头分辨率: {frame.shape[1]}x{frame.shape[0]}")
        
        # 连接机械臂
        print("连接左臂...")
        self.left_arm = SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="arm_left"))
        self.left_arm.connect(calibrate=False)
        print(f"左臂连接状态: {self.left_arm.is_connected}")
        
    def __del__(self):
        if hasattr(self, 'left_arm') and self.left_arm.is_connected:
            self.left_arm.disconnect()
        if hasattr(self, 'cap'):
            self.cap.release()
            
    def get_arm_position(self, arm):
        state = arm.get_observation()
        return {k.replace('.pos', ''): state[k] for k in state.keys() if k.endswith('.pos')}
        
    def set_arm_position(self, arm, positions, wait_time=1.0):
        action = {f"{joint}.pos": pos for joint, pos in positions.items()}
        arm.send_action(action)
        time.sleep(wait_time)
        
    def detect_blue_objects(self, frame):
        """
        检测画面中的蓝色物体
        使用 HSV 颜色空间检测蓝色
        """
        # 转换到 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 蓝色的 HSV 范围 (根据实际调整)
        # 浅蓝色: H=100, S=50, V=50
        # 深蓝色: H=130, S=255, V=255
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        
        # 创建掩码
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # 形态学处理去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # 过滤太小的区域
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w//2, y + h//2  # 中心点
                results.append({
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'cx': cx, 'cy': cy,
                    'area': area
                })
                # 画框
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        
        return frame, mask, results
        
    def pixel_to_arm_target(self, obj, frame_shape):
        """将像素坐标转换为机械臂目标"""
        h, w = frame_shape[:2]
        
        x_norm = (obj['cx'] - w/2) / (w/2)
        y_norm = (obj['cy'] - h/2) / (h/2)
        
        print(f"目标: 像素({obj['cx']}, {obj['cy']}), 归一化({x_norm:.2f}, {y_norm:.2f})")
        
        # 简化的映射 (需根据实际标定调整)
        shoulder_pan = x_norm * 40
        shoulder_lift = -y_norm * 30
        
        return {
            'shoulder_pan': shoulder_pan,
            'shoulder_lift': shoulder_lift,
            'elbow_flex': 30,
            'wrist_flex': -20,
            'wrist_roll': 0,
            'gripper': 80,
        }
        
    def grab(self, arm, target_pos):
        """执行抓取"""
        print(f"抓取位置: {target_pos}")
        
        # 1. 到目标上方
        above = target_pos.copy()
        above['shoulder_lift'] = min(target_pos['shoulder_lift'] + 20, 30)
        above['gripper'] = 80
        print("  1. 到上方...")
        self.set_arm_position(arm, above)
        
        # 2. 下移接近
        reach = target_pos.copy()
        reach['gripper'] = 80
        print("  2. 下移接近...")
        self.set_arm_position(arm, reach)
        
        # 3. 闭合夹爪
        grab_pos = target_pos.copy()
        grab_pos['gripper'] = 20
        print("  3. 闭合夹爪...")
        self.set_arm_position(arm, grab_pos)
        
        # 4. 抬起
        lift = above.copy()
        lift['gripper'] = 20
        print("  4. 抬起")
        self.set_arm_position(arm, lift)
        
        print("抓取完成!")
        
    def run(self):
        print("=" * 50)
        print("XLeRobot 蓝色杯子抓取程序")
        print("=" * 50)
        
        try:
            while True:
                print("\n1. 扫描并抓取蓝色杯子")
                print("2. 实时检测蓝色 (查看效果)")  
                print("3. 打印当前臂位置")
                print("4. 测试夹爪")
                print("5. 退出")
                
                choice = input("选择: ").strip()
                
                if choice == '1':
                    print("\n扫描环境...")
                    
                    # 获取画面
                    ret, frame = self.cap.read()
                    if not ret:
                        print("无法读取摄像头")
                        continue
                    
                    cv2.imwrite('/home/makermods/dexproject/current_frame.jpg', frame)
                    
                    # 检测蓝色
                    result_frame, mask, blue_objects = self.detect_blue_objects(frame)
                    
                    if blue_objects:
                        print(f"检测到 {len(blue_objects)} 个蓝色物体!")
                        
                        # 取最大的那个
                        target = max(blue_objects, key=lambda x: x['area'])
                        print(f"选择最大目标: 面积={target['area']}")
                        
                        # 计算抓取位置
                        arm_target = self.pixel_to_arm_target(target, frame.shape)
                        
                        # 执行抓取
                        self.grab(self.left_arm, arm_target)
                    else:
                        print("未检测到蓝色物体，请调整颜色范围或确保蓝色物体在视野内")
                        cv2.imwrite('/home/makermods/dexproject/blue_mask.jpg', mask)
                        print("已保存 mask 图片供调试")
                        
                elif choice == '2':
                    print("实时检测... Ctrl+C 退出")
                    try:
                        while True:
                            ret, frame = self.cap.read()
                            if not ret:
                                break
                                
                            result, mask, objects = self.detect_blue_objects(frame)
                            
                            # 显示数量
                            cv2.putText(result, f"Blue objects: {len(objects)}", 
                                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            
                            cv2.imwrite('/home/makermods/dexproject/blue_debug.jpg', result)
                            print(f"检测到: {len(objects)} 个蓝色物体")
                            time.sleep(1)
                    except KeyboardInterrupt:
                        print("\n退出")
                        
                elif choice == '3':
                    pos = self.get_arm_position(self.left_arm)
                    print("\n左臂位置:")
                    for joint, val in pos.items():
                        print(f"  {joint}: {val:.1f}°")
                        
                elif choice == '4':
                    print("测试夹爪...")
                    for val in [80, 50, 20, 50, 80]:
                        self.left_arm.send_action({'gripper.pos': val})
                        print(f"  夹爪 -> {val}")
                        time.sleep(0.5)
                        
                elif choice == '5':
                    break
                    
        finally:
            self.left_arm.disconnect()
            self.cap.release()


if __name__ == "__main__":
    grabber = BlueCupGrabber()
    grabber.run()