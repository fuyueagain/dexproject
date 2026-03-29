#!/usr/bin/env python3
"""
XLeRobot 抓取 "冰露" 杯子脚本 v2

功能:
1. 控制头部摄像头旋转查看环境
2. 使用 OCR 识别"冰露"标签
3. 控制机械臂抓取杯子

依赖:
- conda activate new-lerobot
- rapidocr, opencv-python, numpy 已安装

用法:
    conda activate new-lerobot
    python grab_binglu.py
"""

import cv2
import time
import numpy as np
import sys
sys.path.insert(0, '/home/makermods/lerobot-MakerMods/src')

from rapidocr import RapidOCR
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig


class BingluGrabber:
    def __init__(self, camera_index=0, left_arm_port="/dev/ttyACM0", right_arm_port="/dev/ttyACM1"):
        self.camera_index = camera_index
        self.ocr = RapidOCR()
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 {camera_index}")
        
        ret, frame = self.cap.read()
        if ret:
            print(f"摄像头分辨率: {frame.shape[1]}x{frame.shape[0]}")
        
        # 连接机械臂
        print("正在连接左臂...")
        self.left_arm = SO101Follower(SO101FollowerConfig(port=left_arm_port, id="arm_left"))
        self.left_arm.connect(calibrate=False)
        print(f"左臂连接状态: {self.left_arm.is_connected}")
        
        print("正在连接右臂...")
        self.right_arm = SO101Follower(SO101FollowerConfig(port=right_arm_port, id="arm_right"))
        self.right_arm.connect(calibrate=False)
        print(f"右臂连接状态: {self.right_arm.is_connected}")
        
        # 头部摄像头角度 (需要根据实际舵机调整)
        self.head_pan = 0      # 水平角度
        self.head_tilt = 0     # 垂直角度
        
    def __del__(self):
        """清理资源"""
        if hasattr(self, 'left_arm') and self.left_arm.is_connected:
            self.left_arm.disconnect()
        if hasattr(self, 'right_arm') and self.right_arm.is_connected:
            self.right_arm.disconnect()
        if hasattr(self, 'cap'):
            self.cap.release()
            
    def get_arm_position(self, arm):
        """读取机械臂当前关节角度"""
        state = arm.get_observation()
        arm_keys = [k for k in state.keys() if k.endswith('.pos')]
        return {k.replace('.pos', ''): state[k] for k in arm_keys}
        
    def set_arm_position(self, arm, positions, wait_time=1.0):
        """
        设置机械臂关节角度
        
        positions: dict, e.g. {
            'shoulder_pan': 0,
            'shoulder_lift': -30,
            'elbow_flex': 45,
            'wrist_flex': -30,
            'wrist_roll': 0,
            'gripper': 50
        }
        """
        action = {}
        for joint, pos in positions.items():
            action[f"{joint}.pos"] = pos
            
        arm.send_action(action)
        time.sleep(wait_time)
        
    def move_head(self, pan=None, tilt=None, wait_time=0.5):
        """
        移动头部摄像头
        注意: 需要根据实际硬件实现，这里只是模拟
        """
        if pan is not None:
            self.head_pan = np.clip(pan, -90, 90)
        if tilt is not None:
            self.head_tilt = np.clip(tilt, -45, 45)
            
        # TODO: 发送舵机控制命令到串口
        print(f"头部角度: pan={self.head_pan}, tilt={self.head_tilt}")
        time.sleep(wait_time)
        
    def scan_environment(self, steps=3):
        """扫描环境, 左右旋转摄像头"""
        results = []
        
        # 先往右转
        for i in range(steps):
            self.move_head(pan=(i + 1) * 30)
            result = self.detect_binglu()
            if result:
                results.append(result)
                print(f"在位置 {i} (右) 发现冰露!")
                
        # 回到中间
        self.move_head(pan=0)
        
        # 再往左转
        for i in range(steps):
            self.move_head(pan=-(i + 1) * 30)
            result = self.detect_binglu()
            if result:
                results.append(result)
                print(f"在位置 {i} (左) 发现冰露!")
                
        # 回到中间
        self.move_head(pan=0)
        
        return results
        
    def detect_binglu(self):
        """检测画面中的'冰露'文字"""
        ret, frame = self.cap.read()
        if not ret:
            return None
            
        # 保存当前帧
        cv2.imwrite('/home/makermods/dexproject/current_frame.jpg', frame)
        
        # OCR 识别
        result = self.ocr(frame)
        
        if result is None:
            return None
            
        boxes = result.boxes
        txts = result.txts
        scores = result.scores
        
        if boxes is None or len(boxes) == 0:
            return None
            
        # 查找"冰露"
        for i, txt in enumerate(txts):
            if txt and "冰露" in str(txt):
                box = boxes[i]
                score = scores[i]
                print(f"找到'冰露': '{txt}', 置信度: {score:.2f}, 位置: {box}")
                return {
                    'text': txt,
                    'score': score,
                    'box': box,
                    'frame': frame
                }
                
        return None
        
    def pixel_to_arm_target(self, box, frame_shape):
        """
        将像素坐标转换为机械臂目标位置
        这是一个简化的映射，需要根据实际标定结果调整
        """
        h, w = frame_shape[:2]
        
        # 计算目标中心点
        x_center = np.mean(box[:, 0])
        y_bottom = np.max(box[:, 1])  # 使用底部作为抓取点
        
        # 归一化到 -1 到 1
        x_norm = (x_center - w/2) / (w/2)
        y_norm = (y_bottom - h/2) / (h/2)
        
        print(f"目标像素: ({x_center:.0f}, {y_bottom:.0f}), 归一化: ({x_norm:.2f}, {y_norm:.2f})")
        
        # 简化的映射 (需要根据实际标定调整)
        # 假设:
        # - x_norm 控制肩部旋转 (shoulder_pan)
        # - y_norm 控制肩部抬起 (shoulder_lift)
        shoulder_pan = x_norm * 40      # 左右移动范围
        shoulder_lift = -y_norm * 30   # 上下移动范围
        
        arm_target = {
            'shoulder_pan': shoulder_pan,
            'shoulder_lift': shoulder_lift,
            'elbow_flex': 30,            # 固定弯曲角度
            'wrist_flex': -20,           # 固定弯曲角度
            'wrist_roll': 0,             # 不旋转
            'gripper': 80,               # 张开
        }
        
        return arm_target
        
    def grab(self, arm, target_pos):
        """
        执行抓取动作
        1. 移动到目标上方
        2. 下移接近
        3. 闭合夹爪
        """
        print(f"执行抓取, 目标位置: {target_pos}")
        
        # 读取当前位置
        current = self.get_arm_position(arm)
        print(f"当前位置: {current}")
        
        # 步骤1: 移动到目标上方 (抬高手臂)
        above_target = target_pos.copy()
        above_target['shoulder_lift'] = min(target_pos['shoulder_lift'] + 20, 30)
        above_target['gripper'] = 80  # 张开
        print("步骤1: 移动到目标上方")
        self.set_arm_position(arm, above_target)
        
        # 步骤2: 下移接近目标
        reach_target = target_pos.copy()
        reach_target['gripper'] = 80  # 张开
        print("步骤2: 下移接近目标")
        self.set_arm_position(arm, reach_target)
        
        # 步骤3: 闭合夹爪
        grab_target = target_pos.copy()
        grab_target['gripper'] = 20  # 闭合
        print("步骤3: 闭合夹爪")
        self.set_arm_position(arm, grab_target)
        
        # 步骤4: 抬起
        lift_target = above_target.copy()
        lift_target['gripper'] = 20  # 保持闭合
        print("步骤4: 抬起")
        self.set_arm_position(arm, lift_target)
        
        print("抓取完成!")
        
    def run(self):
        """主循环"""
        print("=" * 50)
        print("XLeRobot 冰露杯子抓取程序 v2")
        print("=" * 50)
        
        try:
            while True:
                print("\n1. 扫描环境寻找'冰露'杯子")
                print("2. 测试OCR识别")
                print("3. 打印当前机械臂位置")
                print("4. 测试夹爪")
                print("5. 退出")
                
                choice = input("请选择 (1/2/3/4/5): ").strip()
                
                if choice == '1':
                    print("\n开始扫描环境...")
                    results = self.scan_environment(steps=3)
                    
                    if results:
                        print(f"\n找到 {len(results)} 个目标!")
                        # 使用第一个检测到的目标
                        result = results[0]
                        arm_target = self.pixel_to_arm_target(
                            result['box'], 
                            result['frame'].shape
                        )
                        
                        # 使用左臂抓取
                        print("\n使用左臂执行抓取...")
                        self.grab(self.left_arm, arm_target)
                    else:
                        print("未找到'冰露'杯子,请调整摄像头角度或确保杯子在视野内")
                        
                elif choice == '2':
                    print("测试OCR识别 (每2秒检测一次)")
                    print("按 Ctrl+C 退出")
                    try:
                        while True:
                            ret, frame = self.cap.read()
                            if not ret:
                                print("无法读取摄像头")
                                break
                                
                            cv2.imwrite('/home/makermods/dexproject/current_frame.jpg', frame)
                            
                            result = self.ocr(frame)
                            if result and result.txts:
                                for txt in result.txts:
                                    if txt:
                                        print(f"检测到文字: {txt}")
                            
                            time.sleep(2)
                    except KeyboardInterrupt:
                        print("\n退出测试")
                        
                elif choice == '3':
                    print("\n左臂位置:")
                    left_pos = self.get_arm_position(self.left_arm)
                    for joint, pos in left_pos.items():
                        print(f"  {joint}: {pos:.2f}°")
                        
                    print("\n右臂位置:")
                    right_pos = self.get_arm_position(self.right_arm)
                    for joint, pos in right_pos.items():
                        print(f"  {joint}: {pos:.2f}°")
                        
                elif choice == '4':
                    print("\n测试夹爪...")
                    gripper_values = [80, 50, 20, 50, 80]
                    for val in gripper_values:
                        print(f"设置夹爪: {val}")
                        self.left_arm.send_action({'gripper.pos': val})
                        time.sleep(1)
                        
                elif choice == '5':
                    print("退出程序")
                    break
                    
        finally:
            self.left_arm.disconnect()
            self.right_arm.disconnect()
            self.cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    
    # 默认使用第一个摄像头
    camera_idx = 0
    if len(sys.argv) > 1:
        camera_idx = int(sys.argv[1])
    
    grabber = BingluGrabber(camera_index=camera_idx)
    grabber.run()