# _*_ coding:utf-8 _*_
from typing import Tuple, List
import time
import tkinter as tk
import threading
import RPi.GPIO as GPIO
import serial
import struct
import math
from threading import Lock

from models.Detection import Detection
from models.MoveControl import MoveControl
#from models.Angle import Angle
from models.config import (
    pi,
    color_list,
    color_list_in_fine_area,
    color_list_in_rough_area,
    dis_between_every_circle,
    HeightMode,
    MixMode
)

# 设置一键启动的 GPIO 引脚
BUTTON_PIN = 26  # 假设按键连接到 GPIO 21
GPIO.setmode(GPIO.BCM)  # 使用 BCM 编号
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 设置为输入模式，启用内部上拉电阻

# 整体流程类，控制小车的路径
class LogisticsCom(object):
    def __init__(self, stm32_port: str, stm32_baudrate: int, qr_cam: int , downward_cam: int):
        self.stm32_port = stm32_port
        self.stm32_baudrate = stm32_baudrate
        self.qr_cam = qr_cam
        self.downward_cam = downward_cam

        self.WT901C_ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=9600,
        timeout=0.1
        )


        self.moveControl = MoveControl(self.stm32_port, self.stm32_baudrate)
        self.qr_detection = Detection(self.qr_cam)

        self.downward_detection = None

    # 定义按键回调函数
    def button_callback(self,channel):
        if GPIO.input(channel) == GPIO.LOW:  # 检测到按键按下（低电平）
            return True
    
    def set_debug(self, whether_debug: bool):
        if whether_debug:
            self.debug = True
        else:
            self.debug = False

    # 从二维码扫描结果中，获取两次的任务
    def get_taskinfo(self, task_info: str) -> Tuple[List[int], List[int]]:
        task1_num = [int(task_str) for task_str in task_info[:3]]
        task2_num = [int(task_str) for task_str in task_info[4:]]

        task1_colors = [color_list[i] for i in task1_num]
        task2_colors = [color_list[i] for i in task2_num]


        return task1_colors, task2_colors
        

    def adjustLine(self, angel_thresh=0.6, dis_thresh=0.05, timeout=15):
        t_start = time.time()
        time.sleep(1.2)
        while True:
            # 获取直线信息
            avg_angle, dy_ratio = self.downward_detection.get_line_info()
            print(avg_angle, dy_ratio)
            # 调整逻辑
            if abs(avg_angle) > angel_thresh:
                if avg_angle > 0:
                    self.moveControl.rotate(-1)
                elif avg_angle < 0:
                    self.moveControl.rotate(1)
                continue  # 优先调整角度
            # 检查退出条件
            if abs(avg_angle) < angel_thresh:
                #self.moveControl.set_distance(x=-0.83)
                break
            
            if time.time() - t_start > timeout:
                print("调整超时")
                break     


    def get_z_angle(self):  
        prev_angle = None
        
        while True:
            # 读取包头
            header = self.WT901C_ser.read(1)
            if header != b'\x55':
                continue  # 非包头继续等待
            # 读取包类型
            type_byte = self.WT901C_ser.read(1)
            if type_byte != b'\x53':
                continue  # 非角度包跳过
            # 读取数据段（9字节）
            data_packet = self.WT901C_ser.read(9)
            if len(data_packet) != 9:
                continue  # 数据不完整
            # 校验和验证
            received_checksum = data_packet[-1]
            calculated_checksum = (0x55 + 0x53 + sum(data_packet[:-1])) & 0xFF
            if calculated_checksum != received_checksum:
                print(f"校验失败 预期:{calculated_checksum:02X} 实际:{received_checksum:02X}")
                continue
        
            yawH = data_packet[5]
            yawL = data_packet[6]
            self.current_angle = struct.unpack('>h', bytes([yawH, yawL]))[0] / 32768 * 180
            
            if prev_angle is not None and self.current_angle == prev_angle:
                continue
            prev_angle = self.current_angle


    def adjust_angle(self, angle):
        angle_thresh = 0.2
        every_dis = 1.4

        before_angle = self.current_angle
        print("当前角度", before_angle)
        self.moveControl.rotate(angle)
        after_angle = self.current_angle
        while after_angle == before_angle:
            after_angle = self.current_angle
        # 计算原始角度差值
        raw_delta = after_angle - before_angle
        # 角度解缠绕
        if raw_delta > 180:
            raw_delta -= 360
        elif raw_delta < -180:
            raw_delta += 360
        turn_angle = raw_delta

        dis_angle = angle - turn_angle

        # 新增条件：当 dis_angle 绝对值超过 270 时，二次修正（避免极端跳变）
        if abs(dis_angle) > 270:
            dis_angle = dis_angle - 360 * (1 if dis_angle > 0 else -1)
            print("触发大角度修正，修正后偏差:", dis_angle)

        print("转向后：", after_angle, "实际旋转角度:", turn_angle, "偏差：", dis_angle)

        # 计算需要调整的次数（取绝对值并向上取整）
        if abs(dis_angle) > angle_thresh:
            change_times = math.ceil(abs(dis_angle) / every_dis)
            if dis_angle > 0:
                for _ in range(change_times):
                    self.moveControl.rotate(1)  # 正向调整
            elif dis_angle < 0:
                for _ in range(change_times):
                    self.moveControl.rotate(-1)  # 反向调整
        else:
            print("角度调整完成")




    # 根据圆心调整小车X，Y位置
    def adjustCircle(self , color , thresh=0.02, d_thresh=0.12 , dis_max=127):
        # 矫正值
        clb_x = -1
        clb_y = 8
        while True:
            time.sleep(0.1)
            d_x, d_y = self.downward_detection.get_colored_circle_center(color)
            # d_x, d_y = self.downward_detection.get_circle_center()
            print(d_x, d_y)
            if (abs(d_y) > d_thresh or abs(d_x) > d_thresh):
                print('过位，开始重排')
                if abs(d_y) > thresh:
                    self.moveControl.move_in_mm(y=5 * (d_y//abs(d_y)))
                if abs(d_x) > thresh:
                    self.moveControl.move_in_mm(x=5 * (d_x//abs(d_x)))
                d_x, d_y = self.downward_detection.get_circle_center()
                print(d_x, d_y)
            if (abs(d_y) > thresh or abs(d_x) > thresh):
                if abs(d_y) > thresh:
                    self.moveControl.move_in_mm(y=d_y * dis_max * 0.75 + clb_y)
                else:
                    self.moveControl.move_in_mm(y=clb_y)
                if abs(d_x) > thresh:
                    self.moveControl.move_in_mm(x=-d_x * dis_max + clb_x)
                else:
                    self.moveControl.move_in_mm(x=clb_x)
            else:
                self.moveControl.move_in_mm(y=clb_y)
                self.moveControl.move_in_mm(x=clb_x)
            break


    # 根据圆心调整小车X，Y位置,码垛用
    def adjustCircle_second(self, thresh=0.05, dis_max=127, timeout=30):
        t1 = time.time()
        while True:
            d_x, d_y = self.downward_detection.get_colored_circle_center_second()
            print(d_x, d_y)
            if abs(d_y) > thresh:
                self.moveControl.move_in_mm(y=d_y * dis_max * 0.75)
                continue
            if abs(d_x) > thresh:
                self.moveControl.move_in_mm(x=-d_x * dis_max)
            if abs(d_x) < thresh and abs(d_y) < thresh:
                self.moveControl.move_in_mm(x=-3)
                self.moveControl.move_in_mm(y=9)
                break
            if time.time() - t1 > timeout:
                break

    def adjustCircle_grub(self, color , thresh=0.10):
        d_x, d_y = self.downward_detection.get_colored_circle_center(color)
        print(d_x, d_y)
        if abs(d_x) > thresh:
            if d_x < 0:
                self.moveControl.move_in_mm(x=13)
            elif d_x > 0:
                self.moveControl.move_in_mm(x=-13)
        if abs(d_y) > thresh:
            if d_y > 0:
                self.moveControl.move_in_mm(y=13)
            elif d_y < 0:
                self.moveControl.move_in_mm(y=-13)

    def show_code_in_window(self, qr_info):
        # 创建主窗口
        root = tk.Tk()
        # 设置窗口标题
        root.title("QR_code_get")
        # 最大化到屏幕大小
        w, h = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"{w}x{h}")
        # 创建一个Label来显示文本，并设置字体大小
        label = tk.Label(root, text=qr_info, font=("Helvetica", 140))
        label.pack(pady=70, padx=20)  # 设置Label的填充，使文本居中
        # 运行主循环
        root.mainloop()
        

    # 精加工区域码垛，根据物料块圆心位置，调整小车的X、Y位置
    def adjustColorCircle(self, thresh=0.03, dis_max=60, timeout=30, color='blue'):
        t1 = time.time()
        while True:
            d_x, d_y = self.downward_detection.get_colored_circle_center(color)
            if abs(d_x) > thresh:
                self.moveControl.move_in_mm(y=-d_x * dis_max)
                continue
            if abs(d_y) > thresh:
                self.moveControl.move_in_mm(x=-d_y * dis_max * 0.75)
            if abs(d_x) < thresh and abs(d_y) < thresh:

                break
            if time.time() - t1 > timeout:
                break

    def continueOrNot(self):
        x = input('是否继续程序:')
        if x == 'y' or x == 'Y':
            pass
        else:
            exit(0)
        
    def run_com(self) -> bool:

        # --------------------------------------------------
        #               第一圈
        # --------------------------------------------------

        # 离开启停区
        print("--开始第一圈--")
        print("前往二维码位置…")
        window_thread = threading.Thread(target=self.get_z_angle)
        window_thread.daemon = True  # 设置为守护线程，主线程退出时自动关闭窗口
        window_thread.start()
        time.sleep(0.5)
        self.moveControl.set_distance(y=0.13)

        # 前往二维码
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.qrPos, move_dis=0.63)
        self.moveControl.set_distance(y=0.11)

        # 二维码区 识别二维码
        print("-开始识别二维码-")
        qr_info = self.qr_detection.get_qr_info()
        # 在新线程中显示二维码窗口
        window_thread = threading.Thread(target=self.show_code_in_window, args=(qr_info,))
        window_thread.daemon = True  # 设置为守护线程，主线程退出时自动关闭窗口
        window_thread.start()

        task1_color, task2_color = self.get_taskinfo(qr_info)
        print(task1_color, task2_color)
        
        def switch_cameras():
            del self.qr_detection
            # 初始化新摄像头
            time.sleep(2.5)  # 延长等待时间
            self.downward_detection = Detection(self.downward_cam)
        switch_thread = threading.Thread(target=switch_cameras)
        switch_thread.start()
        
        # 二维码区到原料区
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.rawArea, move_dis=0.83)
        self.moveControl.set_distance(y=-0.15)
        
        # 到达原料区，调整高度，识别颜色，抓取物块
        for i, color in enumerate(task1_color):
            self.moveControl.adjustHeight(target_height=HeightMode.rawArea)
            if i < 2:
                self.downward_detection.detect_color(color)
                self.moveControl.grubBlockFromRawArea()
            else:
                self.downward_detection.detect_color(color)
                self.moveControl.grubBlockFromRawArea()
                print("全部抓取，离开")
                self.moveControl.set_distance(y=0.12)

        self.moveControl.rotate(1) 
        self.moveControl.set_distance(x=-0.44)

        self.adjust_angle(90)

        # 移动过去并调整高度
        self.moveControl.set_distance(x=0.54)

        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=1.22)
        self.adjust_angle(90)
        self.moveControl.set_distance(y=-0.06)

        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0)
        # task1_color = ["blue", "red", "green"]
        # task2_color = ["blue", "green", "red"]
        # 校准圆心，并放下物块(绿色位于中央)

        # 初始位置为 green
        current_color = "green"
        color_order = ["blue", "green", "red"]
        # 遍历任务码中的颜色顺序
        for i, target_color in enumerate(task1_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            
            self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea,move_dis=dis)
            self.adjustCircle(target_color)
            self.moveControl.putBlockToCircle()
            # 更新当前颜色
            current_color = target_color

        for i, target_color in enumerate(task1_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            if i!=0:
                self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.roughAreaLeave)
            else:self.moveControl.move_while_adjusting_height(mix_mode=MixMode.qrPos, move_dis=dis)
            # 更新当前颜色
            current_color = target_color
            if i==2 :
                self.moveControl.grubBlockFromGround()
                self.moveControl.set_distance(y=0.13)
                target_index = color_order.index("green")
                current_index = color_order.index(current_color)
                count_num = current_index - target_index
                dis = count_num * dis_between_every_circle - 0.85 -0.01#最右边圆环到拐角距离
                self.moveControl.move_while_adjusting_height(move_dis=dis , mix_mode=MixMode.circleArea)

        self.adjust_angle(-90)        

        # 校准圆心，并放下物块
        current_color = "green"
        for i, target_color in enumerate(task1_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            if i == 0:
                # 粗加工区右边拐角到暂存区的距离
                self.moveControl.move_while_adjusting_height(move_dis= -0.82 + dis, mix_mode=MixMode.circleArea)
                self.moveControl.set_distance(y=-0.08)
            else:
                self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea,move_dis=dis)
            self.adjustCircle(target_color)
            self.moveControl.putBlockToCircle()
            # 更新当前颜色
            current_color = target_color
            if i==2 :
                self.moveControl.set_distance(y=0.13)
                target_index = color_order.index("green")
                current_index = color_order.index(current_color)
                count_num = current_index - target_index
                dis = count_num * dis_between_every_circle - 0.87 - 0.05#最右边圆环到拐角距离
                self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.circleArea)

        # 到暂存区与原料区的拐弯
        self.adjust_angle(-90)
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.rawArea, move_dis=-0.40)
        # 到原料区
        self.moveControl.set_distance(y=-0.07)
        print("完成第一圈")
        # --------------------------------------------------
        #                 第二圈
        # --------------------------------------------------
        # del self.qr_detection
        # self.downward_detection = Detection(self.downward_cam)
        # task2_color = ["blue", "red", "green"]

        # 到达原料区，调整高度，识别颜色，抓取物块
        for i, color in enumerate(task2_color):
            self.moveControl.adjustHeight(target_height=HeightMode.rawArea)
            if i < 2:
                self.downward_detection.detect_color(color)
                self.moveControl.grubBlockFromRawArea()
            else:
                self.downward_detection.detect_color(color)
                self.moveControl.grubBlockFromRawArea()
                print("全部抓取，离开")
                self.moveControl.set_distance(y=0.10)

        # self.moveControl.rotate(1) 
        self.moveControl.set_distance(x=-0.44)

        self.adjust_angle(90)

        # 移动过去并调整高度
        self.moveControl.set_distance(x=0.54)

        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=1.22)
        self.adjust_angle(90)
        self.moveControl.set_distance(y=-0.07)

        # 初始位置为 green
        current_color = "green"
        color_order = ["blue", "green", "red"]
        # 遍历任务码中的颜色顺序
        for i, target_color in enumerate(task2_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            
            self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea,move_dis=dis)
            self.adjustCircle(target_color)
            self.moveControl.putBlockToCircle()
            # 更新当前颜色
            current_color = target_color

        for i, target_color in enumerate(task2_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            if i!=0:
                self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.roughAreaLeave)
            else:self.moveControl.move_while_adjusting_height(mix_mode=MixMode.qrPos, move_dis=dis)
            # 更新当前颜色
            current_color = target_color
            if i==2 :
                self.moveControl.grubBlockFromGround()
                self.moveControl.set_distance(y=0.13)
                target_index = color_order.index("green")
                current_index = color_order.index(current_color)
                count_num = current_index - target_index
                dis = count_num * dis_between_every_circle - 0.85 -0.01#最右边圆环到拐角距离
                self.moveControl.move_while_adjusting_height(move_dis=dis , mix_mode=MixMode.circleArea)
        self.adjust_angle(-90)
        # self.adjustLine()

        # 粗加工区右边拐角到暂存区的距离
        self.moveControl.move_while_adjusting_height(move_dis=-0.79, mix_mode=MixMode.rawArea)
        self.moveControl.set_distance(y=-0.08)
        # self.moveControl.move_in_mm(y=-5)

        # 校准圆心，并放下物块
        self.adjustCircle_second()
        current_color = "green"
        # 遍历任务码中的颜色顺序
        for i, target_color in enumerate(task2_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            self.moveControl.set_distance(x=dis)
            self.moveControl.putBlockToAnotherBlock()
            # 更新当前颜色
            current_color = target_color
            if i==2 :
                self.moveControl.set_distance(y=0.13)
                target_index = color_order.index("green")
                current_index = color_order.index(current_color)
                count_num = current_index - target_index
                dis = count_num * dis_between_every_circle - 0.08
                self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.rawArea)

        self.moveControl.set_distance(x=0.06)
        self.adjust_angle(90)
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0.53)
        self.adjustLine()
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.rawArea, move_dis=1.22)
        self.adjust_angle(90)
        self.moveControl.set_distance(x=0.93)
        self.adjust_angle(90)
        
        self.moveControl.final()



    def just_RUN(self):
        
        # --------------------------------------------------
        #               第一圈
        # --------------------------------------------------

        # 离开启停区
        print("--开始第一圈--")
        print("前往二维码位置…")
        time.sleep(0.5)
        window_thread = threading.Thread(target=self.get_z_angle)
        window_thread.daemon = True  # 设置为守护线程，主线程退出时自动关闭窗口
        window_thread.start()
        self.moveControl.set_distance(y=0.13)

        # 前往二维码
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.qrPos, move_dis=0.63)
        self.moveControl.set_distance(y=0.13)

        # 二维码区 识别二维码
        print("-开始识别二维码-")
    
        # 二维码区到原料区
        def switch_cameras():
            # 释放旧摄像头资源
            if hasattr(self, 'qr_detection'):
                del self.qr_detection
            # 初始化新摄像头
            self.downward_detection = Detection(self.downward_cam)
        switch_thread = threading.Thread(target=switch_cameras)
        switch_thread.start()
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.rawArea, move_dis=0.83)
        self.moveControl.set_distance(y=-0.13)
        self.moveControl.move_in_mm(y=-17)

        self.moveControl.set_distance(y=0.12)
        self.moveControl.rotate(1)
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=-0.46)
        self.adjust_angle(90)

        # 移动过去并调整高度
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0.54)

        self.moveControl.set_distance(x=1.22)
        self.adjust_angle(90)
        self.moveControl.set_distance(y=-0.06)


        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0)

        dis =- 0.85 -0.01#最右边圆环到拐角距离
        self.moveControl.move_while_adjusting_height(move_dis=dis , mix_mode=MixMode.circleArea)

        self.adjust_angle(-90)


        # 粗加工区右边拐角到暂存区的距离
        self.moveControl.move_while_adjusting_height(move_dis=-0.81, mix_mode=MixMode.circleArea)
        self.moveControl.move_in_mm(x=3)
        self.moveControl.set_distance(y=-0.10)
        self.moveControl.move_in_mm(y=-2)
        
        # 校准圆心，并放下物块
        self.moveControl.move_in_mm(x=-5)

        self.moveControl.set_distance(y=0.13)

        dis = -0.87 #最右边圆环到拐角距离
        self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.circleArea)

        # 到暂存区与原料区的拐弯
        self.adjust_angle(-90)
        self.moveControl.set_distance(y=-0.07)
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.rawArea, move_dis=-0.40)
        
        # 到原料区
        self.moveControl.set_distance(y=-0.05)
        print("完成第一圈")
        # --------------------------------------------------
        #                 第二圈
        # --------------------------------------------------

        # 到达原料区，调整高度，识别颜色，抓取物块
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=-0.43)

        self.adjust_angle(90)

        # 移动过去并调整高度
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0.54)
        self.moveControl.set_distance(x=1.23)

        self.adjust_angle(90)

        self.moveControl.set_distance(y=-0.07)
        self.moveControl.move_in_mm(y=-6)

        self.moveControl.set_distance(y=0.13)
        dis =- 0.84 - 0.02 #最右边圆环到拐角距离
        self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.circleArea)

        self.adjust_angle(-90)

        # 粗加工区右边拐角到暂存区的距离
        self.moveControl.move_while_adjusting_height(move_dis=-0.79, mix_mode=MixMode.rawArea)
        self.moveControl.set_distance(y=-0.08)
        # self.moveControl.move_in_mm(y=-5)

        self.moveControl.set_distance(y=0.13)
        dis = 0.08
        self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.rawArea)

        self.moveControl.set_distance(x=0.06)

        self.adjust_angle(90)

        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0.53)
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.rawArea, move_dis=1.22)

        self.adjust_angle(90)

        self.moveControl.set_distance(x=0.93)

        self.adjust_angle(90)
        
        self.moveControl.final()



    # 快速校准圆环用
    def test_circle(self):
        # self.moveControl.move_while_adjusting_height(mix_mode=MixMode.qrPos, move_dis=0.0)
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0.0)
        del self.qr_detection
        time.sleep(2)
        self.downward_detection = Detection(2)
        self.adjustCircle("green")
        self.moveControl.putBlockToCircle()

    def test_angle(self):
        window_thread = threading.Thread(target=self.get_z_angle)
        window_thread.daemon = True  # 设置为守护线程，主线程退出时自动关闭窗口
        window_thread.start()
        time.sleep(1)
        print("开始执行")
        self.adjust_angle(90)
        time.sleep(1)
        self.adjust_angle(-90)
        time.sleep(1)
        self.adjust_angle(-90)
        time.sleep(1)
        self.adjust_angle(90)


    def test(self):
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.qrPos, move_dis=0)
        task1_color = ["red", "blue", "green"]
        task2_color = ["blue", "green", "red"]
        # del self.qr_detection
        # time.sleep(2)
        # self.downward_detection = Detection(self.downward_cam)

        # self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=0)
        self.moveControl.move_while_adjusting_height(mix_mode=MixMode.rawArea, move_dis=0)
        
        
        current_color = "green"
        color_order = ["blue", "green", "red"]
        
        # 遍历任务码中的颜色顺序
        for i, target_color in enumerate(task1_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            self.moveControl.move_while_adjusting_height(mix_mode=MixMode.circleArea, move_dis=dis)
            # self.adjustCircle(target_color)
            # self.moveControl.putBlockToCircle()
            current_color = target_color

        for i, target_color in enumerate(task1_color):
            # 计算当前颜色和目标颜色的索引
            current_index = color_order.index(current_color)
            target_index = color_order.index(target_color)
            count_num = current_index - target_index
            dis = count_num * dis_between_every_circle
            if i!=0:
                self.moveControl.move_while_adjusting_height(move_dis=dis, mix_mode=MixMode.roughAreaLeave)
            else:self.moveControl.move_while_adjusting_height(mix_mode=MixMode.qrPos, move_dis=dis)
            # 更新当前颜色
            current_color = target_color
            if i==2 :
                self.moveControl.grubBlockFromGround()
                self.moveControl.set_distance(y=0.13)
                target_index = color_order.index("green")
                current_index = color_order.index(current_color)
                count_num = current_index - target_index
                dis = count_num * dis_between_every_circle - 0.85 -0.01#最右边圆环到拐角距离
                self.moveControl.move_while_adjusting_height(move_dis=dis , mix_mode=MixMode.circleArea)



        # self.camera_flag = False
        # def switch_cameras(self):
        #     with self.cam_lock:  # 保证原子操作
        #         if hasattr(self, 'qr_detection'):
        #             del self.qr_detection
        #         time.sleep(0.5)  # 延长等待时间
        #         # 确保摄像头物理复位
        #         self.downward_detection = Detection(self.downward_cam)
        #         self.camera_flag = True

        # switch_thread = threading.Thread(target=switch_cameras)
        # switch_thread.start()
        # if self.camera_flag:
        #     if  self.downward_detection.detect_color("red"):
        #         print("检测到红色")





if __name__ == "__main__":
    stm32_port = "/dev/ttyAMA0"
    stm32_baudrate = 115200
    qr_cam_name = 0
    downward_cam_name = 2

    com = LogisticsCom(stm32_port, stm32_baudrate, qr_cam_name, downward_cam_name)
    # com.set_debug(True)
    print("- 等待启动中…… -")
    try:
        while True:
            #按键的一键启动
            if com.button_callback(BUTTON_PIN):
                print("====== 开始运行 ======")
                com.run_com()
                # com.test()
                # com.test_circle()
                # com.test_angle()
                # com.just_RUN()#测试定位跑圈
                print("====== 任务完成 ======")
                break
            else : continue
                
    except KeyboardInterrupt:
        GPIO.cleanup()  # 清理 GPIO 设置
    
# /home/tb_si/CAR/code/move.py