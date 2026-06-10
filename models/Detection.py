from typing import Tuple, List
import numpy as np
import cv2 as cv
from pyzbar.pyzbar import decode
from imutils import contours
import math
from collections import deque
import matplotlib.pyplot as plt
import time

#from models.config import pi, color_list
pi = 3.14159
color_list = [0, "red", "green", "blue"]

# 输入两个点的坐标，获取两个点之间的距离，并取整
def Distance(x1, y1, x2, y2) -> int:
    x = abs(x1 - x2)
    y = abs(y1 - y2)
    return int(round(math.sqrt(x * x + y * y)))


# 图像识别类，包含摄像头的初始化以及后续的识别
class Detection(object):
    # 颜色矩阵
    lower_green = np.array([40, 31, 54])
    upper_green = np.array([100, 195, 157])
    # lower_green = np.array([50, 35, 0])
    # upper_green = np.array([105, 255, 255])
    lower_blue = np.array([0, 50 ,60])
    upper_blue = np.array([35, 255, 255])
    lower_red = np.array([100, 50, 20])
    upper_red = np.array([150, 255, 200])

    def __init__(self, camera_index) -> None:
        self.camera_index = camera_index
        self.cap = cv.VideoCapture(camera_index, cv.CAP_V4L2)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap_flag = False
        if self.cap.isOpened():
            pass
        elif not self.cap_flag:
            self.cap_flag = True
            print("摄像头打开失败,尝试更换摄像头")
            self.cap = cv.VideoCapture(3, cv.CAP_V4L2)
            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        elif self.cap_flag:
            self.cap_flag = False
            print("摄像头打开失败,尝试更换摄像头")
            self.cap = cv.VideoCapture(2, cv.CAP_V4L2)
            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("摄像头初始化成功")
        _, frame = self.cap.read()
        frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        self.width = frame.shape[1]
        self.height = frame.shape[0]
        self.debug = False
        self.cv_big_version = int(cv.__version__[0])
                # 状态缓存
        self.last_angle = 0
        self.last_distance_ratio_y = 0
        self.last_valid_angle = 0  # 最后有效角度
        
        # 初始化卡尔曼滤波器
        self._init_kalman_filter()
        
        # 角度变化阈值
        self.angle_threshold = 40  # 最大允许角度跳变（度）

    def set_debug(self, whether_debug: bool):
        if whether_debug:
            self.debug = True
        else:
            self.debug = False

    def skip_some(self, count=10):
        for i in range(count):
            _, _ = self.cap.read()

    def _init_kalman_filter(self):
        """初始化角度卡尔曼滤波器"""
        self.kalman_angle = cv.KalmanFilter(2, 1)  # 2个状态量（角度和角速度），1个观测量
        
        # 状态转移矩阵（假设匀速运动模型）
        self.kalman_angle.transitionMatrix = np.array([
            [1, 1],  # 角度 = 前一角度 + 角速度
            [0, 1]   # 角速度保持不变
        ], dtype=np.float32)
        
        # 观测矩阵（只能观测到角度）
        self.kalman_angle.measurementMatrix = np.array([[1, 0]], dtype=np.float32)
        
        # 过程噪声协方差（越小越信任模型）
        self.kalman_angle.processNoiseCov = np.eye(2, dtype=np.float32) * 1e-4
        
        # 观测噪声协方差（越小越信任测量）
        self.kalman_angle.measurementNoiseCov = np.array([[0.1]], dtype=np.float32)
        
        # 后验误差协方差初始化（修正为正确维度）
        self.kalman_angle.errorCovPost = np.eye(2, dtype=np.float32)
        
        # 初始状态：角度=0，角速度=0（修正为正确维度）
        self.kalman_angle.statePost = np.zeros((2, 1), dtype=np.float32)  # 改为二维列向量

    # 获取二维码信息
    def get_qr_info(self, data_len=-1) -> str:
        data = ""
        data_flag = False
        while True:
            _, frame = self.cap.read()
            for barcode in decode(frame):
                data_ = barcode.data.decode("utf-8")
                if self.debug:
                    (x, y, w, h) = barcode.rect
                    cv.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    cv.putText(
                        frame, data_, (x, y), cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255)
                    )
                # 没有数据长度限制
                if not self.debug:
                    if data_len == -1:
                        data = data_
                        data_flag = True
                        break
                    else:
                        if len(data_) == data_len:
                            data = data_
                            data_flag = True
                            break

            if self.debug:
                cv.imshow("frame", frame)
                k = cv.waitKey(1) & 0xFF
                if k == 27:
                    cv.destroyWindow("frame")
                    break
            if not self.debug and data_flag:
                break 
        print("-已获取二维码-")
        return data
        
    def _unwrap_angle(self, new_angle, prev_angle):
        """处理角度环绕问题（-180到180的跳变）"""
        diff = new_angle - prev_angle
        if diff > 180:
            return new_angle - 360
        elif diff < -180:
            return new_angle + 360
        return new_angle
    
    def get_line_info(self):
        self.skip_some()
        count = 8  # 采样帧数
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            height, width = frame.shape[:2]
            display_frame = frame.copy()

            # ================== 颜色检测流程 ==================
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            lower_color = np.array([100, 0, 62])
            upper_color = np.array([180, 255, 255])
            mask = cv.inRange(hsv, lower_color, upper_color)
            
            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
            
            # 设置ROI
            margin = 50
            mask[:margin, :] = 0
            mask[-margin:, :] = 0
            mask[:, :margin] = 0
            mask[:, -margin:] = 0
            
            # 轮廓检测
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            # 过滤轮廓
            filtered_contours = []
            min_area = 500
            for cnt in contours:
                if cv.contourArea(cnt) < min_area:
                    continue
                M = cv.moments(cnt)
                if M["m00"] == 0:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if (margin < cx < width - margin) and (margin < cy < height - margin):
                    filtered_contours.append(cnt)
            
            # 获取主方向
            max_line = None
            if filtered_contours:
                largest = max(filtered_contours, key=cv.contourArea)
                (x1, y1), (x2, y2) = self._get_main_axis_from_contour(largest)
                max_line = np.array([[[x1, y1, x2, y2]]])
            # ================================================

            current_angle = None
            distance_ratio_y = None

            if max_line is not None:
                x1, y1, x2, y2 = max_line[0][0]
                
                # 计算原始角度
                dx = x2 - x1
                dy = y2 - y1
                raw_angle = np.arctan2(-dy, dx) * 180 / np.pi
                if raw_angle > 0:
                    raw_angle = 180 - raw_angle
                else:
                    raw_angle = -180 - raw_angle
                
                # 角度解缠绕
                unwrapped_angle = self._unwrap_angle(raw_angle, self.last_valid_angle)
                
                # 异常值检测
                if abs(unwrapped_angle - self.last_valid_angle) <= self.angle_threshold:
                    # 卡尔曼预测
                    prediction = self.kalman_angle.predict()
                    
                    # 更新测量值
                    measurement = np.array([[unwrapped_angle]], dtype=np.float32)
                    self.kalman_angle.correct(measurement)
                    
                    # 获取滤波后状态
                    filtered_angle = self.kalman_angle.statePost[0, 0]
                    
                    # 更新状态
                    self.last_valid_angle = filtered_angle
                    current_angle = filtered_angle
                    count -= 1
                else:
                    # 使用预测值保持稳定
                    current_angle = self.kalman_angle.predict()[0, 0]

                # 计算距离比例
                midpoint = ((x1 + x2) // 2, (y1 + y2) // 2)
                center_y = height // 2
                distance_ratio_y = (midpoint[1] - center_y) / (height // 2)
                self.last_distance_ratio_y = distance_ratio_y

            else:
                # 无检测时使用预测值
                prediction = self.kalman_angle.predict()
                current_angle = prediction[0, 0]

            # 调试显示
            if self.debug:
                text_angle = f"Angle: {current_angle:.1f}°" if current_angle is not None else "Angle: N/A"
                text_dist = f"DistY: {self.last_distance_ratio_y:.2f}" if self.last_distance_ratio_y is not None else "DistY: N/A"
                
                if max_line is not None:
                    x1, y1, x2, y2 = max_line[0][0]
                    cv.line(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv.circle(display_frame, midpoint, 5, (0, 255, 255), -1)
                
                # 绘制参考线
                cv.line(display_frame, (0, height//2), (width, height//2), (255, 0, 0), 1)
                cv.line(display_frame, (width//2, 0), (width//2, height), (255, 0, 0), 1)
                
                # 显示文本
                cv.putText(display_frame, text_angle, (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv.putText(display_frame, text_dist, (10, 70), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                print(current_angle, self.last_distance_ratio_y)
                
                cv.imshow('Mask', mask)
                cv.imshow('Frame', display_frame)
                if cv.waitKey(1) == 27:
                    break

            if count < 1 and not self.debug:
                break
 
        return current_angle, self.last_distance_ratio_y

    def _get_main_axis_from_contour(self, contour):
        """PCA获取主方向"""
        data = contour.reshape(-1, 2).astype(np.float32)
        mean, eigvec = cv.PCACompute(data, mean=None)
        vx, vy = eigvec[0]
        scale = 1000
        px1 = int(mean[0,0] + vx * scale)
        py1 = int(mean[0,1] + vy * scale)
        px2 = int(mean[0,0] - vx * scale)
        py2 = int(mean[0,1] - vy * scale)
        return (px1, py1), (px2, py2)

    # 识别原料区不同颜色的物料块
    def detect_color(self, color) -> bool:
        # 颜色矩阵
        detect_lower_green = np.array([40, 37, 70])
        detect_upper_green = np.array([81, 255, 255])
        detect_lower_blue = np.array([0, 90 ,174])
        detect_upper_blue = np.array([20, 255, 255])
        detect_lower_red = np.array([114, 67, 113])
        detect_upper_red = np.array([124, 255, 255])

        min_height = 160  # 色块的最小高度
        min_width = 80  # 色块的最小宽度
        assert color in color_list[1:], print("没写这个颜色")
        lower_color = None
        upper_color = None
        if color == "red":
            lower_color = detect_lower_red
            upper_color = detect_upper_red
        elif color == "green":
            lower_color = detect_lower_green
            upper_color = detect_upper_green
        elif color == "blue":
            lower_color = detect_lower_blue
            upper_color = detect_upper_blue
        else:
            print("!!没这个颜色")
        while True:
            _, frame = self.cap.read()
            if frame is None:
                if not self.cap_flag:
                    self.cap_flag = True
                    print("摄像头打开失败,尝试更换摄像头")
                    self.cap = cv.VideoCapture(3, cv.CAP_V4L2)
                    self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                elif self.cap_flag:
                    self.cap_flag = False
                    print("摄像头打开失败,尝试更换摄像头")
                    self.cap = cv.VideoCapture(2, cv.CAP_V4L2)
                    self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                for i in range(5):
                   _, frame = self.cap.read() 
                _, frame = self.cap.read()
            cutted_frame = frame[200:400, 300:500]
            hsv_frame = cv.cvtColor(cutted_frame, cv.COLOR_RGB2HSV)
            img = cv.inRange(hsv_frame, lower_color, upper_color)
            if self.cv_big_version == 4:
                cnts, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
            else:
                _, cnts, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

            if self.debug:
                cv.drawContours(cutted_frame, cnts, -1, (255, 0, 0), 2)
                cv.imshow("frame", frame)
                cv.imshow("cutted_frame", cutted_frame)
                cv.imshow("hsv_frame", hsv_frame)
                cv.imshow("after_hsv", img)
                k = cv.waitKey(1) & 0xFF
                if k == 27:
                    cv.destroyAllWindows()
                    break

            if len(cnts) > 0:
                s = []
                max_index = 0
                (cnts, boundingRects) = contours.sort_contours(cnts)
                for cnt in cnts:
                    s.append(cv.contourArea(cnt))
                    max_index = s.index(max(s))
                    (_, _, w, h) = boundingRects[max_index]
                if w > min_width and h > min_height:
                    print(color)
                    if not self.debug:
                        return True
                    else:
                        pass
                else:
                    pass

    def get_colored_circle_center_grub(self, color, min_r=100) -> Tuple[float, float]:
        detect_lower_green = np.array([40, 37, 70])
        detect_upper_green = np.array([81, 255, 255])
        detect_lower_blue = np.array([0, 90 ,174])
        detect_upper_blue = np.array([20, 255, 255])
        detect_lower_red = np.array([114, 67, 113])
        detect_upper_red = np.array([124, 255, 255])
        assert color in color_list[1:], print("没写这个颜色")
        if color == "red":
            lower_color = detect_lower_red
            upper_color = detect_upper_red
        elif color == "green":
            lower_color = detect_lower_green
            upper_color = detect_upper_green
        else:
            lower_color = detect_lower_blue
            upper_color = detect_upper_blue

        self.skip_some()
        count = count_ori = 8
        D_X = []
        D_Y = []
        while True:
            _, ori_img = self.cap.read()
            # 提取相应颜色区域
            hsv_frame = cv.cvtColor(ori_img, cv.COLOR_RGB2HSV)
            img = cv.inRange(hsv_frame, lower_color, upper_color)
            contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            max_index, max_area = (0, 0)

            # 寻找最大的轮廓
            for i, contour in enumerate(contours):
                if cv.contourArea(contour) > max_area:
                    max_index = i
                    max_area = cv.contourArea(contour)

            # 如果最大轮廓的面积够大，计算圆心
            if max_area > 3.14 * (min_r ** 2):
                (x,y),radius = cv.minEnclosingCircle(contours[max_index])
                center = (int(x), int(y))
                radius = int(radius)
                cv.circle(ori_img, center, radius, (0,255,0), 2)
                cv.circle(ori_img, center, 4, (0, 255, 255), 3)
                dx = (x - self.width / 2) / self.width * 2
                dy = (y - self.height / 2) / self.height * 2
                D_X.append(dx)
                D_Y.append(dy)
                count -= 1
            else:
                pass

            # debug模式下，显示图像
            if self.debug:
                # 在图像正中心绘制一个绿色的点
                center_x = int(self.width / 2)
                center_y = int(self.height / 2)
                cv.circle(ori_img, (center_x, center_y), 2, (0, 255, 0), -1)  # 绿色点，半径为5                
                cv.imshow('ori_img', ori_img)
                cv.imshow('img', img)
                k =cv.waitKey(1) & 0xFF
                if k == 27:
                    break
            # 记录连续多次的圆心，取平均
            if count < 1:
                DeltaX = sum(D_X) / len(D_X)
                DeltaY = sum(D_Y) / len(D_Y)
                D_X = []  # 重置累加值
                D_Y = []  # 重置累加值
                count = count_ori
                if not self.debug:
                    break
                else:
                    print(DeltaX, DeltaY)

        return DeltaX, DeltaY

    def get_colored_circle_center_second(self, min_r=100) -> Tuple[float, float]:

        lower_color = np.array([33,0,0])
        upper_color = np.array([95,200,255])
        self.skip_some()
        count = count_ori = 15
        D_X = []
        D_Y = []
        while True:
            _, ori_img = self.cap.read()
            # 提取相应颜色区域
            hsv_frame = cv.cvtColor(ori_img, cv.COLOR_RGB2HSV)
            img = cv.inRange(hsv_frame, lower_color, upper_color)
            contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            max_index, max_area = (0, 0)

            # 寻找最大的轮廓
            for i, contour in enumerate(contours):
                if cv.contourArea(contour) > max_area:
                    max_index = i
                    max_area = cv.contourArea(contour)

            # 如果最大轮廓的面积够大，计算圆心
            if max_area > 3.14 * (min_r ** 2):
                (x,y),radius = cv.minEnclosingCircle(contours[max_index])
                center = (int(x), int(y))
                radius = int(radius)
                cv.circle(ori_img, center, radius, (0,255,0), 2)
                cv.circle(ori_img, center, 4, (0, 255, 255), 3)
                dx = (x - self.width / 2) / self.width * 2
                dy = (y - self.height / 2) / self.height * 2
                D_X.append(dx)
                D_Y.append(dy)
                count -= 1
            else:
                pass

            # debug模式下，显示图像
            if self.debug:
                # 在图像正中心绘制一个绿色的点
                center_x = int(self.width / 2)
                center_y = int(self.height / 2)
                cv.circle(ori_img, (center_x, center_y), 2, (0, 255, 0), -1)  # 绿色点，半径为5                
                cv.imshow('ori_img', ori_img)
                cv.imshow('img', img)
                k =cv.waitKey(1) & 0xFF
                if k == 27:
                    break
            # 记录连续多次的圆心，取平均
            if count < 1:
                DeltaX = sum(D_X) / len(D_X)
                DeltaY = sum(D_Y) / len(D_Y)
                D_X = []  # 重置累加值
                D_Y = []  # 重置累加值
                count = count_ori
                if not self.debug:
                    break
                else:
                    print(DeltaX, DeltaY)

        return DeltaX, DeltaY

    def get_circle_center(self) -> Tuple[float, float]:
        count = count_ori = 5
        D_X = []
        D_Y = []
        while True:
            _, img = self.cap.read()
            if img is None:
                if not self.cap_flag:
                    self.cap_flag = True
                    print("摄像头打开失败,尝试更换摄像头")
                    self.cap = cv.VideoCapture(3, cv.CAP_V4L2)
                    self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                elif self.cap_flag:
                    self.cap_flag = False
                    print("摄像头打开失败,尝试更换摄像头")
                    self.cap = cv.VideoCapture(2, cv.CAP_V4L2)
                    self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                for i in range(count):
                    _, _ = self.cap.read()
                _, img = self.cap.read()
            img = cv.medianBlur(img, 5)
            cimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            circles = cv.HoughCircles(
                cimg,
                cv.HOUGH_GRADIENT,
                1,
                100,
                param1=100,
                param2=30,
                minRadius=100,
                maxRadius=200,
            )

            if type(circles) != np.ndarray:
                continue

            # 限制，当只有一个圆时
            if len(circles[0]) == 1:
                circles = np.uint16(np.around(circles))
                circle = circles[0, 0, :]
                cv.circle(img, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                cv.circle(img, (circle[0], circle[1]), 2, (0, 0, 255), 3)
                x, y = circle[0], circle[1]
                dx = (x - self.width / 2) / self.width * 2
                dy = (y - self.height / 2) / self.height * 2
                D_X.append(dx)
                D_Y.append(dy)
                count = count - 1

            if self.debug:
                # 在图像正中心绘制一个绿色的点
                center_x = int(self.width / 2)
                center_y = int(self.height / 2)
                cv.circle(img, (center_x, center_y), 2, (0, 255, 0), -1)  # 绿色点，半径为5    
                cv.imshow("frame", img)
                k = cv.waitKey(1) & 0xFF
                if k == 27:
                    break

            # 记录连续多次的圆心，取平均
            if count < 1:
                DeltaX = sum(D_X) / len(D_X)
                DeltaY = sum(D_Y) / len(D_Y)
                D_X = []  # 重置累加值
                D_Y = []  # 重置累加值
                count = count_ori
                if not self.debug:
                    break
                else:
                    print(DeltaX, DeltaY)

        return DeltaX, DeltaY

    def get_colored_circle_center(self, color, min_r=100) -> Tuple[float, float]:

            assert color in color_list[1:], print("没写这个颜色")
            if color == "red":
                lower_color = self.lower_red
                upper_color = self.upper_red
            elif color == "green":
                lower_color = self.lower_green
                upper_color = self.upper_green
            else:
                lower_color = self.lower_blue
                upper_color = self.upper_blue

            self.skip_some()
            count = count_ori = 15
            D_X = []
            D_Y = []
            while True:
                _, ori_img = self.cap.read()
                if ori_img is None:
                    if not self.cap_flag:
                        self.cap_flag = True
                        print("摄像头打开失败,尝试更换摄像头")
                        self.cap = cv.VideoCapture(3, cv.CAP_V4L2)
                        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                    elif self.cap_flag:
                        self.cap_flag = False
                        print("摄像头打开失败,尝试更换摄像头")
                        self.cap = cv.VideoCapture(2, cv.CAP_V4L2)
                        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                    for i in range(count):
                        _, _ = self.cap.read()
                    _, ori_img = self.cap.read()
                # 提取相应颜色区域
                hsv_frame = cv.cvtColor(ori_img, cv.COLOR_RGB2HSV)
                img = cv.inRange(hsv_frame, lower_color, upper_color)
                contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                max_index, max_area = (0, 0)

                # 寻找最大的轮廓
                for i, contour in enumerate(contours):
                    if cv.contourArea(contour) > max_area:
                        max_index = i
                        max_area = cv.contourArea(contour)

                # 如果最大轮廓的面积够大，计算圆心
                if max_area > 3.14 * (min_r ** 2):
                    (x,y),radius = cv.minEnclosingCircle(contours[max_index])
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv.circle(ori_img, center, radius, (0,255,0), 2)
                    cv.circle(ori_img, center, 4, (0, 255, 255), 3)
                    dx = (x - self.width / 2) / self.width * 2
                    dy = (y - self.height / 2) / self.height * 2
                    D_X.append(dx)
                    D_Y.append(dy)
                    count -= 1
                else:
                    pass

                # debug模式下，显示图像
                if self.debug:
                    # 在图像正中心绘制一个绿色的点
                    center_x = int(self.width / 2)
                    center_y = int(self.height / 2)
                    cv.circle(ori_img, (center_x, center_y), 2, (0, 255, 0), -1)  # 绿色点，半径为5                
                    cv.imshow('ori_img', ori_img)
                    cv.imshow('img', img)
                    k =cv.waitKey(1) & 0xFF
                    if k == 27:
                        break
                # 记录连续多次的圆心，取平均
                if count < 1:
                    DeltaX = sum(D_X) / len(D_X)
                    DeltaY = sum(D_Y) / len(D_Y)
                    D_X = []  # 重置累加值
                    D_Y = []  # 重置累加值
                    count = count_ori
                    if not self.debug:
                        break
                    else:
                        print(DeltaX, DeltaY)

            return DeltaX, DeltaY


    def __del__(self):
        print('摄像头检测类销毁')
        self.cap.release()
        del self.cap


if __name__ == "__main__":
    detect = Detection(2)
    detect.set_debug(True)  # 调试模式，会把检测结果显示出来，设置为False的话就不会有图像显示

    # 所有的识别都是通过按下ESC退出，然后进入下一阶段

    # 颜色识别
    for color in ["blue", "green", "red"]:
      detect.detect_color(color)

    # for color in ["blue", "green", "red"]:
    #    detect.get_colored_circle_center(color)

    # # 二维码识别
    # info = detect.get_qr_info(data_len=7)  # 超时时间，20s内没有读取到就退出
    # print(info)

    # # 圆心检测
    # dx, dy = detect.get_circle_center()
    # print("x方向误差{0}, y方向误差{1}".format(dx, dy))

    # #直线检测
    # detect.get_line_info()
