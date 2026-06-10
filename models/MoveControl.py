import serial
from models.config import Mode, HeightMode
import time


# 输入一个十进制的[-128, 127]范围内的整数，获取它补码的十进制
def get_int8(num):
    assert num <= 127 and num >= -128, print("int8类型范围为[-128, 127]")
    if num >= 0:
        return int(num)
    else:
        return int(255 + num + 1)


# 运动控制类，包含小车的距离控制，以及旋转控制
class MoveControl(object):
    def __init__(self, port, baudrate) -> None:
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(port, baudrate)
        print("与stm32串口初始化成功")
        self.serial.flush()

        # 初始化数据列表 [头帧，模式，X方向，Y方向，角度，二维码1，二维码2，尾帧]
        self.send_buffer = [0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0xFE]

    # 生成串口数据、发送数据并等待回传结果
    def __send_serial_msg(self, mode, x_dis=None, y_dis=None, angel=None, qr_info=None):
        if mode == Mode.x_distance:
            assert x_dis != None, print("未指定x方向移动距离")
            self.send_buffer[1] = mode.value
            self.send_buffer[2] = get_int8(x_dis)

        elif mode == Mode.y_distance:
            assert y_dis != None, print("未指定y方向移动距离")
            self.send_buffer[1] = mode.value
            self.send_buffer[3] = get_int8(y_dis)

        elif mode == Mode.rotate:
            assert angel != None, print("未设置角度")
            self.send_buffer[1] = mode.value
            if abs(angel) == 90:
                self.send_buffer[4] = get_int8(angel)
            elif angel <= 0:
                self.send_buffer[4] = get_int8(-1)
            elif angel >= 0:
                self.send_buffer[4] = get_int8(1)
        

        elif mode == Mode.qrcode:
            assert qr_info != None, print("未输入二维码信息")
            self.send_buffer[1] = mode.value

            task1, task2 = int(qr_info[:3]), int(qr_info[4:])
            self.send_buffer[5] = task1 >> 8
            self.send_buffer[6] = task1&0xff
            self.send_buffer[7] = task2 >> 8
            self.send_buffer[8] = task2&0xff

        elif mode == Mode.x_dis_mm:
            assert x_dis != None, print("未指定x方向移动距离，单位:毫米")
            self.send_buffer[1] = mode.value
            self.send_buffer[2] = get_int8(x_dis)

        elif mode == Mode.y_dim_mm:
            assert y_dis != None, print("未指定x方向移动距离，单位:毫米")
            self.send_buffer[1] = mode.value
            self.send_buffer[3] = get_int8(y_dis)

        # 从原料区抓取物料块
        elif mode == Mode.grub_1:
            self.send_buffer[1] = mode.value

        # 从地面抓取物料块
        elif mode == Mode.grub_2:
            self.send_buffer[1] = mode.value

        # 将物料块放到地面上
        elif mode == Mode.down_1:
            self.send_buffer[1] = mode.value

        # 在精加工区，将物料块放到另一个物料块
        elif mode == Mode.down_2:
            self.send_buffer[1] = mode.value
        
        # 调整升降台的位置
        elif mode == Mode.adjust_height:
            self.send_buffer[1] = mode.value
            self.send_buffer[2] = x_dis.value

        elif mode == Mode.adjust_height_manual:
            self.send_buffer[1] = mode.value
            self.send_buffer[2] = x_dis.value
            raise NotImplementedError('这个模式还没有实现')
        
        elif mode == Mode.mix_mode:
            self.send_buffer[1] = mode.value
            self.send_buffer[2] = y_dis.value
            self.send_buffer[3] = get_int8(x_dis)

        elif mode == Mode.final:
            self.send_buffer[1] = mode.value

        else:
            raise ValueError("下位机模式选择错误，现有的模式有", [mode.name for mode in Mode])

        self.serial.write(self.send_buffer)

        self.__wait_for_movement_done()
        time.sleep(0.05)

    # 在下发动作后，比如前进，旋转等，下位机执行结束后会回传任务结束指令[0xFF, 0x01, 0xFE]
    def __wait_for_movement_done(self):
        while True:
            header = ord(self.serial.read())
            if header == self.send_buffer[0]:
                if ord(self.serial.read()) == 0x01:
                    return True
                else:
                    pass
            else:
                pass

    # 程序启动后，需要等待下位机启动指令  [0xFF, 0x10, 0xFE]
    def wait_for_start_cmd(self):
        while True:
            data = self.serial()
            if data:
                rec_str = data.decode()
                if ord(rec_str[1]) == 10 and len(rec_str) == 3:
                    print("下位机已连接")
                    break

    # 向下位机发发送二维码信息
    def send_qr_info(self, qr_info):
        assert qr_info, print("二维码信息为空")
        self.__send_serial_msg(mode=Mode.qrcode, qr_info=qr_info)

    # 距离控制，支持x和y同时输入，先运行x再运行y
    def set_distance(self, x=0, y=0):
        x = 1e-9 if x == 0 else x
        y = 1e-9 if y == 0 else y
        max_one_time_dis = 1.25
        if abs(x) > max_one_time_dis or abs(y) > max_one_time_dis:
            Warning("单次距离设置最远为{0}，需多次调用".format(max_one_time_dis))

        times_x, times_y = int(abs(x) / max_one_time_dis), int(
            abs(y) // max_one_time_dis
        )
        rest_x, rest_y = (
            abs(x) - times_x * max_one_time_dis,
            abs(y) - times_y * max_one_time_dis,
        )
        factor_x, factor_y = abs(x) // x, abs(y) // y

        for _ in range(times_x):
            dis = 125 * factor_x
            self.__send_serial_msg(mode=Mode.x_distance, x_dis=dis)
        if rest_x > 0.001:
            dis = factor_x * rest_x / max_one_time_dis * 125
            self.__send_serial_msg(mode=Mode.x_distance, x_dis=dis)
        else:
            pass

        for _ in range(times_y):
            dis = 125 * factor_y
            self.__send_serial_msg(mode=Mode.y_distance, y_dis=dis)
        if rest_y > 0.001:
            dis = factor_y * rest_y / max_one_time_dis * 125
            self.__send_serial_msg(mode=Mode.y_distance, y_dis=dis)
        else:
            pass

    # 较为精细的距离控制
    def move_in_mm(self, x=0, y=0):
        where_x = x
        where_y = y
        x = 128 if x > 128 else int(x)
        x = -127 if x < -127 else int(x)
        y = 128 if y > 128 else int(y)
        y = -127 if y < -127 else int(y)
        if x != 0:
            self.__send_serial_msg(mode=Mode.x_dis_mm, x_dis=x)
        if x == 0:
            if where_x > 0:
                self.__send_serial_msg(mode=Mode.x_dis_mm, x_dis=1)
            else :
                self.__send_serial_msg(mode=Mode.x_dis_mm, x_dis=-1)

        if y != 0:
            self.__send_serial_msg(mode=Mode.y_dim_mm, y_dis=y)
        if y == 0:
            if where_y > 0:
                self.__send_serial_msg(mode=Mode.y_dim_mm, y_dis=1)
            else :
                self.__send_serial_msg(mode=Mode.y_dim_mm, y_dis=-1)


    # 旋转，角度定义为[-127, 128]之间
    def rotate(self, angle):
        assert angle <= 128 and angle >= -127, print("角度应当在[-127, 128]之间")
        self.__send_serial_msg(mode=Mode.rotate, angel=angle)

    def test(self, num):
        self.send_buffer[1] = 3
        self.send_buffer[4] = num

        self.serial.write(self.send_buffer)

    # 从原料区抓取物料块
    def grubBlockFromRawArea(self):
        self.__send_serial_msg(mode=Mode.grub_1)

    # 从地面抓取物料块
    def grubBlockFromGround(self):
        self.__send_serial_msg(mode=Mode.grub_2)

    # 将物料块放到圆环上
    def putBlockToCircle(self):
        self.__send_serial_msg(mode=Mode.down_1)

    # 精加工区，将物料块放到另一个物料块上
    def putBlockToAnotherBlock(self):
        self.__send_serial_msg(mode=Mode.down_2)

    #自定义
    def final(self):
        self.__send_serial_msg(mode=Mode.final)
    


    # 调整升降台到合适的位置    1.读取二维码  2.识别原料区  3.识别圆环   4.识别物料块圆心 
    def adjustHeight(self, target_height):
        heightConfigs = [mode for mode in HeightMode]
        if target_height not in heightConfigs:
            raise ValueError("小车高度模式中没有这个模式{0}".format(target_height))
        self.__send_serial_msg(mode=Mode.adjust_height, x_dis=target_height)

    """
        混合模式: 支持在小车的移动过程中移动升降台，减少时间
        输入参数：
            height_mode: 期望的模式，支持单纯的升降台移动，以及夹取后的直接离开
            move_dis: 小车移动距离（仅限X方向移动）
        返回参数：
            无；当接收到下位机信号后，函数结束
    """
    def move_while_adjusting_height(self, mix_mode, move_dis: float):
        if abs(move_dis) > 1.25:
            raise ValueError('太远了，这个模式最远支持1.25m')
        dis = 100 * move_dis
        self.__send_serial_msg(mode=Mode.mix_mode, x_dis=dis, y_dis=mix_mode)

    def __del__(self):
        self.serial.close()
        print("程序结束，释放串口")
