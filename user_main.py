from machine import *
from display import *
from smartcar import *
from seekfree import *
import gc
import time
import math

# wifi开关
wifi_en = True 

if wifi_en:
    # WiFi调参初始化
    try:
        wifi = WIFI_SPI("OnePlus 13", "1234567890xia", WIFI_SPI.TCP_CONNECT, "192.168.31.9", "8086")
        wifi.send_str("WiFi parameter tuning ready.\r\n")
        time.sleep_ms(500)
        wifi_enabled = True
        print("WiFi调参模块初始化成功")
    except:
        wifi_enabled = False
else:
    wifi_enabled = False

# 全局变量
PI = 3.14
ticker_flag = False
Filter_data = [0, 0, 0]
last_yaw = 0

# CCD相关全局变量
ccd_ticker_flag = False
ccd_ticker_count = 0
line_deviation = 0  # 线路偏差
line_control_output = 0  # 线路控制输出

# CCD算法参数 - 移植自C语言示例
CCD1_SET_WIDTH = 38  # 近端CCD设定宽度
CCD2_SET_WIDTH = 20  # 远端CCD设定宽度

# 阈值参数 - 需要调试
THRESHOLD_MULTIPLE_1 = 19  # CCD1阈值倍数 (%)
THRESHOLD_MULTIPLE_2 = 19  # CCD2阈值倍数 (%)
THRESHOLD_1 = 42  # CCD1二值化阈值 (%)
THRESHOLD_2 = 42  # CCD2二值化阈值 (%)

# 环岛状态定义
NO_RING = 0
FIND_RING = 1
READY_IN_RING = 2
IN_RING = 3
READY_OUT_RING = 4
OUT_RING = 5
READY_NO_RING = 6

# 环岛相关全局变量
ring_state = NO_RING
ring_left = False
ring_right = False
ring_speed = 0
pid_ring_flag = 0
encoder_ring = 0
angle_ring = 0
angle_gz = 0  # 陀螺仪累积角度

# 环岛参数 - 可通过WiFi调参
READY_IN_RING_ENCODER = 50  # 准备进入环岛的编码器距离
IN_RING_ENCODER = 30  # 进入环岛的编码器距离
READY_OUT_RING_ANGLE = 180  # 准备出环岛的角度
OUT_RING_ANGLE = 270  # 出环岛的角度
NO_RING_ENCODER = 80  # 退出环岛状态的编码器距离

# 环岛检测阈值
RING_QULU_THRESHOLD = 20  # 环岛曲率检测阈值
RING_QULU_EXIT_THRESHOLD = 30  # 环岛退出曲率阈值

# 移除十字路口和坡道检测功能，只保留环岛检测

# 编码器积分值（用于距离计算）
encoder_integral = 0

# CCD信息类
class CCDInformation:
    def __init__(self):
        self.max_val = 0
        self.min_val = 0
        self.threshold = 0
        self.aver = 0
        self.bin_thrd = 0

# 赛道信息类
class TrackInformation:
    def __init__(self):
        # CCD1(近端)原图像
        self.left_sideline1 = 0
        self.right_sideline1 = 0
        self.middle_sideline1 = 63.0
        self.middle_sideline1_last = 63.0
        self.width1 = 0
        
        # CCD2(远端)原图像
        self.left_sideline2 = 0
        self.right_sideline2 = 0
        self.middle_sideline2 = 63.0
        self.middle_sideline2_last = 63.0
        self.width2 = 0
        
        # 曲率计算
        self.left_qulu = 0.0  # 左边曲率
        self.right_qulu = 0.0  # 右边曲率

# 全局CCD对象
CCD1 = CCDInformation()  # 近端CCD
CCD2 = CCDInformation()  # 远端CCD
Trk = TrackInformation()  # 赛道信息

# 边界检测标志
CCD1_left_flag = False
CCD1_right_flag = False
CCD2_left_flag = False
CCD2_right_flag = False

# 黑白场景标志
black_write_1 = False
black_write_2 = False

# 直线弯道判断标志
straight = False
curve = False

# 硬件初始化
end_switch = Pin('D20', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
end_state = end_switch.value()

# 蜂鸣器初始化
beep = Pin('D24', Pin.OUT, pull=Pin.PULL_UP_47K, value=False)

def beep_on():
    """蜂鸣器响"""
    beep.high()

def beep_off():
    """蜂鸣器停"""
    beep.low()

def beep_short():
    """短响一声"""
    beep_on()
    time.sleep_ms(100)
    beep_off()

def beep_long():
    """长响一声 - 延长到1秒，便于识别进入环岛状态"""
    beep_on()
    time.sleep_ms(1000)  # 延长响声时间，便于调试时识别
    beep_off()

motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=False)

encoder_l = encoder("C0", "C1", True)
encoder_r = encoder("C2", "C3")

imu = IMU660RX()
imu_data = imu.get()

# CCD初始化
ccd = TSL1401(10)
ccd.set_resolution(TSL1401.RES_12BIT)
time.sleep_ms(500)  # CCD初始化延时

# IPS200屏幕初始化
# 定义片选引脚
cs = Pin('B29', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
# 定义控制引脚
rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc = Pin('B5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(2)
# 清屏
lcd.clear(0x0000)

# PID参数 - 进一步增强响应强度
angle_kp = -1853
angle_ki = 0
angle_kd =55

roll_angle_Kp = 0.094
roll_angle_Ki = 0
roll_angle_Kd = 0.0855  # 进一步增强角度环响应

speed_Kp = 0.098
speed_Ki = 0
speed_Kd = 3.98

# 线路跟踪PD控制器参数
line_kp = 5 # 比例控制，快速响应
line_kd = 180  # 微分控制，提高稳定性

# 控制变量
angle_1 = speed_1 = motor1 = motor2 = 0
med_roll_angle = 34.1  # 调整平衡角度
TARGET_SPEED = 20  # 设置小的前进速度进行测试
ticker_count = 0

# WiFi调参数据存储 - 前四个通道改为CCD阈值参数
wifi_data = [THRESHOLD_MULTIPLE_1, THRESHOLD_MULTIPLE_2, THRESHOLD_1, THRESHOLD_2, speed_Kp, line_kd, speed_Kd, line_kp]

# 环岛调参数据存储 (可扩展到更多通道)
ring_wifi_data = [RING_QULU_THRESHOLD, READY_IN_RING_ENCODER, IN_RING_ENCODER, READY_OUT_RING_ANGLE]

def update_wifi_parameters():
    """更新WiFi调参数据"""
    global THRESHOLD_MULTIPLE_1, THRESHOLD_MULTIPLE_2, THRESHOLD_1, THRESHOLD_2
    global speed_Kp, line_kd, speed_Kd, line_kp
    global pid_speed, pid_line, wifi_data, motor1, motor2
    
    if not wifi_enabled:
        return
    
    try:
        # 数据解析
        data_flag = wifi.data_analysis()
        
        # 检查各通道是否有数据更新
        for i in range(8):
            if data_flag[i]:
                wifi_data[i] = wifi.get_data(i)
        
        # 更新CCD阈值参数 (前4个通道)
        THRESHOLD_MULTIPLE_1 = int(wifi_data[0])  # 转换为整数
        THRESHOLD_MULTIPLE_2 = int(wifi_data[1])  # 转换为整数
        THRESHOLD_1 = int(wifi_data[2])           # 转换为整数
        THRESHOLD_2 = int(wifi_data[3])           # 转换为整数
        
        # 更新控制参数 (后4个通道)
        speed_Kp = wifi_data[4]
        line_kd = wifi_data[5]
        speed_Kd = wifi_data[6]
        line_kp = wifi_data[7]
        
        # 更新PID控制器参数
        pid_speed.kp = speed_Kp
        pid_speed.kd = speed_Kd
        
        # 更新巡线PD控制器参数
        pid_line.kp = line_kp
        pid_line.kd = line_kd
        
        # 发送示波器数据 - 显示CCD阈值和控制输出
        wifi.send_oscilloscope(
            Trk.left_qulu, Trk.right_qulu, line_deviation, 
            THRESHOLD_MULTIPLE_1, THRESHOLD_MULTIPLE_2, THRESHOLD_1, THRESHOLD_2, line_control_output)
    
    except:
        pass

# 卡尔曼滤波参数
class KalmanFilter:
    def __init__(self, Q=0.70, R=200):
        self.P = 0.02
        self.Q = Q
        self.R = R
        self.output = 0.0
    
    def update(self, input_val):
        self.P += self.Q
        G = self.P / (self.P + self.R)
        self.output += G * (input_val - self.output)
        self.P = (1 - G) * self.P
        return self.output

# PID控制器类
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.err_sum = self.err_last = 0
    
    def update(self, setpoint, current):
        err = setpoint - current
        self.err_sum += err
        err_diff = err - self.err_last
        output = self.kp * err + self.ki * self.err_sum + self.kd * err_diff
        self.err_last = err
        return output

# PD控制器类（用于线路跟踪）
class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.err_last = 0
    
    def update(self, setpoint, current):
        err = setpoint - current
        err_diff = err - self.err_last
        output = self.kp * err + self.kd * err_diff
        self.err_last = err
        return output

# IMU数据类
class IMUData:
    def __init__(self):
        self.acc_x = self.acc_y = self.acc_z = 0.0
        self.gyro_x = self.gyro_y = self.gyro_z = 0.0
        self.Pitch = self.Roll = self.Yaw = 0.0
        self.Total_Yaw = 0.0

# 四元数类
class Quaternion:
    def __init__(self):
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0

# 初始化对象
kalman_l = KalmanFilter()
kalman_r = KalmanFilter()
pid_angle_speed = PIDController(angle_kp, angle_ki, angle_kd)
pid_angle = PIDController(roll_angle_Kp, roll_angle_Ki, roll_angle_Kd)
pid_speed = PIDController(speed_Kp, speed_Ki, speed_Kd)
pid_line = PDController(line_kp, line_kd)  # 线路跟踪PD控制器
imu_data_obj = IMUData()
quaternion = Quaternion()

# 积分误差
I_ex = I_ey = I_ez = 0.0
delta_T = 0.001
param_Kp, param_Ki = 18.0, 0.008  # 适当降低姿态解算增益

def limit(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def limit_angle(value):
    return max(-1, min(value, 1))

def inv_sqrt(x):
    return 1.0 / math.sqrt(x) if x > 0 else 1.0

def imu_process():
    global imu_data, imu_data_obj, last_yaw
    
    # 数据有效性检查
    for i in range(3, 6):
        if abs(imu_data[i]) < 30 or abs(imu_data[i]) > 30000:
            imu_data[i] = 0
    
    # 角速度转换
    imu_data_obj.gyro_x = (imu_data[3] - Filter_data[0]) * PI / 180 / 16.4
    imu_data_obj.gyro_y = (imu_data[4] - Filter_data[1]) * PI / 180 / 16.4
    imu_data_obj.gyro_z = (imu_data[5] - Filter_data[2]) * PI / 180 / 14.4
    
    # 加速度滤波 - 增加平滑性减少摆动
    alpha = 0.35  # 进一步降低滤波系数，增加平滑性
    imu_data_obj.acc_x = (imu_data[0] * alpha / 4096) + (imu_data_obj.acc_x * (1 - alpha))
    imu_data_obj.acc_y = (imu_data[1] * alpha / 4096) + (imu_data_obj.acc_y * (1 - alpha))
    imu_data_obj.acc_z = (imu_data[2] * alpha / 4096) + (imu_data_obj.acc_z * (1 - alpha))
    
    # 姿态解算
    ahrs_update(imu_data_obj.gyro_x, imu_data_obj.gyro_y, imu_data_obj.gyro_z,
                imu_data_obj.acc_x, imu_data_obj.acc_y, imu_data_obj.acc_z)

def ahrs_update(gx, gy, gz, ax, ay, az):
    global I_ex, I_ey, I_ez, last_yaw, quaternion, imu_data_obj
    
    halfT = 0.5 * delta_T
    
    # 四元数预计算
    q0q0 = quaternion.q0 * quaternion.q0
    q0q1 = quaternion.q0 * quaternion.q1
    q0q2 = quaternion.q0 * quaternion.q2
    q1q1 = quaternion.q1 * quaternion.q1
    q1q3 = quaternion.q1 * quaternion.q3
    q2q2 = quaternion.q2 * quaternion.q2
    q2q3 = quaternion.q2 * quaternion.q3
    q3q3 = quaternion.q3 * quaternion.q3
    
    # 归一化加速度
    norm = inv_sqrt(ax*ax + ay*ay + az*az)
    ax *= norm
    ay *= norm
    az *= norm
    
    # 重力向量
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3
    
    # 误差计算
    ex = ay * vz - az * vy
    ey = az * vx - ax * vz
    ez = ax * vy - ay * vx
    
    # PI修正
    I_ex += delta_T * ex
    I_ey += delta_T * ey
    I_ez += delta_T * ez
    
    gx += param_Kp * ex + param_Ki * I_ex
    gy += param_Kp * ey + param_Ki * I_ey
    gz += param_Kp * ez + param_Ki * I_ez
    
    # 四元数更新
    q0, q1, q2, q3 = quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3
    
    quaternion.q0 += (-q1*gx - q2*gy - q3*gz) * halfT
    quaternion.q1 += (q0*gx + q2*gz - q3*gy) * halfT
    quaternion.q2 += (q0*gy - q1*gz + q3*gx) * halfT
    quaternion.q3 += (q0*gz + q1*gy - q2*gx) * halfT
    
    # 归一化四元数
    norm = inv_sqrt(quaternion.q0**2 + quaternion.q1**2 + quaternion.q2**2 + quaternion.q3**2)
    quaternion.q0 *= norm
    quaternion.q1 *= norm
    quaternion.q2 *= norm
    quaternion.q3 *= norm
    
    # 计算欧拉角
    value1 = limit_angle(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2)
    imu_data_obj.Roll = math.asin(value1) * 180 / PI
    imu_data_obj.Pitch = -math.atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1,
                                   -2 * quaternion.q1**2 - 2 * quaternion.q2**2 + 1) * 180 / PI
    imu_data_obj.Yaw = math.atan2(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3,
                                 -2 * quaternion.q2**2 - 2 * quaternion.q3**2 + 1) * 180 / PI
    
    # 累积偏航角
    error_yaw = imu_data_obj.Yaw - last_yaw
    if error_yaw > 180:
        error_yaw -= 360
    elif error_yaw < -180:
        error_yaw += 360
    
    imu_data_obj.Total_Yaw += error_yaw
    last_yaw = imu_data_obj.Yaw
    
    # 保持在0-360度范围
    if imu_data_obj.Total_Yaw > 360:
        imu_data_obj.Total_Yaw -= 360
    elif imu_data_obj.Total_Yaw < 0:
        imu_data_obj.Total_Yaw += 360
    
    # 更新环岛用的陀螺仪累积角度
    global angle_gz
    angle_gz = imu_data_obj.Total_Yaw

def imu_init():
    global Filter_data, imu_data
    Filter_data = [0, 0, 0]
    
    for _ in range(1000):
        imu_data = imu.get()
        for i in range(3):
            Filter_data[i] += imu_data[i + 3]
        time.sleep_ms(1)
    
    for i in range(3):
        Filter_data[i] /= 1000

def control_loop(timer):
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2, imu_data, line_control_output
    
    ticker_flag = True
    ticker_count = (ticker_count + 1) % 10
    
    # 1ms: 角速度控制
    imu_data = imu.get()
    imu_process()
    
    motor1 = pid_angle_speed.update(angle_1, -imu_data_obj.gyro_x)
    motor2 = motor1
    
    # CCD巡线控制
    motor1 -= line_control_output  # 左电机增加转向控制
    motor2 += line_control_output  # 右电机减少转向控制
    
    motor1 = limit(motor1, -6666, 6666)  # 增加电机输出限制，提高响应强度
    motor2 = limit(motor2, -6666, 6666)  # 增加电机输出限制，提高响应强度
    
    motor_l.duty(motor1)
    motor_r.duty(motor2)
    
    # 5ms: 角度控制
    if ticker_count % 5 == 0:
        angle_1 = pid_angle.update(med_roll_angle - speed_1, imu_data_obj.Pitch)
    
    # 10ms: 速度控制
    if ticker_count == 0:
        avg_speed = (kalman_l.output + kalman_r.output) / 2
        speed_1 = pid_speed.update(TARGET_SPEED, avg_speed)
        speed_1 = limit(speed_1, -10, 10)  # 限制角度偏移

def encoder_update(timer):
    global encoder_integral
    kalman_l.update(encoder_l.get())
    kalman_r.update(encoder_r.get())
    
    # 更新编码器积分值（用于距离计算）
    avg_encoder = (abs(encoder_l.get()) + abs(encoder_r.get())) / 2
    encoder_integral += avg_encoder

def ccd_image_init():
    """CCD图像初始化"""
    global Trk, CCD1, CCD2
    Trk.middle_sideline1 = 63.0
    Trk.middle_sideline2 = 63.0
    CCD1.bin_thrd = 0
    CCD2.bin_thrd = 0

def ccd1_get(ccd_data):
    """CCD1数据获取和处理 - 近端CCD"""
    global CCD1, THRESHOLD_MULTIPLE_1, THRESHOLD_1
    
    if not ccd_data or len(ccd_data) < 128:
        return
    
    # 计算最大最小值 (范围5-122，对应C代码)
    CCD1.max_val = 0
    CCD1.min_val = ccd_data[4] if len(ccd_data) > 4 else 0
    CCD1.aver = 0
    
    # 统计最大最小值
    for i in range(5, min(123, len(ccd_data))):
        if ccd_data[i] > CCD1.max_val:
            CCD1.max_val = ccd_data[i]
        if ccd_data[i] < CCD1.min_val:
            CCD1.min_val = ccd_data[i]
    
    # 计算中心区域平均值 (48-78)
    count = 0
    total = 0
    for i in range(48, min(78, len(ccd_data))):
        total += ccd_data[i]
        count += 1
    
    if count > 0:
        CCD1.aver = total // count
    
    # 二值化阈值计算
    if CCD1.bin_thrd == 0:
        CCD1.bin_thrd = 1
    elif CCD1.bin_thrd == 1:
        CCD1.bin_thrd = (CCD1.aver * THRESHOLD_1) // 100
    
    # 动态阈值计算
    if CCD1.max_val + CCD1.min_val > 0:
        CCD1.threshold = ((CCD1.max_val - CCD1.min_val) * 100 * THRESHOLD_MULTIPLE_1) // ((CCD1.max_val + CCD1.min_val) * 100)

def ccd2_get(ccd_data):
    """CCD2数据获取和处理 - 远端CCD"""
    global CCD2, THRESHOLD_MULTIPLE_2, THRESHOLD_2
    
    if not ccd_data or len(ccd_data) < 128:
        return
    
    # 计算最大最小值 (范围10-117，对应C代码)
    CCD2.max_val = 0
    CCD2.min_val = ccd_data[4] if len(ccd_data) > 4 else 0
    CCD2.aver = 0
    
    # 统计最大最小值
    for i in range(10, min(118, len(ccd_data))):
        if ccd_data[i] > CCD2.max_val:
            CCD2.max_val = ccd_data[i]
        if ccd_data[i] < CCD2.min_val:
            CCD2.min_val = ccd_data[i]
    
    # 计算中心区域平均值 (53-73)
    count = 0
    total = 0
    for i in range(53, min(73, len(ccd_data))):
        total += ccd_data[i]
        count += 1
    
    if count > 0:
        CCD2.aver = total // count
    
    # 二值化阈值计算
    if CCD2.bin_thrd == 0:
        CCD2.bin_thrd = 1
    elif CCD2.bin_thrd == 1:
        CCD2.bin_thrd = (CCD2.aver * THRESHOLD_2) // 100
    
    # 动态阈值计算
    if CCD2.max_val + CCD2.min_val > 0:
        CCD2.threshold = ((CCD2.max_val - CCD2.min_val) * 100 * THRESHOLD_MULTIPLE_2) // ((CCD2.max_val + CCD2.min_val) * 100)

def left_right_sideline(ccd_data1, ccd_data2):
    """左右边界检测 - 移植自C语言核心算法"""
    global Trk, CCD1, CCD2, CCD1_left_flag, CCD1_right_flag, CCD2_left_flag, CCD2_right_flag
    global black_write_1, black_write_2
    
    if not ccd_data1 or not ccd_data2:
        return
    
    # 保存上次中线位置
    Trk.middle_sideline1_last = Trk.middle_sideline1
    Trk.middle_sideline2_last = Trk.middle_sideline2
    
    # CCD1边界检测 (近端)
    start_pos = int(Trk.middle_sideline1_last)
    
    # 左边界检测
    CCD1_left_flag = False
    for i in range(start_pos, 4, -1):
        if i >= 5 and i < len(ccd_data1):
            # 梯度检测算法
            if (ccd_data1[i] + ccd_data1[i-5]) > 0:
                gradient = abs(ccd_data1[i] - ccd_data1[i-5]) * 100 // (ccd_data1[i] + ccd_data1[i-5])
                if gradient > CCD1.threshold and ccd_data1[i] > ccd_data1[i-5]:
                    Trk.left_sideline1 = i
                    CCD1_left_flag = True
                    break
    
    if not CCD1_left_flag:
        Trk.left_sideline1 = 5
    
    # 右边界检测
    CCD1_right_flag = False
    for i in range(start_pos, min(122, len(ccd_data1))):
        if i + 5 < len(ccd_data1):
            # 梯度检测算法
            if (ccd_data1[i] + ccd_data1[i+5]) > 0:
                gradient = abs(ccd_data1[i] - ccd_data1[i+5]) * 100 // (ccd_data1[i] + ccd_data1[i+5])
                if gradient > CCD1.threshold and ccd_data1[i] > ccd_data1[i+5]:
                    Trk.right_sideline1 = i
                    CCD1_right_flag = True
                    break
    
    if not CCD1_right_flag:
        Trk.right_sideline1 = 122
    
    # 黑白场景判断
    black_write_1 = CCD1.aver < CCD1.bin_thrd
    
    # 单边丢失补偿
    if CCD1_left_flag and not CCD1_right_flag:
        for i in range(Trk.left_sideline1, min(122, len(ccd_data1))):
            if i + 5 < len(ccd_data1) and (ccd_data1[i] + ccd_data1[i+5]) > 0:
                gradient = abs(ccd_data1[i] - ccd_data1[i+5]) * 100 // (ccd_data1[i] + ccd_data1[i+5])
                if gradient > CCD1.threshold and ccd_data1[i] > ccd_data1[i+5]:
                    Trk.right_sideline1 = i
                    CCD1_right_flag = True
                    break
    
    elif not CCD1_left_flag and CCD1_right_flag:
        for i in range(Trk.right_sideline1, 4, -1):
            if i >= 5 and (ccd_data1[i] + ccd_data1[i-5]) > 0:
                gradient = abs(ccd_data1[i] - ccd_data1[i-5]) * 100 // (ccd_data1[i] + ccd_data1[i-5])
                if gradient > CCD1.threshold and ccd_data1[i] > ccd_data1[i-5]:
                    Trk.left_sideline1 = i
                    CCD1_left_flag = True
                    break
    
    # CCD2边界检测 (远端) - 类似逻辑
    start_pos2 = int(Trk.middle_sideline2_last)
    
    # 左边界检测
    CCD2_left_flag = False
    for i in range(start_pos2, 9, -1):
        if i >= 10 and i < len(ccd_data2):
            if (ccd_data2[i] + ccd_data2[i-5]) > 0:
                gradient = abs(ccd_data2[i] - ccd_data2[i-5]) * 100 // (ccd_data2[i] + ccd_data2[i-5])
                if gradient > CCD2.threshold and ccd_data2[i] > ccd_data2[i-5]:
                    Trk.left_sideline2 = i
                    CCD2_left_flag = True
                    break
    
    if not CCD2_left_flag:
        Trk.left_sideline2 = 10
    
    # 右边界检测
    CCD2_right_flag = False
    for i in range(start_pos2, min(117, len(ccd_data2))):
        if i + 5 < len(ccd_data2):
            if (ccd_data2[i] + ccd_data2[i+5]) > 0:
                gradient = abs(ccd_data2[i] - ccd_data2[i+5]) * 100 // (ccd_data2[i] + ccd_data2[i+5])
                if gradient > CCD2.threshold and ccd_data2[i] > ccd_data2[i+5]:
                    Trk.right_sideline2 = i
                    CCD2_right_flag = True
                    break
    
    if not CCD2_right_flag:
        Trk.right_sideline2 = 117
    
    # CCD2单边丢失补偿
    if CCD2_left_flag and not CCD2_right_flag:
        for i in range(Trk.left_sideline2, min(117, len(ccd_data2))):
            if i + 10 < len(ccd_data2) and (ccd_data2[i] + ccd_data2[i+10]) > 0:
                gradient = abs(ccd_data2[i] - ccd_data2[i+10]) * 100 // (ccd_data2[i] + ccd_data2[i+10])
                if gradient > CCD2.threshold and ccd_data2[i] > ccd_data2[i+10]:
                    Trk.right_sideline2 = i
                    CCD2_right_flag = True
                    break
    
    elif not CCD2_left_flag and CCD2_right_flag:
        for i in range(Trk.right_sideline2, 9, -1):
            if i >= 10 and (ccd_data2[i] + ccd_data2[i-10]) > 0:
                gradient = abs(ccd_data2[i] - ccd_data2[i-10]) * 100 // (ccd_data2[i] + ccd_data2[i-10])
                if gradient > CCD2.threshold and ccd_data2[i] > ccd_data2[i-10]:
                    Trk.left_sideline2 = i
                    CCD2_left_flag = True
                    break
    
    # 黑白场景判断
    black_write_2 = CCD2.aver < CCD2.bin_thrd

def ccd_curvature_calc():
    """曲率计算 - 移植自C语言示例"""
    global Trk, CCD1_SET_WIDTH, CCD2_SET_WIDTH, straight, curve
    
    center = 63.5  # 图像中心
    
    # 左边曲率计算
    if ((Trk.left_sideline1 <= center and Trk.left_sideline2 <= center) or 
        (Trk.left_sideline1 > center and Trk.left_sideline2 > center)):
        # 同侧情况
        Trk.left_qulu = abs(abs(center - Trk.left_sideline1) * 1.0 - 
                           (abs(center - Trk.left_sideline2) * CCD1_SET_WIDTH / CCD2_SET_WIDTH))
    else:
        # 异侧情况
        Trk.left_qulu = abs(abs(center - Trk.left_sideline1) * 1.0 + 
                           (abs(center - Trk.left_sideline2) * CCD1_SET_WIDTH / CCD2_SET_WIDTH))
    
    # 右边曲率计算
    if ((Trk.right_sideline1 >= center and Trk.right_sideline2 >= center) or 
        (Trk.right_sideline1 < center and Trk.right_sideline2 < center)):
        # 同侧情况
        Trk.right_qulu = abs(abs(Trk.right_sideline1 - center) * 1.0 - 
                            (abs(Trk.right_sideline2 - center) * CCD1_SET_WIDTH / CCD2_SET_WIDTH))
    else:
        # 异侧情况
        Trk.right_qulu = abs(abs(Trk.right_sideline1 - center) * 1.0 + 
                            (abs(Trk.right_sideline2 - center) * CCD1_SET_WIDTH / CCD2_SET_WIDTH))
    
    # 直线弯道判断
    if (Trk.right_qulu < 6 and Trk.left_qulu < 6 and 
        abs(Trk.middle_sideline1 - Trk.middle_sideline2) < 6 and
        abs(Trk.middle_sideline1 - center) < 6 and 
        abs(Trk.middle_sideline2 - center) < 6):
        straight = True
        curve = False
    else:
        straight = False
        curve = True

def middle_sideline():
    """中线计算"""
    global Trk, ring_state, ring_left, ring_right
    
    # 基础中线计算
    Trk.middle_sideline1 = (Trk.left_sideline1 + Trk.right_sideline1) / 2.0
    Trk.middle_sideline2 = (Trk.left_sideline2 + Trk.right_sideline2) / 2.0
    
    # 宽度计算
    Trk.width1 = Trk.right_sideline1 - Trk.left_sideline1
    Trk.width2 = Trk.right_sideline2 - Trk.left_sideline2
    
    # 环岛中线修正
    if (ring_state == FIND_RING or ring_state == OUT_RING) and ring_left:
        # 左环岛：找到环岛或出环岛时，中线偏向右边
        Trk.middle_sideline1 = Trk.right_sideline1 - (CCD1_SET_WIDTH / 2)
    elif (ring_state == READY_IN_RING or ring_state == READY_OUT_RING) and ring_left:
        # 左环岛：准备进入或准备出环岛时，中线偏向左边
        Trk.middle_sideline1 = Trk.left_sideline1 + (CCD1_SET_WIDTH / 2)
    elif (ring_state == FIND_RING or ring_state == OUT_RING) and ring_right:
        # 右环岛：找到环岛或出环岛时，中线偏向左边
        Trk.middle_sideline1 = Trk.left_sideline1 + (CCD1_SET_WIDTH / 2)
    elif (ring_state == READY_IN_RING or ring_state == READY_OUT_RING) and ring_right:
        # 右环岛：准备进入或准备出环岛时，中线偏向右边
        Trk.middle_sideline1 = Trk.right_sideline1 - (CCD1_SET_WIDTH / 2)

def ring_detection():
    """环岛检测状态机 - 移植自C语言示例"""
    global ring_state, ring_left, ring_right, encoder_ring, encoder_integral
    global angle_ring, angle_gz, CCD1_left_flag, CCD1_right_flag, black_write_1, black_write_2
    global READY_IN_RING_ENCODER, IN_RING_ENCODER, READY_OUT_RING_ANGLE, OUT_RING_ANGLE, NO_RING_ENCODER
    global RING_QULU_THRESHOLD, RING_QULU_EXIT_THRESHOLD
    
    if ring_state == NO_RING:
        # 检测左环岛 - 使用远端CCD2检测，提前发现环岛入口
        # 左环岛特征：远端CCD左边界丢失，右边界存在，右侧曲率小
        if (not CCD2_left_flag and CCD2_right_flag and 
            Trk.right_qulu <= RING_QULU_THRESHOLD and not black_write_2):
            ring_state = FIND_RING
            ring_left = True
            encoder_ring = encoder_integral
            beep_short()  # 短响表示检测到环岛
            
        # 检测右环岛 - 使用远端CCD2检测，提前发现环岛入口  
        # 右环岛特征：远端CCD右边界丢失，左边界存在，左侧曲率小
        elif (CCD2_left_flag and not CCD2_right_flag and 
              Trk.left_qulu <= RING_QULU_THRESHOLD and not black_write_2):
            ring_state = FIND_RING
            ring_right = True
            encoder_ring = encoder_integral
            beep_short()  # 短响表示检测到环岛
            
    elif ring_state == FIND_RING:
        if ring_left:
            # 左环岛确认
            if (abs(encoder_ring - encoder_integral) > READY_IN_RING_ENCODER and 
                Trk.left_sideline2 < 27):
                ring_state = READY_IN_RING
                angle_gz = 0
                angle_ring = 0
                encoder_ring = encoder_integral
                beep_short()  # 确认环岛
            elif Trk.right_qulu > RING_QULU_EXIT_THRESHOLD:
                ring_state = NO_RING
                ring_left = False
                beep_off()  # 取消环岛，停止响声
                
        if ring_right:
            # 右环岛确认
            if (abs(encoder_ring - encoder_integral) > READY_IN_RING_ENCODER and 
                Trk.right_sideline2 > 100):
                ring_state = READY_IN_RING
                angle_gz = 0
                angle_ring = 0
                encoder_ring = encoder_integral
                beep_short()  # 确认环岛
            elif Trk.left_qulu > RING_QULU_EXIT_THRESHOLD:
                ring_state = NO_RING
                ring_right = False
                beep_off()  # 取消环岛，停止响声
                
    elif ring_state == READY_IN_RING:
        # 准备进入环岛
        if abs(encoder_ring - encoder_integral) > IN_RING_ENCODER:
            ring_state = IN_RING
            beep_long()  # 长响表示进入环岛
            
    elif ring_state == IN_RING:
        # 在环岛中，检测角度变化
        if abs(angle_ring - angle_gz) > READY_OUT_RING_ANGLE:
            ring_state = READY_OUT_RING
            encoder_ring = encoder_integral
            # 移除蜂鸣器，减少噪音干扰
            
    elif ring_state == READY_OUT_RING:
        # 准备出环岛
        if abs(angle_ring - angle_gz) > OUT_RING_ANGLE:
            ring_state = OUT_RING
            encoder_ring = encoder_integral
            # 移除蜂鸣器，减少噪音干扰
            beep_off()

            
    elif ring_state == OUT_RING:
        # 出环岛后，等待一定距离后恢复正常
        if abs(encoder_ring - encoder_integral) > NO_RING_ENCODER:
            ring_state = READY_NO_RING
            
    elif ring_state == READY_NO_RING:
        # 完全退出环岛状态
        ring_state = NO_RING
        ring_left = False
        ring_right = False
        angle_gz = 0
        encoder_ring = 0
        encoder_integral = 0
        beep_off()  # 环岛完成，停止响声

# 移除十字路口和坡道检测函数

def element_detection():
    """元素检测主函数 - 只检测环岛"""
    global encoder_integral, ring_state
    
    # 只有在行驶一定距离后才开始检测元素
    if encoder_integral > 25:
        # 只检测环岛
        ring_detection()

def ccd_processing(ccd_data1, ccd_data2):
    """CCD主处理函数 - 移植自C语言示例"""
    # 1. CCD数据获取和预处理
    ccd1_get(ccd_data1)  # 近端CCD
    ccd2_get(ccd_data2)  # 远端CCD
    
    # 2. 边界检测
    left_right_sideline(ccd_data1, ccd_data2)
    
    # 3. 曲率计算
    ccd_curvature_calc()
    
    # 4. 元素检测
    element_detection()
    
    # 5. 中线计算
    middle_sideline()
    
    # 返回融合后的中线偏差
    center = 63.0  # 赛道中心
    
    # 双CCD融合策略 - 近端为主，远端辅助
    if CCD1_left_flag and CCD1_right_flag:
        # 近端双边界都有效，优先使用
        deviation = Trk.middle_sideline1 - center
    elif CCD2_left_flag and CCD2_right_flag:
        # 远端双边界有效，近端无效时使用
        deviation = Trk.middle_sideline2 - center
    elif CCD1_left_flag or CCD1_right_flag:
        # 近端单边有效
        deviation = Trk.middle_sideline1 - center
    elif CCD2_left_flag or CCD2_right_flag:
        # 远端单边有效
        deviation = Trk.middle_sideline2 - center
    else:
        # 都无效，保持上次偏差
        deviation = line_deviation
    
    return deviation

# 保留原来的calculate_dual_ccd_deviation函数名以兼容现有代码
def calculate_dual_ccd_deviation(upper_center, lower_center):
    """兼容函数 - 简化版本"""
    global line_deviation
    center = 62
    
    # 检查数据有效性
    upper_valid = upper_center != -1
    lower_valid = lower_center != -1
    
    if not upper_valid and not lower_valid:
        return line_deviation
    
    if upper_valid and lower_valid:
        current_deviation = upper_center * 0.6 + lower_center * 0.4 - center
    elif upper_valid:
        current_deviation = upper_center - center
    else:
        current_deviation = lower_center - center
    
    # 限制偏差范围
    current_deviation = max(-35, min(35, current_deviation))
    return current_deviation

def ccd_process(timer):
    """CCD数据处理函数，独立定时器运行 - 使用新的算法"""
    global ccd_ticker_flag, ccd_ticker_count, line_deviation, line_control_output
    
    ccd_ticker_flag = True
    ccd_ticker_count = (ccd_ticker_count + 1) % 100
    
    try:
        # 读取双CCD数据 - 注意：近端ccd.get(1)，远端ccd.get(0)
        ccd_data_upper = ccd.get(0)  # 远端CCD
        ccd_data_lower = ccd.get(1)  # 近端CCD
        
        # 检查数据有效性
        if not ccd_data_upper and not ccd_data_lower:
            return
        
        # 使用新的CCD处理算法 - 参数顺序：近端，远端
        new_deviation = ccd_processing(ccd_data_lower, ccd_data_upper)
        
        # 平滑过渡，避免突变
        if abs(new_deviation - line_deviation) > 15:
            line_deviation = line_deviation * 0.7 + new_deviation * 0.3
        else:
            line_deviation = new_deviation
            
            # 使用PD控制器计算线路控制输出
            line_control_output = pid_line.update(0, line_deviation)  # 目标偏差为0
            
            # 限制线路控制输出
        line_control_output = limit(line_control_output, -3000, 3000)
            
        # 在屏幕上显示CCD数据波形
        try:
            # 显示远端CCD (CCD0) 在屏幕上半部分
            if ccd_data_upper:
                lcd.wave(0, 0, 128, 96, ccd_data_upper, max=4095)
            
            # 显示近端CCD (CCD1) 在屏幕下半部分
            if ccd_data_lower:
                lcd.wave(0, 96, 128, 96, ccd_data_lower, max=4095)
                
            # 显示边界线和中线 - 使用line函数画垂直线
            try:
                # ===== 远端CCD (上半部分) 的边界线和中线 =====
                # 画远端CCD左边界线 (红色)
                if CCD2_left_flag and 0 <= Trk.left_sideline2 <= 127:
                    lcd.line(Trk.left_sideline2, 0, Trk.left_sideline2, 24, color=0xF800, thick=2)
                
                # 画远端CCD右边界线 (红色)
                if CCD2_right_flag and 0 <= Trk.right_sideline2 <= 127:
                    lcd.line(Trk.right_sideline2, 0, Trk.right_sideline2, 24, color=0xF800, thick=2)
                
                # 画远端CCD中线 (绿色)
                if 0 <= int(Trk.middle_sideline2) <= 127:
                    lcd.line(int(Trk.middle_sideline2), 0, int(Trk.middle_sideline2), 24, color=0x07E0, thick=2)
                
                # ===== 近端CCD (下半部分) 的边界线和中线 =====
                # 画近端CCD左边界线 (红色)
                if CCD1_left_flag and 0 <= Trk.left_sideline1 <= 127:
                    lcd.line(Trk.left_sideline1, 96, Trk.left_sideline1, 120, color=0xF800, thick=2)
                
                # 画近端CCD右边界线 (红色)
                if CCD1_right_flag and 0 <= Trk.right_sideline1 <= 127:
                    lcd.line(Trk.right_sideline1, 96, Trk.right_sideline1, 120, color=0xF800, thick=2)
                
                # 画近端CCD中线 (绿色)
                if 0 <= int(Trk.middle_sideline1) <= 127:
                    lcd.line(int(Trk.middle_sideline1), 96, int(Trk.middle_sideline1), 120, color=0x07E0, thick=2)
                
                # 第1行：近端边界位置
                lcd.str12(0, 195, f"L1:{Trk.left_sideline1:3d} R1:{Trk.right_sideline1:3d} M1:{Trk.middle_sideline1:4.1f}", 0xFFFF)
                
                # 第2行：远端边界位置和偏差
                lcd.str12(0, 207, f"L2:{Trk.left_sideline2:3d} R2:{Trk.right_sideline2:3d} Dev:{line_deviation:4.1f}", 0xFFFF)
                
                # 第3行：两侧曲率 (重点显示)
                lcd.str12(0, 219, f"QL:{Trk.left_qulu:5.1f} QR:{Trk.right_qulu:5.1f}", 0x07FF)
                
                # 第4行：CCD1边界检测状态 (重点显示)
                ccd1_status = ""
                ccd1_status += "L1:" + ("Y" if CCD1_left_flag else "N")
                ccd1_status += " R1:" + ("Y" if CCD1_right_flag else "N")
                lcd.str12(0, 231, f"CCD1 {ccd1_status}", 0xF81F)  # 紫色
                
                # 第5行：CCD2边界检测状态
                ccd2_status = ""
                ccd2_status += "L2:" + ("Y" if CCD2_left_flag else "N")
                ccd2_status += " R2:" + ("Y" if CCD2_right_flag else "N")
                lcd.str12(0, 243, f"CCD2 {ccd2_status}", 0xF81F)  # 紫色
                
                # 第6行：黑白场景检测 (重点显示)
                black_status = ""
                black_status += "B1:" + ("Y" if black_write_1 else "N")
                black_status += " B2:" + ("Y" if black_write_2 else "N")
                lcd.str12(0, 255, f"Black {black_status}", 0xFFE0)  # 黄色
                
                # 第7行：环岛状态
                ring_status = ""
                if ring_state == NO_RING: ring_status = "NoRing"
                elif ring_state == FIND_RING: ring_status = "Find"
                elif ring_state == READY_IN_RING: ring_status = "ReadyIn"
                elif ring_state == IN_RING: ring_status = "InRing"
                elif ring_state == READY_OUT_RING: ring_status = "ReadyOut"
                elif ring_state == OUT_RING: ring_status = "OutRing"
                elif ring_state == READY_NO_RING: ring_status = "ReadyNo"
                
                ring_dir = ""
                if ring_left: ring_dir = "L"
                elif ring_right: ring_dir = "R"
                
                lcd.str12(0, 267, f"Ring:{ring_status}{ring_dir}", 0xF800)  # 红色
                
                # 第8行：系统状态
                lcd.str12(0, 279, f"System:Running", 0x07E0)  # 绿色
                
            except:
                pass
                
        except:
            pass  # 显示出错不影响主要功能
            
    except Exception as e:
        # 发生错误时逐渐减小控制输出，避免突然停止
        line_control_output *= 0.9

# 初始化定时器
pit1 = ticker(1)
pit3 = ticker(3)
pit2 = ticker(2)  # CCD处理定时器
pit1.capture_list(imu)
pit3.capture_list(encoder_l, encoder_r)
pit2.capture_list(ccd)  # CCD定时器捕获CCD
pit1.callback(control_loop)
pit3.callback(encoder_update)
pit2.callback(ccd_process)  # CCD处理回调

# 启动系统
imu_init()
ccd_image_init()  # 初始化CCD图像处理
pit1.start(1)
pit3.start(10)
pit2.start(8)  # CCD

# 系统启动完成
print("init")

# 主循环
while True:
    if ticker_flag:
        ticker_flag = False
    
    if ccd_ticker_flag:
        ccd_ticker_flag = False
    
    # WiFi调参更新
    update_wifi_parameters()
    
    if end_switch.value() != end_state:
        pit1.stop()
        pit3.stop()
        pit2.stop()  # 停止CCD定时器
        break
    
    gc.collect()



