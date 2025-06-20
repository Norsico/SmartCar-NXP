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

# 硬件初始化
end_switch = Pin('D20', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
end_state = end_switch.value()

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
angle_kp = -1839
angle_ki = 0
angle_kd =55

roll_angle_Kp = 0.095
roll_angle_Ki = 0
roll_angle_Kd = 0.0855  # 进一步增强角度环响应

speed_Kp = 0.09530006
speed_Ki = 0
speed_Kd = 3.98

# 线路跟踪PD控制器参数
line_kp = 16.9  # 比例控制，快速响应
line_kd = 415  # 微分控制，提高稳定性

# 控制变量
angle_1 = speed_1 = motor1 = motor2 = 0
med_roll_angle = 34.1  # 调整平衡角度
TARGET_SPEED = 85  # 设置小的前进速度进行测试
ticker_count = 0

# WiFi调参数据存储
wifi_data = [angle_kp, angle_kd, roll_angle_Kp, roll_angle_Kd, speed_Kp, line_kd, TARGET_SPEED, line_kp]

def update_wifi_parameters():
    """更新WiFi调参数据"""
    global angle_kd, roll_angle_Kp, roll_angle_Kd, speed_Kd, TARGET_SPEED, line_kp, line_kd
    global med_roll_angle, pid_angle_speed, pid_angle, pid_speed, pid_line, wifi_data, motor1, motor2
    
    if not wifi_enabled:
        return
    
    try:
        # 数据解析
        data_flag = wifi.data_analysis()
        
        # 检查各通道是否有数据更新
        for i in range(8):
            if data_flag[i]:
                wifi_data[i] = wifi.get_data(i)
        
        # 更新PID参数
        angle_kp = wifi_data[0]
        angle_kd = wifi_data[1]
        roll_angle_Kp = wifi_data[2]
        roll_angle_Kd = wifi_data[3]
        speed_Kp = wifi_data[4]
        line_kd = wifi_data[5]
        TARGET_SPEED = wifi_data[6]
        line_kp = wifi_data[7]
        
        # 重新初始化PID控制器以应用新参数
        pid_angle_speed.kp = angle_kp
        pid_angle_speed.ki = angle_ki  # 保持原值
        pid_angle_speed.kd = angle_kd
        
        pid_angle.kp = roll_angle_Kp
        pid_angle.ki = roll_angle_Ki  # 保持原值
        pid_angle.kd = roll_angle_Kd
        
        pid_speed.kp = speed_Kp
        pid_speed.ki = speed_Ki  # 保持原值
        pid_speed.kd = speed_Kd  # 保持原值
        
        # 更新巡线PD控制器参数
        pid_line.kp = line_kp
        pid_line.kd = line_kd
        
        # 发送示波器数据 - 通道0显示imu_data_obj.Pitch，通道1、2显示motor1、motor2
        wifi.send_oscilloscope(
            imu_data_obj.Pitch, motor1, motor2, 
            wifi_data[2], wifi_data[3], wifi_data[4], wifi_data[5], wifi_data[6])
    
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
    kalman_l.update(encoder_l.get())
    kalman_r.update(encoder_r.get())

def handle_ccd_data(arr, ccd_id):
    """
    处理单个CCD数据，找出大于阈值的数据点并计算它们的坐标中间值
    增强版：能够识别多条线并选择最长的线，抗干扰能力强
    arr: CCD数据数组
    ccd_id: CCD标识（0为上方，1为下方）
    返回: 有效数据点的坐标中间值，如果没有有效数据点则返回-1
    """
    threshold = 2500  # 提高阈值，减少噪声干扰
    min_line_length = 5  # 最小线段长度，过滤小的干扰点
    max_gap = 2  # 允许的最大间隙，小间隙会被忽略
    
    # 找出所有大于阈值的数据点
    valid_indices = []
    for i, value in enumerate(arr):
        if value > threshold:
            valid_indices.append(i)
    
    if not valid_indices:
        return -1
    
    # 将连续的点分组，形成线段
    line_segments = []
    current_segment = [valid_indices[0]]
    
    for i in range(1, len(valid_indices)):
        gap = valid_indices[i] - valid_indices[i-1]
        
        if gap <= max_gap + 1:  # 连续或小间隙
            current_segment.append(valid_indices[i])
        else:
            # 间隙太大，结束当前线段
            if len(current_segment) >= min_line_length:
                line_segments.append(current_segment.copy())
            current_segment = [valid_indices[i]]
    
    # 添加最后一个线段
    if len(current_segment) >= min_line_length:
        line_segments.append(current_segment)
    
    if not line_segments:
        return -1
    
    # 找到最长的线段
    longest_segment = max(line_segments, key=len)
    
    # 对最长线段进行质量评估
    segment_length = len(longest_segment)
    segment_start = longest_segment[0]
    segment_end = longest_segment[-1]
    segment_width = segment_end - segment_start + 1
    
    # 如果线段太短，可能是干扰
    if segment_length < min_line_length:
        return -1
    
    # 计算线段中心，使用加权平均减少边缘效应
    if segment_length >= 10:
        # 对于较长的线段，去掉两端的20%，使用中间部分计算中心
        trim_count = max(1, segment_length // 5)
        trimmed_segment = longest_segment[trim_count:-trim_count]
        if trimmed_segment:
            center_position = sum(trimmed_segment) / len(trimmed_segment)
        else:
            center_position = sum(longest_segment) / len(longest_segment)
    else:
        # 短线段直接计算中心
        center_position = sum(longest_segment) / len(longest_segment)
    
    return center_position

def calculate_dual_ccd_deviation(upper_center, lower_center):
    """
    双CCD融合算法：根据上下两个CCD的中间值计算融合后的线路偏差
    upper_center: 上方CCD中心位置（前瞻性强）
    lower_center: 下方CCD中心位置（精确性强）
    返回: 融合后的线路偏差值
    """
    global line_deviation
    center = 62  # 赛道中心位置
    
    # 检查数据有效性
    upper_valid = upper_center != -1
    lower_valid = lower_center != -1
    
    if not upper_valid and not lower_valid:
        # 都没检测到，保持上次偏差
        return line_deviation
    
    # 融合策略
    if upper_valid and lower_valid:
        # 双CCD都有效时的融合算法
        upper_deviation = upper_center - center
        lower_deviation = lower_center - center
        
        # 检查两个CCD读数一致性
        deviation_diff = abs(upper_deviation - lower_deviation)
        
        if deviation_diff < 15:
            # 读数一致，说明是直线或缓弯
            # 上方CCD权重稍高，提供前瞻性
            current_deviation = upper_deviation * 0.6 + lower_deviation * 0.4
        elif deviation_diff < 25:
            # 读数有差异，可能是弯道
            # 根据偏差大小动态调整权重
            if abs(upper_deviation) > abs(lower_deviation):
                # 上方CCD偏差更大，可能检测到即将到来的弯道
                current_deviation = upper_deviation * 0.5 + lower_deviation * 0.5
            else:
                # 下方CCD偏差更大，以精确跟踪为主
                current_deviation = upper_deviation * 0.3 + lower_deviation * 0.7
        else:
            # 读数差异很大，可能有干扰，优先相信下方CCD
            current_deviation = upper_deviation * 0.2 + lower_deviation * 0.8
            
    elif upper_valid:
        # 只有上方CCD有效
        current_deviation = upper_center - center
    else:
        # 只有下方CCD有效
        current_deviation = lower_center - center
    
    # 异常值过滤：如果偏差变化过大，可能是干扰
    if abs(current_deviation - line_deviation) > 25:
        # 偏差变化太大，使用加权平均平滑过渡
        current_deviation = line_deviation * 0.6 + current_deviation * 0.4
    
    # 限制偏差范围
    current_deviation = max(-35, min(35, current_deviation))
    
    return current_deviation

def ccd_process(timer):
    """CCD数据处理函数，独立定时器运行"""
    global ccd_ticker_flag, ccd_ticker_count, line_deviation, line_control_output
    
    ccd_ticker_flag = True
    ccd_ticker_count = (ccd_ticker_count + 1) % 100
    
    try:
        # 读取双CCD数据
        ccd_data_upper = ccd.get(0)  # 上方CCD（前瞻性）
        ccd_data_lower = ccd.get(1)  # 下方CCD（精确性）
        
        # 检查数据有效性
        if not ccd_data_upper and not ccd_data_lower:
            return
        
        # 处理双CCD数据获取中间值
        upper_center = -1
        lower_center = -1
        
        if ccd_data_upper:
            upper_center = handle_ccd_data(ccd_data_upper, 0)
            
        if ccd_data_lower:
            lower_center = handle_ccd_data(ccd_data_lower, 1)
        
        # 使用双CCD融合算法计算线路偏差
        new_deviation = calculate_dual_ccd_deviation(upper_center, lower_center)
        
        # 只有当偏差有效时才更新
        if new_deviation is not None:
            line_deviation = new_deviation
            
            # 使用PD控制器计算线路控制输出
            line_control_output = pid_line.update(0, line_deviation)  # 目标偏差为0
            
            # 限制线路控制输出
            line_control_output = limit(line_control_output, -2000, 2000)
            
        # 在屏幕上显示CCD数据波形
        try:
            # 显示上方CCD (CCD0) 在屏幕上半部分
            if ccd_data_upper:
                lcd.wave(0, 0, 128, 96, ccd_data_upper, max=4095)
            
            # 显示下方CCD (CCD1) 在屏幕下半部分
            if ccd_data_lower:
                lcd.wave(0, 96, 128, 96, ccd_data_lower, max=4095)
                
            # 在屏幕上显示一些关键信息
            # lcd.str(0, 200, f"U:{upper_center:.0f} L:{lower_center:.0f} D:{line_deviation:.1f}", size=12)
        except:
            pass  # 显示出错不影响主要功能
        
        # 调试信息（可选）
        # print(f"上方CCD: {upper_center:.1f}, 下方CCD: {lower_center:.1f}, 融合偏差: {line_deviation:.1f}")
            
    except Exception as e:
        # 发生错误时逐渐减小控制输出，避免突然停止
        line_control_output *= 0.8

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
pit1.start(1)
pit3.start(10)
pit2.start(8)  # CCD

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


