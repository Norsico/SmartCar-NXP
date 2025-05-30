from machine import *
from display import *
from smartcar import *
from seekfree import *
import gc
import time
import math

# 全局变量
PI = 3.14159265358
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

# PID参数 - 进一步增强响应强度
angle_kp, angle_ki, angle_kd = -2600.0, 0, -420.0  # 进一步增强角速度环响应
roll_angle_Kp, roll_angle_Ki, roll_angle_Kd = 0.09, 0, 0.28  # 进一步增强角度环响应
speed_Kp, speed_Ki, speed_Kd = 0.095, 0, 0.015

# 线路跟踪PD控制器参数
line_kp = 15  # 比例控制，快速响应
line_kd = 5  # 微分控制，提高稳定性

# 控制变量
angle_1 = speed_1 = motor1 = motor2 = 0
med_roll_angle = 20.5  # 调整平衡角度
TARGET_SPEED = 20  # 设置小的前进速度进行测试
ticker_count = 0

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
    
    # 添加线路控制输出到电机控制
    motor1 -= line_control_output  # 左电机增加转向控制
    motor2 += line_control_output  # 右电机减少转向控制
    
    motor1 = limit(motor1, -7200, 7200)  # 增加电机输出限制，提高响应强度
    motor2 = limit(motor2, -7200, 7200)  # 增加电机输出限制，提高响应强度
    
    motor_l.duty(motor1)
    motor_r.duty(motor2)
    
    # 5ms: 角度控制
    if ticker_count % 5 == 0:
        angle_1 = pid_angle.update(med_roll_angle - speed_1, imu_data_obj.Pitch)
    
    # 10ms: 速度控制
    if ticker_count == 0:
        avg_speed = (kalman_l.output + kalman_r.output) / 2
        speed_1 = pid_speed.update(TARGET_SPEED, avg_speed)
        speed_1 = limit(speed_1, -5, 5)  # 限制角度偏移在±5度内

def encoder_update(timer):
    kalman_l.update(encoder_l.get())
    kalman_r.update(encoder_r.get())

def handle_ccd_data(arr):
    """
    处理CCD数据，找出大于阈值的数据点并计算它们的坐标中间值
    arr: CCD数据数组
    返回: 有效数据点的坐标中间值，如果没有有效数据点则返回-1
    """
    threshold = 2000
    valid_indices = []
    
    # 找出所有大于阈值的数据点的索引
    for i, value in enumerate(arr):
        if value > threshold:
            valid_indices.append(i)
    
    if not valid_indices:
        return -1
    
    # 如果检测点太少，直接返回中间值
    if len(valid_indices) < 3:
        return sum(valid_indices) / len(valid_indices)
    
    # 寻找最大的连续区域（主赛道）
    max_group = []
    current_group = [valid_indices[0]]
    
    for i in range(1, len(valid_indices)):
        # 如果相邻点距离小于3个像素，认为是连续的
        if valid_indices[i] - valid_indices[i-1] <= 3:
            current_group.append(valid_indices[i])
        else:
            # 断开了，检查当前组是否更大
            if len(current_group) > len(max_group):
                max_group = current_group.copy()
            current_group = [valid_indices[i]]
    
    # 检查最后一组
    if len(current_group) > len(max_group):
        max_group = current_group.copy()
    
    # 如果找到连续区域，使用该区域的中心
    if max_group:
        return sum(max_group) / len(max_group)
    else:
        # 没有连续区域，使用最接近中心的点
        center = 64
        closest_point = min(valid_indices, key=lambda x: abs(x - center))
    # 如果有有效数据点，计算它们的中间值
    if valid_indices:
        middle_index = sum(valid_indices) / len(valid_indices)
        return middle_index
    else:
        # 如果没有有效数据点，返回-1表示未找到赛道
        return -1

def calculate_line_deviation(middle1, middle2):
    """
    根据两个CCD的中间值计算线路偏差
    middle1: CCD1的中间值 (上方摄像头)
    middle2: CCD2的中间值 (下方摄像头)
    返回: 线路偏差值
    """
    global line_deviation
    center = 62  # 赛道中心位置
    current_deviation = 0
    
    # 如果两个CCD都检测到线路
    if middle1 != -1 and middle2 != -1:
        # 检查两个CCD数据是否合理（差距不应该太大）
        diff = abs(middle1 - middle2)
        if diff < 15:  # 如果两个CCD读数相近，说明是直线
            current_deviation = (middle1 * 0.6 + middle2 * 0.4) - center
        else:
            # 差距较大时，优先相信上方CCD（预判作用）
            current_deviation = middle1 - center
    elif middle1 != -1:
        # 只有上方CCD检测到
        current_deviation = middle1 - center
    elif middle2 != -1:
        # 只有下方CCD检测到
        current_deviation = middle2 - center
    else:
        # 都没检测到，保持上次偏差
        return line_deviation
    
    # 异常值过滤：如果偏差变化过大，可能是干扰
    if abs(current_deviation - line_deviation) > 20:
        # 偏差变化太大，使用加权平均平滑过渡
        current_deviation = line_deviation * 0.7 + current_deviation * 0.3
    
    # 限制偏差范围
    current_deviation = max(-30, min(30, current_deviation))
    
    return current_deviation

def ccd_process(timer):
    """CCD数据处理函数，独立定时器运行"""
    global ccd_ticker_flag, ccd_ticker_count, line_deviation, line_control_output
    
    ccd_ticker_flag = True
    ccd_ticker_count = (ccd_ticker_count + 1) % 100
    
    try:
        # 读取两个CCD的数据
        ccd_data1 = ccd.get(0)  # 上方CCD
        ccd_data2 = ccd.get(1)  # 下方CCD
        
        # 检查数据有效性
        if not ccd_data1 or not ccd_data2:
            return
        
        # 处理CCD数据获取中间值
        middle_value1 = handle_ccd_data(ccd_data1)
        middle_value2 = handle_ccd_data(ccd_data2)
        
        # 计算线路偏差
        new_deviation = calculate_line_deviation(middle_value1, middle_value2)
        
        # 只有当偏差有效时才更新
        if new_deviation is not None:
            line_deviation = new_deviation
            
            # 使用PD控制器计算线路控制输出
            line_control_output = pid_line.update(0, line_deviation)  # 目标偏差为0
            
            # 限制线路控制输出
            line_control_output = limit(line_control_output, -2000, 2000)
        
        # 每20次循环打印一次调试信息
        if ccd_ticker_count % 20 == 0:
            status1 = "OK" if middle_value1 != -1 else "MISS"
            status2 = "OK" if middle_value2 != -1 else "MISS"
            print("CCD1:{:.1f}[{}] CCD2:{:.1f}[{}] 偏差:{:.1f} 输出:{:.0f}".format(
                middle_value1 if middle_value1 != -1 else 0, status1,
                middle_value2 if middle_value2 != -1 else 0, status2,
                line_deviation, line_control_output))
            
    except Exception as e:
        print("CCD处理错误:", e)
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
pit2.start(10)  # CCD以10ms间隔运行，快速响应

# 主循环
while True:
    if ticker_flag:
        ticker_flag = False
    
    if ccd_ticker_flag:
        ccd_ticker_flag = False
    
    if end_switch.value() != end_state:
        pit1.stop()
        pit3.stop()
        pit2.stop()  # 停止CCD定时器
        print("系统停止")
        break
    
    gc.collect()




