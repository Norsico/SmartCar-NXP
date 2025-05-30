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

# 硬件初始化
end_switch = Pin('D20', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
end_state = end_switch.value()

motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=True)

encoder_l = encoder("C0", "C1", True)
encoder_r = encoder("C2", "C3")

imu = IMU660RX()
imu_data = imu.get()

# PID参数
angle_kp, angle_ki, angle_kd = -1180.0, 0.0, -195.0
roll_angle_Kp, roll_angle_Ki, roll_angle_Kd = 0.04, 0.00004, 0.16
speed_Kp, speed_Ki, speed_Kd = 0.095, 0.000012, 0.015

# 控制变量
angle_1 = speed_1 = motor1 = motor2 = 0
med_roll_angle = 25
TARGET_SPEED = 10
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
imu_data_obj = IMUData()
quaternion = Quaternion()

# 积分误差
I_ex = I_ey = I_ez = 0.0
delta_T = 0.001
param_Kp, param_Ki = 15.5, 0.006

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
    
    # 加速度滤波
    alpha = 0.3
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
    imu_data_obj.Pitch = math.atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1,
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
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2, imu_data
    
    ticker_flag = True
    ticker_count = (ticker_count + 1) % 10
    
    # 1ms: 角速度控制
    imu_data = imu.get()
    imu_process()
    
    motor1 = pid_angle_speed.update(angle_1, -imu_data_obj.gyro_x)
    motor2 = motor1
    motor1 = limit(motor1, -4000, 4000)
    motor2 = limit(motor2, -4000, 4000)
    
    motor_l.duty(motor1)
    motor_r.duty(-motor2)
    
    # 5ms: 角度控制
    if ticker_count % 5 == 0:
        angle_1 = pid_angle.update(med_roll_angle - speed_1, -imu_data_obj.Pitch)
    
    # 10ms: 速度控制
    if ticker_count == 0:
        avg_speed = (kalman_l.output + kalman_r.output) / 2
        speed_1 = pid_speed.update(TARGET_SPEED, avg_speed)

def encoder_update(timer):
    kalman_l.update(encoder_l.get())
    kalman_r.update(encoder_r.get())

# 初始化定时器
pit1 = ticker(1)
pit3 = ticker(3)
pit1.capture_list(imu)
pit3.capture_list(encoder_l, encoder_r)
pit1.callback(control_loop)
pit3.callback(encoder_update)

# 启动系统
imu_init()
pit1.start(1)
pit3.start(10)

# 主循环
while True:
    if ticker_flag:
        ticker_flag = False
    
    if end_switch.value() != end_state:
        pit1.stop()
        pit3.stop()
        print("系统停止")
        break
    
    gc.collect()
