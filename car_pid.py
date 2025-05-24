from machine import *
from display import *
from smartcar import *
from seekfree import *
import gc
import time
import math
import ustruct

# 日志记录标志
LOG_ENABLED = True
LOG_INTERVAL = 100  # 每隔多少次循环记录一次日志
log_counter = 0

err_1 = 0
err_sum_1 = 0
err_last_1 = 0
err_2 = 0
err_sum_2 = 0
err_last_2 = 0
err_3 = 0
err_sum_3 = 0
err_last_3 = 0
min_value = 0
max_value = 0
ticker_flag = 0
ticker_flag2 = 0
Filter_data = [0, 0, 0]
PI = 3.14159265358
last_yaw = 0
# 开关
end_switch = Pin('D20', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
end_state = end_switch.value()

# 电机初始化
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=True)

# 编码器初始化
encoder_l = encoder("C0", "C1", True)
encoder_r = encoder("C2", "C3")

# 陀螺仪初始化
imu = IMU660RX()
imu_data = imu.get()

ticker_count = 0
ticker_count2 = 0

##############################################UART
uart2 = UART(2)
uart2.init(115200)

# 串级参数 - 调整PID参数以减小振荡
# ////////////角速度//////////////////////
angle_kp = -1200.0  # 原值-1500.0，进一步减小以降低抖动
angle_ki = -0.5  # 原值-0.8，降低积分作用进一步减少振荡
angle_kd = -180.0  # 原值-150.0，增加微分作用抑制振荡
# ////////////角度//////////////////////
roll_angle_Kp = 0.055  # 原值0.07，减小以降低响应强度
roll_angle_Ki = 0.00005  # 原值0.0001，减小以降低积分作用
roll_angle_Kd = 0.12  # 原值0.09，增加以增强稳定性
# ////////////速度//////////////////////
speed_Kp = 0.018  # 原值0.025，进一步减小以减少摆动幅度
speed_Ki = 0.000015  # 原值0.00002，减小以降低长期累积效应
speed_Kd = 0.008  # 原值0.005，增加以提供更好的阻尼效果，抑制摆动
angle_1 = 0
speed_1 = 0
motor1 = 0
motor2 = 0
med_roll_angle = 26.1

# 前进速度控制参数
TARGET_SPEED = 15  # 目标速度，正值前进，负值后退，0为静止平衡
                  # 推荐范围：0-20，建议从小值(5-8)开始尝试
                  # 调整此参数控制小车前进后退
FORWARD_OFFSET = 0.1  # 前进偏移补偿，微调前进姿态

# 电机死区补偿参数
MOTOR_DEADBAND = 230  # 原值250，减小死区补偿，提高低速响应平滑性

# 平衡静态偏移调整
# 可调整此参数使机器人完全静止
STATIC_OFFSET = 0.06  # 原值0.08，减小静态偏移，与新的med_roll_angle匹配
# 当机器人向前漂移时，增加此值；向后漂移时，减小此值

# 左右电机平衡调整参数（解决转圈问题）
LEFT_MOTOR_FACTOR = 0.95  # 左电机输出缩放因子，小于1可减少逆时针转动

# 静止模式控制参数
STATIC_SPEED_THRESHOLD = 12  # 原值8，增大静止阈值，更容易进入静止模式
STATIC_CONTROL_FACTOR = 0.4  # 原值0.5，减小静止模式控制强度，降低摆动

# 速度积分限制参数
SPEED_INTEGRAL_LIMIT = 500  # 原值800，减小速度积分限幅，防止积分累积导致的摆动

# 速度平滑参数
SPEED_SMOOTH_FACTOR = 0.7  # 速度值平滑系数，防止速度突变引起的摆动
last_speed_value = 0  # 上一次的速度值

# 运动模式（默认为平衡模式）
MOTION_MODE = 0  # 0=平衡静止，1=缓慢前进，2=加速，3=减速...
# 可以通过外部按键或串口命令切换模式

#################################################编码器卡尔曼滤波
KAL_P = 0.02  # 估算协方差
KAL_G = 0.0  # 卡尔曼增益
KAL_Q = 0.50  # 过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏 (原值0.70)
KAL_R = 250  # 测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好 (原值200)
KAL_Output = 0.0  # 卡尔曼滤波器输出

KAL_P2 = 0.02  # 估算协方差
KAL_G2 = 0.0  # 卡尔曼增益
KAL_Q2 = 0.50  # 过程噪声协方差 (原值0.70)
KAL_R2 = 250  # 测量噪声协方差 (原值200)
KAL_Output2 = 0.0  # 卡尔曼滤波器输出

# 日志记录函数
def log_data(message, data=None):
    if LOG_ENABLED:
        if data is not None:
            uart2.write("{}: {}\n".format(message, data))
        else:
            uart2.write("{}\n".format(message))

######################################################
def limit(value, min_value, max_value):
    if value < min_value:
        value = min_value
    elif value > max_value:
        value = max_value
    else:
        value = value
    return value

# 电机死区补偿函数
def motor_output_with_deadband(duty):
    # 对于接近零的小信号，直接输出零，防止电机抖动
    if abs(duty) < 30:  # 原值50，降低小信号阈值
        return 0
    
    # 死区补偿
    if duty > 0:
        return duty + MOTOR_DEADBAND
    elif duty < 0:
        return duty - MOTOR_DEADBAND
    else:
        return 0

# 位置式PID控制
def calculate_pid(err, err_sum, err_last, med, value, kp, ki, kd):
    err = med - value
    err_sum += err
    # 限制积分项，防止积分饱和
    err_sum = limit(err_sum, -2000, 2000)
    err_x = err - err_last
    pwm = kp * err + ki * err_sum + kd * err_x
    err_last = err
    return pwm, err_sum


def pid_position_1(med, value, kp, ki, kd):
    global err_1, err_sum_1, err_last_1
    pwm_1, err_sum_1 = calculate_pid(err_1, err_sum_1, err_last_1, med, value, kp, ki, kd)
    err_last_1 = err_1
    return pwm_1


def pid_position_2(med, value, kp, ki, kd):
    global err_2, err_sum_2, err_last_2
    pwm_2, err_sum_2 = calculate_pid(err_2, err_sum_2, err_last_2, med, value, kp, ki, kd)
    err_last_2 = err_2
    return pwm_2


def pid_position_3(med, value, kp, ki, kd):
    global err_3, err_sum_3, err_last_3
    # 对速度环积分项进行特殊限制
    err = med - value
    err_sum_3 += err
    # 更严格地限制速度积分，防止缓慢漂移
    err_sum_3 = limit(err_sum_3, -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT)
    err_x = err - err_last_3
    pwm_3 = kp * err + ki * err_sum_3 + kd * err_x
    err_last_3 = err
    return pwm_3

class bianmaqi:
    def __init__(self, KAL_templ_pluse, KAL_tempr_pluse):
        self.KAL_templ_pluse = KAL_templ_pluse
        self.KAL_tempr_pluse = KAL_tempr_pluse


Encoders = bianmaqi(0, 0)


class Imu_element:
    def __init__(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, Pitch, Roll, Yaw, X, Y, Z, Total_Yaw):
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.gyro_x = gyro_x
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z
        self.Pitch = Pitch
        self.Roll = Roll
        self.Yaw = Yaw
        self.X = X
        self.Y = Y
        self.Z = Z
        self.Total_Yaw = Total_Yaw

Imu = Imu_element(0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00)

#############################################################
class param():
    def __init__(self, param_Kp, param_Ki):
        self.param_Kp = param_Kp
        self.param_Ki = param_Ki

Param = param(15.5, 0.006)


class QInfo:
    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0


Q_info = QInfo()
delta_T = 0.001  # 采样周期
I_ex, I_ey, I_ez = 0.0, 0.0, 0.0  # 积分误差


def invSqrt(x):
    if x <= 0:
        return 1.0
    return 1.0 / (math.sqrt(x))


#############################################################
max_gyro_x = 0
############################陀螺仪############################

def Limit(value):
    if value > 1:
        value = 1
    elif value < -1:
        value = -1
    return value

# 姿态解算函数
def Imu660():
    alpha = 0.3
    global imu_data, max_gyro_x
    # 更严格的阈值过滤，防止异常数据
    if abs(imu_data[3]) < 30 or abs(imu_data[3]) > 30000:
        imu_data[3] = 0
    if abs(imu_data[4]) < 30 or abs(imu_data[4]) > 30000:
        imu_data[4] = 0
    if abs(imu_data[5]) < 30 or abs(imu_data[5]) > 30000:
        imu_data[5] = 0

    Imu.X = int(imu_data[3] / 16.4)
    Imu.Y = int(imu_data[4] / 16.4)  # 俯仰角
    Imu.Z = int(imu_data[5] / 16.4)
    
    # 使用低通滤波处理陀螺仪数据，减少高频噪声
    gyro_filter_alpha = 0.7  # 低通滤波系数
    
    new_gyro_x = round((float(imu_data[3]) - Filter_data[0]), 3) * PI / 180 / 16.4
    new_gyro_y = round((float(imu_data[4]) - Filter_data[1]), 3) * PI / 180 / 16.4
    new_gyro_z = round((float(imu_data[5]) - Filter_data[2]), 3) * PI / 180 / 14.4
    
    Imu.gyro_x = gyro_filter_alpha * new_gyro_x + (1-gyro_filter_alpha) * Imu.gyro_x
    Imu.gyro_y = gyro_filter_alpha * new_gyro_y + (1-gyro_filter_alpha) * Imu.gyro_y
    Imu.gyro_z = gyro_filter_alpha * new_gyro_z + (1-gyro_filter_alpha) * Imu.gyro_z
    
    Imu.acc_x = round(((float(imu_data[0]) * alpha) / 4096 + Imu.acc_x * (1 - alpha)), 3)
    Imu.acc_y = round(((float(imu_data[1]) * alpha) / 4096 + Imu.acc_y * (1 - alpha)), 3)
    Imu.acc_z = round(((float(imu_data[2]) * alpha) / 4096 + Imu.acc_z * (1 - alpha)), 3)
    # 四元素调用
    IMU_AHRSupdate(Imu.gyro_x, Imu.gyro_y, Imu.gyro_z, Imu.acc_x, Imu.acc_y, Imu.acc_z)
    if abs(max_gyro_x) < abs(Imu.Pitch):
        max_gyro_x = Imu.Pitch

# 陀螺仪初始化
def Imu_Init():
    global Filter_data
    global imu_data
    log_data("开始陀螺仪初始化")
    Filter_data[0] = 0
    Filter_data[1] = 0
    Filter_data[2] = 0

    for i in range(0, 1000):
        imu_data = imu.get()
        Filter_data[0] += imu_data[3]
        Filter_data[1] += imu_data[4]
        Filter_data[2] += imu_data[5]
        time.sleep_ms(1)
    Filter_data[0] = float(Filter_data[0] / 1000)
    Filter_data[1] = float(Filter_data[1] / 1000)
    Filter_data[2] = float(Filter_data[2] / 1000)
    log_data("陀螺仪初始化完成", [Filter_data[0], Filter_data[1], Filter_data[2]])

# 四元数
def IMU_AHRSupdate(gx, gy, gz, ax, ay, az):
    global I_ex, I_ey, I_ez, last_yaw
    halfT = 0.5 * delta_T
    value1 = 0
    # 当前的机体坐标系上的重力单位向量
    vx, vy, vz = 0.0, 0.0, 0.0
    ex, ey, ez = 0.0, 0.0, 0.0
    q0, q1, q2, q3 = 0.0, 0.0, 0.0, 0.0

    q0q0 = Q_info.q0 * Q_info.q0
    q0q1 = Q_info.q0 * Q_info.q1
    q0q2 = Q_info.q0 * Q_info.q2
    q1q1 = Q_info.q1 * Q_info.q1
    q1q3 = Q_info.q1 * Q_info.q3
    q2q2 = Q_info.q2 * Q_info.q2
    q2q3 = Q_info.q2 * Q_info.q3
    q3q3 = Q_info.q3 * Q_info.q3

    # 对加速度数据进行归一化
    norm = invSqrt(ax * ax + ay * ay + az * az)
    ax *= norm
    ay *= norm
    az *= norm

    # 计算当前重力单位向量
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3

    # 计算误差
    ex = ay * vz - az * vy
    ey = az * vx - ax * vz
    ez = ax * vy - ay * vx

    # 用误差进行PI修正
    I_ex += delta_T * ex  # 积分误差
    I_ey += delta_T * ey
    I_ez += delta_T * ez
    
    # 限制积分项增长，防止积分饱和
    I_ex = limit(I_ex, -10, 10)
    I_ey = limit(I_ey, -10, 10)
    I_ez = limit(I_ez, -10, 10)

    gx += Param.param_Kp * ex + Param.param_Ki * I_ex
    gy += Param.param_Kp * ey + Param.param_Ki * I_ey
    gz += Param.param_Kp * ez + Param.param_Ki * I_ez

    # 四元数微分方程
    q0 = Q_info.q0
    q1 = Q_info.q1
    q2 = Q_info.q2
    q3 = Q_info.q3

    Q_info.q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT
    Q_info.q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT
    Q_info.q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT
    Q_info.q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT

    # 归一化四元数
    norm = invSqrt(Q_info.q0 ** 2 + Q_info.q1 ** 2 + Q_info.q2 ** 2 + Q_info.q3 ** 2)
    Q_info.q0 *= norm
    Q_info.q1 *= norm
    Q_info.q2 *= norm
    Q_info.q3 *= norm

    # 计算欧拉角
    value1 = Limit(-2 * Q_info.q1 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q2)
    Imu.Roll = round(math.asin(value1) * 180 / math.pi, 3)  # pitch
    Imu.Pitch = round(math.atan2(2 * Q_info.q2 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q1,
                                 -2 * Q_info.q1 ** 2 - 2 * Q_info.q2 ** 2 + 1) * 180 / math.pi, 3)  # roll
    Imu.Yaw = round(math.atan2(2 * Q_info.q1 * Q_info.q2 + 2 * Q_info.q0 * Q_info.q3,
                               -2 * Q_info.q2 ** 2 - 2 * Q_info.q3 ** 2 + 1) * 180 / math.pi, 3)  # yaw
    # 计算偏航角的误差
    error_yaw = Imu.Yaw - last_yaw
    if error_yaw < -360:
        error_yaw += 360
    if error_yaw > 360:
        error_yaw -= 360
    Imu.Total_Yaw += error_yaw
    last_yaw = Imu.Yaw

    # 保证total_yaw在0到360度之间
    if Imu.Total_Yaw > 360:
        Imu.Total_Yaw -= 360
    if Imu.Total_Yaw < 0:
        Imu.Total_Yaw += 360



def KalmanFilter(input):
    global KAL_P
    global KAL_G
    global KAL_Output

    KAL_P = KAL_P + KAL_Q  # 估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    KAL_G = KAL_P / (KAL_P + KAL_R)  # //卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    # 更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    KAL_Output = KAL_Output + KAL_G * (input - KAL_Output)  # 当前 估算值 = 上次 最优值
    KAL_P = (1 - KAL_G) * KAL_P  # 更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差
    return KAL_Output


def KalmanFilter2(input):
    global KAL_P2
    global KAL_G2
    global KAL_Output2

    KAL_P2 = KAL_P2 + KAL_Q2  # 估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    KAL_G2 = KAL_P2 / (KAL_P2 + KAL_R2)  # //卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    # 更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    KAL_Output2 = KAL_Output2 + KAL_G2 * (input - KAL_Output2)  # 当前 估算值 = 上次 最优值
    KAL_P2 = (1 - KAL_G2) * KAL_P2  # 更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差
    return KAL_Output2

# 转向
def control_turn(zhong2):
    errt = (zhong2 - 64)
    turn_kp = a * abs(errt) + Kpc
    turn = pid_turn(64, zhong2, turn_kp, turn_ki, turn_kd) + Imu.gyro_z * turn_kd2

    return turn

# 角速度环
def angle_speed1(med_gyro, cur_gyro):
    motor = pid_position_1(med_gyro, cur_gyro, angle_kp, angle_ki, angle_kd)
    motor = limit(motor, -4000, 4000)
    return (motor)

# 角度环
def angle(med_roll_angle, cur_roll_angle):
    global angle_1
    angle_1 = pid_position_2(med_roll_angle, cur_roll_angle, roll_angle_Kp, roll_angle_Ki, roll_angle_Kd)
    return (angle_1)

# 速度环
def speed(med_speed, cur_speed):
    global speed_1
    speed_1 = pid_position_3(med_speed, cur_speed, speed_Kp, speed_Ki, speed_Kd)
    return speed_1

# 回调函数1
def time_pit_handler(time):
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2, log_counter  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 10) else (1)  # 计数标注

    if ticker_count % 1 == 0:  # 角速度 1ms 执行一次
        Imu660()  # 陀螺仪解算
        
        # 记录日志
        log_counter += 1
        if log_counter >= LOG_INTERVAL and LOG_ENABLED:
            log_counter = 0
            log_data("IMU", [Imu.Pitch, Imu.gyro_x, Imu.Roll, Imu.gyro_y])

        motor1 = angle_speed1(angle_1, -Imu.gyro_x)
        motor2 = angle_speed1(angle_1, -Imu.gyro_x)

        # 加入死区补偿和平滑处理
        smooth_factor = 0.8  # 原值0.7，增加平滑系数，防止电机输出突变
        # 保存上一次电机输出
        global last_motor1, last_motor2
        if 'last_motor1' not in globals():
            last_motor1 = 0
            last_motor2 = 0
            
        # 平滑处理
        motor1_smooth = smooth_factor * motor1 + (1-smooth_factor) * last_motor1
        motor2_smooth = smooth_factor * motor2 + (1-smooth_factor) * last_motor2
        
        # 电机死区补偿
        motor1_output = motor_output_with_deadband(motor1_smooth)
        motor2_output = motor_output_with_deadband(motor2_smooth)
        
        # 左右电机平衡调整（解决转圈问题）
        motor1_output = motor1_output * LEFT_MOTOR_FACTOR
        
        # 更新上一次电机输出
        last_motor1 = motor1_smooth
        last_motor2 = motor2_smooth

        # 输出到电机
        motor_l.duty(motor1_output)
        motor_r.duty(-motor2_output)
        
    if ticker_count % 5 == 0:  # 角度 5ms 执行一次
        # 更精细的角度调整，加入前进补偿
        forward_angle_offset = 0
        if TARGET_SPEED > 0:  # 前进时需要前倾一定角度
            forward_angle_offset = -FORWARD_OFFSET * abs(TARGET_SPEED) / 10
        elif TARGET_SPEED < 0:  # 后退时需要后倾
            forward_angle_offset = FORWARD_OFFSET * abs(TARGET_SPEED) / 10
            
        angle_1 = angle(med_roll_angle - speed_1 + STATIC_OFFSET + forward_angle_offset, -Imu.Pitch)

    if ticker_count % 10 == 0:  # 速度 10ms 执行一次
        encl_data = encoder_l.get()
        encr_data = encoder_r.get()

        # 添加静止检测逻辑
        raw_avg_speed = (Encoders.KAL_templ_pluse + Encoders.KAL_tempr_pluse) / 2
        
        # 对速度值进行平滑处理，减少突变
        global last_speed_value
        avg_speed = SPEED_SMOOTH_FACTOR * raw_avg_speed + (1-SPEED_SMOOTH_FACTOR) * last_speed_value
        last_speed_value = avg_speed
        
        # 检测左右轮速度差异，记录日志
        speed_diff = Encoders.KAL_templ_pluse - Encoders.KAL_tempr_pluse
        if log_counter == 0 and LOG_ENABLED:
            log_data("SPEED_DIFF", speed_diff)
        
        # 速度控制逻辑
        if TARGET_SPEED == 0:  # 平衡静止模式
            # 改进的静止逻辑：设置更大的静止阈值
            if abs(avg_speed) < STATIC_SPEED_THRESHOLD:  # 使用可配置阈值
                # 在静止模式中，增强角度控制，降低速度控制影响
                speed_target = 0
                speed_value = avg_speed * 0.4  # 原值0.5，进一步降低实际速度敏感度
                
                # 静止模式下减弱速度控制的影响
                speed_1 = speed(speed_target, speed_value) * STATIC_CONTROL_FACTOR
                
                # 清除积分项，防止积分在静止时累积
                if abs(avg_speed) < 3:  # 原值2，略微提高清除积分的条件
                    global err_sum_3
                    err_sum_3 = err_sum_3 * 0.8  # 不立即清零，而是逐渐衰减，避免突变
            else:
                # 正常行驶模式，但对速度的反应更加平滑
                speed_result = speed(0, avg_speed)
                # 对速度控制进行平滑过渡
                speed_1 = speed_1 * 0.7 + speed_result * 0.3  # 缓慢过渡到新的速度控制值
        else:  # 前进/后退模式
            # 计算速度误差
            speed_error = TARGET_SPEED - avg_speed
            
            # 前进/后退模式使用不同的控制参数（可以更积极）
            forward_kp = 0.022  # 比正常平衡模式略高，改善跟踪性能
            forward_ki = 0.00002  # 保持较低积分作用
            forward_kd = 0.006  # 适中的微分作用
            
            # 速度控制计算
            speed_result = pid_position_3(TARGET_SPEED, avg_speed, forward_kp, forward_ki, forward_kd)
            
            # 平滑过渡
            transition_factor = 0.5  # 更快的响应速率，但仍有平滑效果
            speed_1 = speed_1 * (1-transition_factor) + speed_result * transition_factor
            
        if log_counter == 0 and LOG_ENABLED:
            log_data("SPEED", [speed_1, avg_speed, TARGET_SPEED])

# 回调函数2
def time_pit3_handler(time):
    global ticker_flag2, ticker_count2  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag2 = True  # 否则它会新建一个局部变量

    Encoders.KAL_templ_pluse = KalmanFilter(encoder_l.get())
    Encoders.KAL_tempr_pluse = KalmanFilter2(encoder_r.get())



pit1 = ticker(1)
pit3 = ticker(3)
pit1.capture_list(imu)
pit3.capture_list(encoder_l, encoder_r)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
pit3.callback(time_pit3_handler)

# 启动 ticker 实例 参数是触发周期 单位是毫秒
Imu_Init()
pit1.start(1)
pit3.start(10)

log_data("系统初始化完成")

# 串口命令处理函数
def uart_handler():
    global TARGET_SPEED
    if uart2.any():
        cmd = uart2.readline().decode().strip()
        if cmd.startswith('SPD:'):
            try:
                # 设置目标速度，格式: SPD:10 (前进速度10)
                new_speed = float(cmd.split(':')[1])
                TARGET_SPEED = new_speed
                log_data("新目标速度设置为", TARGET_SPEED)
            except:
                log_data("无效的速度命令")
        elif cmd == 'STOP':
            # 紧急停止命令
            TARGET_SPEED = 0
            log_data("紧急停止")

# 主循环
while True:

    if ticker_flag:    
        ticker_flag = False
        
    # 处理可能的串口命令
    uart_handler()
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    if end_switch.value() != end_state:
        pit1.stop()
        # 删除错误的pit2引用
        pit3.stop()
        log_data("Ticker stop.")
        print("Ticker stop.")
        break

    gc.collect()

