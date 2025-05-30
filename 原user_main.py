from machine import *
from display import *
from smartcar import *
from seekfree import *
import gc
import time
import math
import ustruct

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

# ////////////角速度//////////////////////
angle_kp = -1180.0  # 原值-1100.0，增加比例系数以提高响应力
angle_ki = -0.0  # 保持为0，避免积分作用引起的摇摆
angle_kd = -195.0  # 原值-180.0，适当增加微分作用，提供稳定性
# ////////////角度//////////////////////
roll_angle_Kp = 0.04  # 原值0.058，增加比例系数以提高响应力
roll_angle_Ki = 0.00004  # 原值0.00002，适当增加积分作用
roll_angle_Kd = 0.16  # 原值0.14，增加微分作用以提供稳定性
# ////////////速度//////////////////////
speed_Kp = 0.095  # 原值0.085，增加以提高速度控制的影响力
speed_Ki = 0.000012  # 原值0.000008，适当增加积分作用
speed_Kd = 0.015  # 原值0.012，增加以提供更好的阻尼效果
angle_1 = 0
speed_1 = 0
motor1 = 0
motor2 = 0
med_roll_angle = 25
TARGET_SPEED = 10

#################################################编码器卡尔曼滤波
KAL_P = 0.02  # 估算协方差
KAL_G = 0.0  # 卡尔曼增益
KAL_Q = 0.70  # 过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
KAL_R = 200  # 测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
KAL_Output = 0.0  # 卡尔曼滤波器输出

KAL_P2 = 0.02  # 估算协方差
KAL_G2 = 0.0  # 卡尔曼增益
KAL_Q2 = 0.70  # 过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
KAL_R2 = 200  # 测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
KAL_Output2 = 0.0  # 卡尔曼滤波器输出


######################################################
def limit(value, min_value, max_value):
    if value < min_value:
        value = min_value
    elif value > max_value:
        value = max_value
    else:
        value = value
    return value

# 位置式PID控制
def calculate_pid(err, err_sum, err_last, med, value, kp, ki, kd):
    err = med - value
    err_sum += err
    err_x = err - err_last
    pwm = kp * err + ki * err_sum + kd * err_x
    err_last = err
    return pwm


def pid_position_1(med, value, kp, ki, kd):
    global err_1, err_sum_1, err_last_1
    pwm_1 = calculate_pid(err_1, err_sum_1, err_last_1, med, value, kp, ki, kd)
    err_last_1 = err_1
    return pwm_1


def pid_position_2(med, value, kp, ki, kd):
    global err_2, err_sum_2, err_last_2
    pwm_2 = calculate_pid(err_2, err_sum_2, err_last_2, med, value, kp, ki, kd)
    err_last_2 = err_2
    return pwm_2


def pid_position_3(med, value, kp, ki, kd):
    global err_3, err_sum_3, err_last_3
    pwm_3 = calculate_pid(err_3, err_sum_3, err_last_3, med, value, kp, ki, kd)
    err_last_3 = err_3
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
    return 1.0  # / (math.sqrt(x))


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
    if abs(imu_data[3]) < 30 or abs(imu_data[3]) > 30000:
        imu_data[3] = 0
    if abs(imu_data[4]) < 30 or abs(imu_data[4]) > 30000:
        imu_data[4] = 0
    if abs(imu_data[5]) < 30 or abs(imu_data[5]) > 30000:
        imu_data[5] = 0

    Imu.X = int(imu_data[3] / 16.4)
    Imu.Y = int(imu_data[4] / 16.4)  # 俯仰角
    Imu.Z = int(imu_data[5] / 16.4)
    Imu.gyro_x = round((float(imu_data[3]) - Filter_data[0]), 3) * PI / 180 / 16.4
    Imu.gyro_y = round((float(imu_data[4]) - Filter_data[1]), 3) * PI / 180 / 16.4
    Imu.gyro_z = round((float(imu_data[5]) - Filter_data[2]), 3) * PI / 180 / 14.4
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
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 10) else (1)  # 计数标注

    if ticker_count % 1 == 0:  # 角速度 1ms 执行一次
        Imu660()  # 陀螺仪解算
         # 测试用，配置参数后通过vofa 串口打印

        motor1 = angle_speed1(angle_1, -Imu.gyro_x)
        motor2 = angle_speed1(angle_1, -Imu.gyro_x)

        motor_l.duty(motor1)  # 输出
        motor_r.duty(-motor2)
        
    if ticker_count % 5 == 0:  # 角度 5ms 执行一次
        angle_1 = angle(med_roll_angle - speed_1, -Imu.Pitch)

    if ticker_count % 10 == 0:  # 速度 10ms 执行一次
        #encl_data = encoder_l.get()
        #encr_data = encoder_r.get()
        #print("{:>6f}\n{:>6f}".format(Encoders.KAL_templ_pluse, Encoders.KAL_tempr_pluse)) 
        speed_1 = speed(TARGET_SPEED, (Encoders.KAL_templ_pluse + Encoders.KAL_tempr_pluse) / 2)
        #print("{:>6f}\n".format(speed_1))

# 回调函数2
def time_pit3_handler(time):
    global ticker_flag2, ticker_count2  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag2 = True  # 否则它会新建一个局部变量
    global KAL_templ_pluse
    global KAL_tempr_pluse

    Encoders.KAL_templ_pluse = KalmanFilter(encoder_l.get())
    Encoders.KAL_tempr_pluse = KalmanFilter2(encoder_r.get())

def time_pit2_handeler():
    pass

pit1 = ticker(1)
#pit2 = ticker(2)
pit3 = ticker(3)
pit1.capture_list(imu)
pit3.capture_list(encoder_l, encoder_r)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
#pit3.callback(time_pit2_handeler)
pit3.callback(time_pit3_handler)

# 启动 ticker 实例 参数是触发周期 单位是毫秒
Imu_Init()
pit1.start(1)
#pit2.start(5)
pit3.start(10)

# 主循环
while True:

    if ticker_flag:    
        ticker_flag = False
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    if end_switch.value() != end_state:
        pit1.stop()
        pit2.stop()
        pit3.stop()
        print("Ticker stop.")
        break

    gc.collect()
