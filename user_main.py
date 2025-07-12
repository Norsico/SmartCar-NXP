from machine import *
from display import *
from smartcar import *
from seekfree import *
import gc
import time
import math

# wifi开关
wifi_en = True   

# 元素识别开关 - 关闭后只巡线不检测元素
element_en = False  # False: 只巡线，True: 检测元素

MIDDLE_LINE = 64

if wifi_en:
    # WiFi调参初始化
    try:
        wifi = WIFI_SPI("xyh", "1261340160xyh", WIFI_SPI.TCP_CONNECT, "192.168.43.3", "8086")
        wifi.send_str("WiFi parameter tuning ready.\r\n")
        time.sleep_ms(500)
        wifi_enabled = True
        print("WiFi调参模块初始化成功")
    except Exception as e:
        print(e)
        print("WiFi调参模块初始化失败")
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

# 按键相关变量
key = KEY_HANDLER(10)  # 按键扫描周期为10

# CCD算法参数 - 移植自C语言示例
CCD1_SET_WIDTH = 32  # 近端CCD设定宽度
CCD2_SET_WIDTH = 30  # 远端CCD设定宽度

# 阈值参数 - 需要调试
# CCD阈值参数 - 根据参考代码优化
# 梯度检测阈值倍数：控制边界检测灵敏度 (参考值: 20-50)
# - 值越小越灵敏，容易检测到边界但可能误判
# - 值越大越保守，不易误判但可能漏检
THRESHOLD_MULTIPLE_1 = 45  # 近端更灵敏
THRESHOLD_MULTIPLE_2 = 41  # 远端适中

# PID参数 - 进一步增强响应强度
angle_kp = -1700 #测过了 两个都是负的 kd不是正的
angle_ki = 0
angle_kd = -96

roll_angle_Kp = 0.1145 #纯纯脑瘫角度环 调死我了
roll_angle_Ki = 0
roll_angle_Kd = 0 #0.0826 

speed_Kp = 0.34 # 0.063 老铁我发现这东西不能给大 给大了就容易震动了 速度环参数给偏小一点 速度积分也是 跑起来效果就比大的好
speed_Ki = 0 # 添加积分项，消除稳态误差，防止转弯时速度控制不准确
speed_Kd = 0 #0.136 # 1.7 给小了虽然到达预定速度的时间会变长但是到达之后毕竟参数小震荡肯定好点 还是选择稳定好 要速度快可以改预定速度

# 线路跟踪PD控制器参数 - 参考C代码优化
line_kp = 22 # 增大比例系数，提高响应速度（参考C代码舵机控制强度）
line_squart_kp = 0.0202  # 减小平方项系数，避免过度响应
line_kd = 2000 # 适当减小微分系数，减少直线震荡

TARGET_SPEED = 65  # 目标速度
med_roll_angle = 57.1  # 调整平衡角度

# 二值化阈值百分比：控制黑白场景判断 (参考值: 30-60)
# - 用于判断当前区域是否为黑色场景(起跑线、停车区等)
# - 值越小越容易判断为黑色场景
THRESHOLD_1 = 2400      
THRESHOLD_2 = 2500

# 环岛状态定义 - 参考C代码的7阶段状态机
NO_RING = 0            # 无环岛
FIND_RING = 1          # 发现环岛 
FIND_RING_STAGE2 = 7   # 发现环岛第二阶段
READY_IN_RING = 2      # 准备进入环岛
IN_RING = 3            # 在环岛中
READY_OUT_RING = 4     # 准备出环岛
OUT_RING = 5           # 出环岛
READY_NO_RING = 6      # 准备回到无环岛状态

# 环岛相关全局变量
ring_state = NO_RING
ring_left = False
ring_right = False

# 环岛阈值调整
original_threshold_1 = THRESHOLD_MULTIPLE_1  # 保存原始阈值
ring_threshold_1 = 20  # 环岛内部使用的较小阈值

# 环岛各阶段参数 - 需要根据实际测试调整
READY_IN_RING_ENCODER = 32     # 进入环岛前的编码器距离（增大，因为积分值会更大）
IN_RING_ENCODER = 10           # 环岛内部编码器距离
NO_RING_ENCODER = 34          # 出环岛后的编码器距离

# 编码器积分值（用于距离计算）
encoder_integral = 0
ring_encoder = 0        # 环岛编码器计数
ring_angle = 0          # 环岛角度计数

# 编码器低通滤波参数
encoder_filter_alpha = 0.3  # 滤波系数，0-1之间，越小滤波越强
encoder_l_filtered = 0.0    # 左编码器滤波后的值
encoder_r_filtered = 0.0    # 右编码器滤波后的值

# 十字路口相关全局变量
cross_flag = False      # 十字路口标志
cross_encoder = 0       # 十字路口编码器计数
cross_delay_encoder = 0 # 环岛结束后的延时编码器值
cross_middle_line = MIDDLE_LINE  # 检测到十字路口时保存的中线值
pre_cross_flag = False  # 预十字标志
pre_cross_encoder = 0   # 预十字编码器计数

# 十字路口参数 - 需要调试优化
CROSS_ENCODER = 20      # 十字路口编码器距离阈值（参考值）
CROSS_DELAY = 40        # 环岛结束后延时距离，避免误检测
PRE_CROSS_ENCODER = 10  # 预十字编码器距离阈值

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
        self.middle_sideline1 = MIDDLE_LINE
        self.middle_sideline1_last = MIDDLE_LINE
        self.width1 = 0
        
        # CCD1历史边界值
        self.left_sideline1_last = 0
        self.right_sideline1_last = 0
        
        # CCD2(远端)原图像
        self.left_sideline2 = 0
        self.right_sideline2 = 0
        self.middle_sideline2 = MIDDLE_LINE
        self.middle_sideline2_last = MIDDLE_LINE
        self.width2 = 0
        
        # CCD2历史边界值
        self.left_sideline2_last = 0
        self.right_sideline2_last = 0
        
        # 曲率计算
        self.right_qulu = 0.0  # 右边曲率
        self.left_qulu = 0.0  # 左边曲率

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

# 蜂鸣器初始化
beep = Pin('D24', Pin.OUT, pull=Pin.PULL_UP_47K, value=False)

# 蜂鸣器状态标志位
BEEP_OFF = 0
BEEP_SHORT = 1
BEEP_LONG = 2
BEEP_ON = 3
BEEP_DOUBLE_SHORT = 4

beep_state = BEEP_OFF
beep_timer = 0
beep_double_count = 0  # 双响计数器

def beep_on():
    """蜂鸣器响"""
    beep.high()

def beep_off():
    """蜂鸣器停"""
    beep.low()

def set_beep_short():
    """设置短响标志"""
    global beep_state, beep_timer
    beep_state = BEEP_SHORT
    beep_timer = 3  # 短响60ms，20ms*3=60ms

def set_beep_long():
    """设置长响标志"""
    global beep_state, beep_timer
    beep_state = BEEP_LONG
    beep_timer = 30  # 长响600ms，20ms*30=600ms

def set_beep_double_short():
    """设置双短响标志"""
    global beep_state, beep_timer, beep_double_count
    beep_state = BEEP_DOUBLE_SHORT
    beep_timer = 5  # 第一声短响100ms
    beep_double_count = 0  # 重置计数器

def set_beep_off():
    """设置蜂鸣器停止标志"""
    global beep_state
    beep_state = BEEP_OFF

def beep_process():
    """蜂鸣器处理函数 - 在主循环中调用"""
    global beep_state, beep_timer, beep_double_count
    
    if beep_state == BEEP_OFF:
        beep_off()
    elif beep_state == BEEP_SHORT:
        if beep_timer > 0:
            beep_on()
            beep_timer -= 1
        else:
            beep_off()
            beep_state = BEEP_OFF
    elif beep_state == BEEP_LONG:
        if beep_timer > 0:
            beep_on()
            beep_timer -= 1
        else:
            beep_off()
            beep_state = BEEP_OFF
    elif beep_state == BEEP_DOUBLE_SHORT:
        if beep_double_count == 0:  # 第一声短响
            if beep_timer > 0:
                beep_on()
                beep_timer -= 1
            else:
                beep_off()
                beep_double_count = 1
                beep_timer = 2  # 间隔40ms
        elif beep_double_count == 1:  # 间隔
            if beep_timer > 0:
                beep_off()
                beep_timer -= 1
            else:
                beep_double_count = 2
                beep_timer = 3  # 第二声短响60ms
        elif beep_double_count == 2:  # 第二声短响
            if beep_timer > 0:
                beep_on()
                beep_timer -= 1
            else:
                beep_off()
                beep_state = BEEP_OFF
                beep_double_count = 0
    elif beep_state == BEEP_ON:
        beep_on()

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
lcd.mode(0)
# 清屏
lcd.clear(0x0000)

# 保存原始线路跟踪参数
original_line_kp = line_kp
original_line_kd = line_kd

# 环岛线路跟踪参数
ring_line_kp = 11.99   # 环岛内部线路跟踪比例系数
ring_line_kd = 352  # 环岛内部线路跟踪微分系数
# 环岛目标速度
ring_target_speed = 80  # 环岛内部目标速度

# 偏航角速度抑制参数
gyro_z_kd = 0  # 偏航角速度D控制系数，抑制左右摆动


# 控制变量
angle_1 = speed_1 = motor1 = motor2 = 0

# 保存原始目标速度
original_target_speed = TARGET_SPEED

ticker_count = 0
gyro_z_control = 0  # 偏航角速度抑制控制输出

# 中线低通滤波参数
middle_line_filter_alpha = 0.3  # 滤波系数，0-1之间，越小滤波越强
middle_line_filtered = MIDDLE_LINE  # 滤波后的中线值

# 巡线控制输出低通滤波参数
line_output_filter_alpha = 0.4  # 控制输出滤波系数，响应稍快一些
line_output_filtered = 0.0  # 滤波后的控制输出值

# 角速度控制输出低通滤波参数
motor1_filter_alpha = 0.5  # 角速度控制输出滤波系数，0-1之间，越小滤波越强
motor1_filtered = 0.0  # 滤波后的角速度控制输出值

# WiFi调参数据存储 - 将line_kp和line_kd替换为TARGET_SPEED和med_roll_angle
wifi_data = [angle_kp, angle_kd, roll_angle_Kp, roll_angle_Kd, speed_Kp, line_squart_kp, TARGET_SPEED, med_roll_angle]

def update_wifi_parameters():
    """更新WiFi调参数据
    通道0: angle_kp (角速度环比例系数)
    通道1: angle_kd (角速度环微分系数)
    通道2: roll_angle_Kp (角度环比例系数)
    通道3: roll_angle_Kd (角度环微分系数)
    通道4: speed_Kp (速度环比例系数)
    通道5: line_squart_kp (线路跟踪平方项系数)
    通道6: TARGET_SPEED (目标速度)
    通道7: med_roll_angle (平衡角度)
    """
    global angle_kp, angle_kd, roll_angle_Kp, roll_angle_Kd
    global speed_Kp, line_squart_kp, TARGET_SPEED, med_roll_angle
    global wifi_data, motor1, motor2
    global pid_angle_speed, pid_angle, pid_speed, pid_line
    
    if not wifi_enabled:
        return
    
    try:
        # 数据解析
        data_flag = wifi.data_analysis()
        
        # 检查各通道是否有数据更新
        for i in range(8):
            if data_flag[i]:
                wifi_data[i] = wifi.get_data(i)
        
        # 更新参数
        angle_kp = wifi_data[0]          # 角速度环比例系数
        angle_kd = wifi_data[1]          # 角速度环微分系数
        roll_angle_Kp = wifi_data[2]     # 角度环比例系数
        roll_angle_Kd = wifi_data[3]     # 角度环微分系数
        speed_Kp = wifi_data[4]          # 速度环比例系数
        line_squart_kp = wifi_data[5]    # 线路跟踪平方项系数
        TARGET_SPEED = wifi_data[6]      # 目标速度
        med_roll_angle = wifi_data[7]    # 平衡角度
        
        # 更新PID控制器参数
        pid_angle_speed.kp = angle_kp
        pid_angle_speed.kd = angle_kd
        pid_angle.kp = roll_angle_Kp
        pid_angle.kd = roll_angle_Kd
        pid_speed.kp = speed_Kp
        pid_line.kp_squart = line_squart_kp
        
        # 发送示波器数据 - 显示角度和速度相关信息
        # 计算当前速度（编码器平均值）
        current_speed = -(kalman_l.output + kalman_r.output) / 2
        wifi.send_oscilloscope(
            current_speed, pid_speed.err_sum, TARGET_SPEED, imu_data_obj.Pitch)
    
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
        # 限制积分项，防止积分饱和
        self.err_sum = max(-1000, min(1000, self.err_sum))
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
    
# D控制器类（用于偏航角速度抑制）
class DController:
    def __init__(self, kd):
        self.kd = kd
        self.last_value = 0.0
    
    def update(self, current_value):
        # D控制：输出与输入变化率成正比
        diff = current_value - self.last_value
        output = -self.kd * diff  # 负号表示抑制变化
        self.last_value = current_value
        return output

# PD控制器类（用于线路跟踪）
class LinePDController:
    def __init__(self, kp, kd, kp_squart):
        self.kp = kp
        self.kp_squart = kp_squart
        self.kd = kd
        self.err_last = 0
    
    def update(self, setpoint, current):
        err = setpoint - current
        err_diff = err - self.err_last
        
        # 基础PD控制
        output = self.kp * err + self.kd * err_diff + self.kp_squart * err * abs(err)
        
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
pid_line = LinePDController(line_kp, line_kd, line_squart_kp)  # 线路跟踪PD控制器
gyro_z_controller = DController(gyro_z_kd)  # 偏航角速度抑制控制器
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
    alpha = 0.3  # 进一步降低滤波系数，增加平滑性
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
    imu_data_obj.Pitch = 90 + math.atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1,
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
    
    # 注释：简化版本不需要陀螺仪累积角度
    # global angle_gz
    # angle_gz = imu_data_obj.Total_Yaw

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
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2, imu_data, line_control_output, gyro_z_control
    global motor1_filtered, motor1_filter_alpha
    ticker_flag = True
    ticker_count = (ticker_count + 1) % 10

    # 1ms: 角速度控制
    imu_data = imu.get()
    imu_process()
    
    # 角速度控制输出
    motor1_raw = pid_angle_speed.update(angle_1, imu_data_obj.gyro_x)
    
    # 对角速度控制输出进行低通滤波
    motor1_filtered = motor1_filter_alpha * motor1_raw + (1 - motor1_filter_alpha) * motor1_filtered
    motor1 = motor1_filtered

    # 偏航角速度抑制控制
    gyro_z_control = gyro_z_controller.update(imu_data_obj.gyro_z)
    
    motor2 = motor1
    # CCD巡线控制
    motor1 += line_control_output  # 左电机增加转向控制
    motor2 -= line_control_output  # 右电机减少转向控制
    
    # 偏航角速度抑制控制（双电机差速）
    motor1 += gyro_z_control  # 左电机增加偏航抑制
    motor2 -= gyro_z_control  # 右电机减少偏航抑制
    
    motor1 = limit(motor1, -8888, 8888)  # 增加电机输出限制，提高响应强度
    motor2 = limit(motor2, -8888, 8888)  # 增加电机输出限制，提高响应强度
    
    motor_l.duty(-motor1)
    motor_r.duty(-motor2)
    
    # 5ms: 角度控制
    if ticker_count % 2 == 0:
        angle_1 = pid_angle.update(med_roll_angle - speed_1, imu_data_obj.Pitch)
    
    # 10ms: 速度控制
    if ticker_count % 5 == 0:
        avg_speed = -(kalman_l.output + kalman_r.output) / 2
        speed_1 = pid_speed.update(TARGET_SPEED, avg_speed)
        speed_1 = limit(speed_1, -10, 10)  # 限制角度偏移

def encoder_update(timer):
    global encoder_integral, encoder_l_filtered, encoder_r_filtered, encoder_filter_alpha
    
    # 获取原始编码器数据（每个周期的脉冲数）
    encoder_l_raw = encoder_l.get()
    encoder_r_raw = encoder_r.get()
    
    # 编码器低通滤波
    encoder_l_filtered = encoder_filter_alpha * encoder_l_raw + (1 - encoder_filter_alpha) * encoder_l_filtered
    encoder_r_filtered = encoder_filter_alpha * encoder_r_raw + (1 - encoder_filter_alpha) * encoder_r_filtered
    
    # 使用滤波后的编码器值
    kalman_l.output = encoder_l_filtered
    kalman_r.output = encoder_r_filtered
    
    # 计算平均脉冲数（参考C代码逻辑）
    # encoder = (encoder_L + encoder_R) * 0.5
    avg_encoder = (abs(encoder_l_filtered) + abs(encoder_r_filtered)) * 0.5
    
    # 积分计算距离（参考C代码：encoder_integral += encoder * 0.02）
    # 这里编码器值就是脉冲数，直接乘以时间周期进行积分
    distance_increment = avg_encoder * 0.005  # 5ms定时器周期
    encoder_integral += distance_increment

def ccd_image_init():
    """CCD图像初始化"""
    global Trk, CCD1, CCD2
    Trk.middle_sideline1 = MIDDLE_LINE
    Trk.middle_sideline2 = MIDDLE_LINE
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
    
    # 二值化阈值计算 - 直接使用THRESHOLD_1作为固定阈值
    CCD1.bin_thrd = THRESHOLD_1
    
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
    
    # 计算中心区域平均值 (20-100)
    count = 0
    total = 0
    for i in range(20, min(100, len(ccd_data))):
        total += ccd_data[i]
        count += 1
    
    if count > 0:
        CCD2.aver = total // count
    
    # 二值化阈值计算 - 直接使用THRESHOLD_2作为固定阈值
    CCD2.bin_thrd = THRESHOLD_2
    
    # 动态阈值计算
    if CCD2.max_val + CCD2.min_val > 0:
        CCD2.threshold = ((CCD2.max_val - CCD2.min_val) * 100 * THRESHOLD_MULTIPLE_2) // ((CCD2.max_val + CCD2.min_val) * 100)

def left_right_sideline(ccd_data1, ccd_data2):
    """左右边界检测 - 移植自C语言核心算法"""
    global Trk, CCD1, CCD2, CCD1_left_flag, CCD1_right_flag, CCD2_left_flag, CCD2_right_flag
    global black_write_1, black_write_2
    
    if not ccd_data1 or not ccd_data2:
        return
    
    # 保存上次中线位置和边界位置
    Trk.middle_sideline1_last = Trk.middle_sideline1
    Trk.middle_sideline2_last = Trk.middle_sideline2
    Trk.left_sideline1_last = Trk.left_sideline1
    Trk.right_sideline1_last = Trk.right_sideline1
    Trk.left_sideline2_last = Trk.left_sideline2
    Trk.right_sideline2_last = Trk.right_sideline2
    
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
    # 单边丢失补偿 - 参考C代码的智能搜索策略
    if CCD1_left_flag and not CCD1_right_flag:
        # 左边有效，右边丢线，从左边界向右搜索
        # 使用更严格的搜索条件，避免误判
        for i in range(Trk.left_sideline1 + 5, min(122, len(ccd_data1))):
            if i + 5 < len(ccd_data1) and (ccd_data1[i] + ccd_data1[i+5]) > 0:
                gradient = abs(ccd_data1[i] - ccd_data1[i+5]) * 100 // (ccd_data1[i] + ccd_data1[i+5])
                # 提高阈值，避免噪点干扰
                if gradient > CCD1.threshold * 1.2 and ccd_data1[i] > ccd_data1[i+5]:
                    Trk.right_sideline1 = i
                    CCD1_right_flag = True
                    break
    
    elif not CCD1_left_flag and CCD1_right_flag:
        # 右边有效，左边丢线，从右边界向左搜索
        # 同样使用更严格的搜索条件
        for i in range(Trk.right_sideline1 - 5, 4, -1):
            if i >= 5 and (ccd_data1[i] + ccd_data1[i-5]) > 0:
                gradient = abs(ccd_data1[i] - ccd_data1[i-5]) * 100 // (ccd_data1[i] + ccd_data1[i-5])
                # 提高阈值，避免噪点干扰
                if gradient > CCD1.threshold * 1.2 and ccd_data1[i] > ccd_data1[i-5]:
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
    
    center = MIDDLE_LINE  # 图像中心
    
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
    """中线计算 - 包含环岛特殊处理逻辑"""
    global Trk, CCD1_left_flag, CCD1_right_flag, CCD2_left_flag, CCD2_right_flag
    global ring_state, ring_left, ring_right, cross_flag
    
    # CCD2中线计算
    if CCD2_left_flag and CCD2_right_flag:
        # 双边都有效，正常计算
        Trk.middle_sideline2 = (Trk.left_sideline2 + Trk.right_sideline2) / 2.0
    elif CCD2_left_flag and not CCD2_right_flag:
        Trk.middle_sideline2 = (Trk.left_sideline2 + 127) / 2.0
    elif not CCD2_left_flag and CCD2_right_flag:
        Trk.middle_sideline2 = (0 + Trk.right_sideline2) / 2.0
    else:
        pass  # Trk.middle_sideline2保持不变

    # 基础中线计算
    # CCD1中线计算
    if CCD1_left_flag and CCD1_right_flag:
        # 双边都有效，正常计算
        Trk.middle_sideline1 = (Trk.left_sideline1 + Trk.right_sideline1) / 2.0 
    elif CCD1_left_flag and not CCD1_right_flag:
        # 左边有效，右边丢线，使用上次右边界值计算中线
        Trk.middle_sideline1 = (Trk.left_sideline1 + Trk.right_sideline1) / 2.0
    elif not CCD1_left_flag and CCD1_right_flag:
        # 右边有效，左边丢线，使用上次左边界值计算中线
        Trk.middle_sideline1 = (Trk.left_sideline1_last +Trk.right_sideline1) / 2.0
    else:
        # 近端CCD双边都丢线，检查远端CCD是否有边界
        if CCD2_left_flag and CCD2_right_flag:
            # 远端CCD双边都有效，使用远端中线
            Trk.middle_sideline1 = (Trk.left_sideline2 + Trk.right_sideline2) / 2.0
        elif CCD2_left_flag and not CCD2_right_flag:
            # 远端CCD左边有效，右边丢线
            Trk.middle_sideline1 = (Trk.left_sideline2 + Trk.right_sideline2_last) / 2.0
        elif not CCD2_left_flag and CCD2_right_flag:
            # 远端CCD右边有效，左边丢线
            Trk.middle_sideline1 = (Trk.left_sideline2_last + Trk.right_sideline2) / 2.0
        else:
            # 远端CCD也双边丢线，保持上次中线值
            pass  # Trk.middle_sideline1保持不变
    
    # 宽度计算
    Trk.width1 = Trk.right_sideline1 - Trk.left_sideline1
    Trk.width2 = Trk.right_sideline2 - Trk.left_sideline2
    
    # 元素识别关闭时，跳过所有特殊中线处理，只使用基础中线
    if not element_en:
        return
    
    # # 十字路口中线特殊处理
    # # 十字路口期间，根据边界情况选择循迹策略
    # if cross_flag:
    #     # 检查远端和近端是否都有边界
    #     if CCD1_left_flag and CCD1_right_flag and CCD2_left_flag and CCD2_right_flag:
    #         # 远端和近端都有边界，使用平均中线来循迹
    #         Trk.middle_sideline1 = (Trk.middle_sideline1 + Trk.middle_sideline2) / 2.0
    #     else:
    #         # 没有四边界情况，使用近端中线
    #         # 这里已经在基础中线计算中完成了，不需要额外处理
    #         pass
    
    # 环岛中线特殊处理 - 参考C代码逻辑
    # 左环岛处理
    if ring_left:
        if ring_state == FIND_RING_STAGE2:
            Trk.middle_sideline1 = Trk.right_sideline1 - 26
        elif ring_state == FIND_RING:
            Trk.middle_sideline1 = Trk.right_sideline1 - 20
        elif ring_state == READY_IN_RING or ring_state == IN_RING :
            # 环岛内部阶段：按左边缘循迹
            if CCD1_left_flag:
                # 有左边界时，沿左边缘行驶（偏移量设为正值，让小车靠近左边界）
                Trk.middle_sideline1 = Trk.left_sideline1 + 34
            else:
                # 左边界丢失时，使用上次左边界位置
                Trk.middle_sideline1 = Trk.left_sideline1_last + 28
        elif ring_state == READY_OUT_RING:
            if CCD1_left_flag:
                # 有左边界时，沿左边缘行驶（偏移量设为正值，让小车靠近左边界）
                Trk.middle_sideline1 = Trk.left_sideline1 + 20
            else:
                # 左边界丢失时，使用上次左边界位置
                Trk.middle_sideline1 = Trk.left_sideline1_last + 23
        elif ring_state == OUT_RING:
            Trk.middle_sideline1 = Trk.right_sideline1 - 26
    
    # # 右环岛处理
    # elif ring_right:
    #     if ring_state == FIND_RING_STAGE2:
    #         # 进入环岛阶段：基于左边界偏移计算中线
    #         Trk.middle_sideline1 = Trk.left_sideline1 + 44
    #     elif ring_state == READY_IN_RING or ring_state == IN_RING or ring_state == READY_OUT_RING:
    #         # 环岛内部阶段：按右边缘循迹
    #         if CCD1_right_flag:
    #             # 有右边界时，使用右边界偏移
    #             Trk.middle_sideline1 = Trk.right_sideline1 - 30
    #         else:
    #             # 右边界丢失时，使用上次右边界位置
    #             Trk.middle_sideline1 = Trk.right_sideline1_last - 30

    #     elif ring_state == OUT_RING or ring_state == READY_OUT_RING:
    #         # 出环岛阶段：按近端CCD1左边界巡线
    #         if CCD1_left_flag:
    #             # 有左边界时，按左边界偏移计算中线
    #             Trk.middle_sideline1 = Trk.left_sideline1 + 44
    #         else:
    #             # 左边界也丢失时，保持上次中线
    #             pass
    
    # 如果是十字路口，使用CCD2的中线（这里可以根据需要添加十字处理）
    # if cross_flag:
    #     Trk.middle_sideline1 = Trk.middle_sideline2

def ring_detection():
    """
    环岛检测
    """
    global ring_state, ring_left, ring_right
    global CCD1_left_flag, CCD1_right_flag, CCD2_left_flag, CCD2_right_flag
    global black_write_1, black_write_2
    global ring_encoder, ring_angle, encoder_integral
    global imu_data_obj  # 使用IMU数据
    
    if ring_state == NO_RING:
        # 检测左环岛 - 阶段1：远端左侧丢线，近端左侧不丢线
        # 条件：远端CCD左侧丢线 + 近端CCD左侧不丢线 
        if (CCD1_right_flag and CCD2_right_flag and (not CCD2_left_flag) and CCD1_left_flag 
             and Trk.right_qulu <= 15 and (Trk.right_sideline2-Trk.left_sideline2)>60
             and Trk.middle_sideline2<70 ):
            ring_encoder = encoder_integral  # 记录发现环岛时的编码器值
            ring_state = FIND_RING
            ring_left = True
            ring_right = False
            set_beep_short()  # 发现环岛：短响一声
            
            
        # # 检测右环岛 - 阶段1：远端右侧丢线，近端右侧不丢线
        # # 条件：远端CCD右侧丢线 + 近端CCD右侧不丢线
        # elif (CCD1_left_flag and CCD2_left_flag and not CCD2_right_flag and CCD1_right_flag):
        #     ring_state = FIND_RING
        #     ring_left = False
        #     ring_right = True
        #     set_beep_short()  # 发现环岛：短响一声
        #     ring_encoder = encoder_integral  # 记录发现环岛时的编码器值
            
    elif ring_state == FIND_RING and ring_left:
        # 阶段1→2：左环岛确认第二阶段
        # 条件2：近端左侧丢线，远端左侧不丢线
        if (CCD1_right_flag and CCD2_right_flag and CCD2_left_flag and (not CCD1_left_flag)
             and Trk.right_qulu <= 15 
             and Trk.middle_sideline2<70):
            # 记录第二阶段完成的编码器值
            ring_encoder = encoder_integral
            ring_state = FIND_RING_STAGE2
            set_beep_short()  # 第二阶段完成：短响一声
            
        elif abs(ring_encoder - encoder_integral) >= 10 or Trk.right_qulu >= 18:  # 如果走了太远还没满足条件，可能是误判
            ring_state = NO_RING
            ring_left = False
            # # 恢复近端CCD原始阈值
            # global THRESHOLD_MULTIPLE_1
            # THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    # elif ring_state == FIND_RING and ring_right:
    #     # 阶段1→2：右环岛确认第二阶段
    #     # 条件2：近端右侧丢线，远端右侧不丢线

    #     if (not CCD1_right_flag and CCD2_right_flag and Trk.left_qulu <= 22):
    #         # 记录第二阶段完成的编码器值
    #         ring_encoder = encoder_integral
    #         ring_state = FIND_RING_STAGE2
    #         set_beep_short()  # 第二阶段完成：短响一声
            
    #     elif abs(ring_encoder - encoder_integral) >= 10 or Trk.left_qulu > 25:  # 如果走了太远还没满足条件，可能是误判
    #         ring_state = NO_RING
    #         ring_right = False
    #         # 恢复近端CCD原始阈值
    #         global THRESHOLD_MULTIPLE_1
    #         THRESHOLD_MULTIPLE_1 = original_threshold_1
    #     elif (not CCD2_right_flag and CCD1_right_flag and Trk.left_qulu <= 22):
    #         ring_state = FIND_RING
    #         ring_left = False
    #         ring_right = True
    #         set_beep_short()  # 发现环岛：短响一声
    #         ring_encoder = encoder_integral  # 记录发现环岛时的编码器值
            
    elif ring_state == FIND_RING_STAGE2 and ring_left:
        # 阶段2→3：左环岛确认第三阶段
        # 条件3：远端左侧丢线（入环标志）
        if ((not CCD2_left_flag) and CCD1_left_flag and CCD2_right_flag and CCD1_right_flag 
            and Trk.right_qulu <= 15 
            and Trk.middle_sideline2<70):
            ring_angle = imu_data_obj.Yaw  # 记录进入环岛时的角度
            ring_encoder = encoder_integral
            ring_state = READY_IN_RING
            set_beep_short()  # 确认环岛：短响
            # 只降低近端CCD1阈值，提高边界检测灵敏度
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = ring_threshold_1
            # 修改线路跟踪参数为环岛专用参数
            # line_kp = ring_line_kp
            # line_kd = ring_line_kd
            # pid_line.kp = line_kp
            # pid_line.kd = line_kd
            
        elif abs(ring_encoder - encoder_integral) >= 10 or Trk.right_qulu >= 20:  # 如果走了太远还没满足条件，可能是误判
            ring_state = NO_RING
            ring_left = False
            # # 恢复近端CCD原始阈值
            # global THRESHOLD_MULTIPLE_1
            # THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    # elif ring_state == FIND_RING_STAGE2 and ring_right:
    #     # 阶段2→3：右环岛确认第三阶段
    #     # 条件3：远端右侧丢线（入环标志）
        
    #     if not CCD2_right_flag and CCD1_right_flag and Trk.left_qulu <= 25:
    #         ring_state = READY_IN_RING
    #         set_beep_short()  # 确认环岛：短响
    #         # 只降低近端CCD1阈值，提高边界检测灵敏度
    #         global THRESHOLD_MULTIPLE_1, line_kp, line_kd, pid_line
    #         THRESHOLD_MULTIPLE_1 = ring_threshold_1
    #         # 修改线路跟踪参数为环岛专用参数
    #         ring_encoder = encoder_integral
    #         line_kp = ring_line_kp
    #         line_kd = ring_line_kd
    #         pid_line.kp = line_kp
    #         pid_line.kd = line_kd
            
    #     elif abs(ring_encoder - encoder_integral) >= 25 or Trk.left_qulu > 25:  # 如果走了太远还没满足条件，可能是误判
    #         ring_state = NO_RING
    #         ring_right = False
    #         # 恢复近端CCD1原始阈值
    #         global THRESHOLD_MULTIPLE_1
    #         THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    elif ring_state == READY_IN_RING and ring_left:
        # 阶段2→3：准备进入环岛 -> 在环岛中
        # 条件：编码器距离足够（走了足够远开始执行环岛策略）
        if ring_left and abs(ring_encoder - encoder_integral) > 15:
            # ring_encoder = encoder_integral  # 记录进入环岛时的角度
            ring_state = IN_RING
            set_beep_long()  # 进入环岛：长响一声
            # 修改目标速度为环岛专用速度，并清零速度积分项避免突变
            global TARGET_SPEED
            TARGET_SPEED = ring_target_speed
            # pid_speed.err_sum = 0  # 清零积分项，避免速度切换时的冲击
        # elif ring_right and abs(ring_encoder - encoder_integral) > 35:
        #     ring_state = IN_RING
        #     set_beep_long()  # 进入环岛：长响一声
        #     # 修改目标速度为环岛专用速度，并清零速度积分项避免突变
        #     global TARGET_SPEED
        #     TARGET_SPEED = ring_target_speed
        #     pid_speed.err_sum = 0  # 清零积分项，避免速度切换时的冲击
        #     ring_angle = imu_data_obj.Yaw  # 记录进入环岛时的角度
        #     ring_encoder = encoder_integral
        # elif abs(ring_encoder - encoder_integral) > 60:   
        #     ring_state = NO_RING
        #     ring_left = False
        #     ring_right = False
        #     global THRESHOLD_MULTIPLE_1, line_kp, line_kd, pid_line, TARGET_SPEED
        #     # 恢复近端CCD1原始阈值和线路跟踪参数
        #     THRESHOLD_MULTIPLE_1 = original_threshold_1
        #     line_kp = original_line_kp
        #     line_kd = original_line_kd
        #     pid_line.kp = line_kp
        #     pid_line.kd = line_kd
        #     TARGET_SPEED = original_target_speed
                
    elif ring_state == IN_RING and ring_left:
        # 阶段3→4：在环岛中 -> 准备出环岛
        # 条件：使用角度Roll判断是否转过足够角度
        if ((Trk.right_sideline2-Trk.left_sideline2)>60 and (Trk.right_sideline1-Trk.left_sideline1)>60):  # 可调整角度阈值
            ring_encoder = encoder_integral
            # 恢复近端CCD1原始阈值
            # global THRESHOLD_MULTIPLE_1
            # THRESHOLD_MULTIPLE_1 = original_threshold_1
            ring_state = READY_OUT_RING
            # ring_angle = imu_data_obj.Yaw  # 重新记录角度值用于出环岛阶段
            # ring_encoder = encoder_integral

                
    elif ring_state == READY_OUT_RING and ring_left:
#         if abs(ring_angle - imu_data_obj.Yaw) > 66 and CCD1_right_flag:  # 可调整角度阈值
        if abs(ring_encoder - encoder_integral) > 12:
            # 阶段4→5：准备出环岛 -> 出环岛
            # 条件：近端CCD1重新检测到边界（出环岛开始）
                # 左环岛：检测到右边界表示开始出环岛
            # ring_angle = imu_data_obj.Yaw  # 重新记录角度值用于最终阶段
            ring_encoder = encoder_integral
            ring_state = OUT_RING   
            set_beep_long()  # 出环岛：长响一声
            # elif ring_right:
            #     # 右环岛：检测到左边界表示开始出环岛
            #     ring_state = OUT_RING
            #     ring_angle = imu_data_obj.Yaw  # 重新记录角度值用于最终阶段
            #     set_beep_long()  # 出环岛：长响一声
            #     ring_encoder = encoder_integral

                
    elif ring_state == OUT_RING and ring_left:
        # 阶段5→6：出环岛 -> 准备回到无环岛
        # 条件：编码器距离足够（出环岛后走了足够远）
        if abs(ring_encoder - encoder_integral) > 10:
            ring_state = READY_NO_RING
            # 恢复近端CCD1原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    elif ring_state == READY_NO_RING and ring_left:
        # 阶段6→0：准备回到无环岛 -> 无环岛
        # 直接清除所有环岛标志
        ring_state = NO_RING
        ring_left = False
        ring_right = False
        ring_encoder = 0
        ring_angle = 0
        global TARGET_SPEED
        TARGET_SPEED = original_target_speed
        set_beep_off()  # 停止蜂鸣器
        
        # 恢复线路跟踪参数和目标速度
        # global line_kp, line_kd, pid_line, 
        # line_kp = original_line_kp
        # line_kd = original_line_kd
        # pid_line.kp = line_kp
        # pid_line.kd = line_kd
        # pid_speed.err_sum = 0  # 清零积分项，避免速度切换时的冲击
        # # 记录环岛结束时的编码器值，用于延时
        # global cross_delay_encoder
        # cross_delay_encoder = encoder_integral
        



def cross_detection():
    """十字路口检测 - 移植参考代码的检测策略"""
    global cross_flag, cross_encoder, encoder_integral
    global pre_cross_flag, pre_cross_encoder
    global CCD1_left_flag, CCD1_right_flag, CCD2_left_flag, CCD2_right_flag
    global ring_state, ring_left, ring_right
    global cross_delay_encoder, black_write_2
    
    
    # 预十字检测条件：
    # 1. 当前没有环岛状态
    # 2. 远端CCD双边都丢线且不是黑色场景
    # 3. 近端CCD双边都有线
    # 4. 当前没有十字路口标志和预十字标志
    if (ring_state == NO_RING and (not black_write_2) and
        (CCD1_left_flag) and (CCD1_right_flag) and
        abs(Trk.right_sideline1 - Trk.left_sideline1) < 90 and (not cross_flag) and (not pre_cross_flag)):
        # 检测到预十字
        pre_cross_flag = True
        pre_cross_encoder = encoder_integral  # 记录预十字检测时的编码器值
        
    # 预十字状态处理
    if pre_cross_flag:
        # 检查近端CCD宽度是否大于100
        width1 = Trk.right_sideline1 - Trk.left_sideline1
        
        if abs(width1) > 100:
            # 宽度大于100，置为发现十字，开始十字路口模式
            pre_cross_flag = False  # 清除预十字标志
            cross_flag = True       # 设置十字路口标志
            set_beep_short()        # 十字路口：短响
            
            # 保存检测到十字路口时的中线值，后续直行使用
            global cross_middle_line
            cross_middle_line = Trk.middle_sideline1
            
            # 记录当前编码器值
            cross_encoder = encoder_integral
            
        elif abs(pre_cross_encoder - encoder_integral) > PRE_CROSS_ENCODER:
            # 编码器积分大于预设值且没有发现十字，清除预十字标志
            pre_cross_flag = False
            pre_cross_encoder = 0
        
    # 十字路口退出条件：
    # 走过足够距离后结束十字路口
    if cross_flag and abs(cross_encoder - encoder_integral) > 8:
        # 退出十字路口状态
        cross_flag = False
        set_beep_off()  # 停止蜂鸣器
        
        # 重置编码器计数
        cross_encoder = 0
        
        # 重置保存的中线值
        global cross_middle_line
        cross_middle_line = MIDDLE_LINE

def element_detection():
    """元素检测主函数 - 直接使用边界检测算法结果"""
    # 检查元素识别开关
    if not element_en:
        return  # 元素识别关闭，直接返回
    
    # 环岛检测和处理 - 只有在非十字路口状态时才进行
    if not cross_flag:  # 十字路口期间不检测环岛
        ring_detection()
    
def ccd_processing(ccd_data1, ccd_data2):
    """CCD主处理函数 - 移植自C语言示例，优化巡线控制"""
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
    
    # 6.偏差计算
    center = MIDDLE_LINE # 赛道中心
    
    # 计算近端和远端偏差
    deviation1 = Trk.middle_sideline1 - center  # 近端偏差（当前位置）
    deviation2 = Trk.middle_sideline2 - center  # 远端偏差（前瞻位置）
    
    # 改进的CCD控制策略
    if CCD1_left_flag or CCD1_right_flag:
        # 远端边界不全或者只有近端CCD有边界，使用近端CCD控制
        deviation = deviation1
    else:
        # 近端CCD失效，保持上次偏差（添加衰减避免失控）
        global line_deviation
        deviation = line_deviation * 0.95  # 逐渐衰减，避免持续偏移
    
    # 偏差限制 - 参考舵机控制的限制策略
    deviation = max(-70, min(70, deviation))
    
    return deviation
# 全局CCD数据变量 - 用于主循环显示
ccd_data_upper = None
ccd_data_lower = None

def ccd_process(timer):
    """CCD数据处理函数，独立定时器运行 - 只处理CCD算法，不包含显示"""
    global ccd_ticker_flag, ccd_ticker_count, line_deviation, line_control_output
    global key, ccd_data_upper, ccd_data_lower
    
    ccd_ticker_flag = True
    ccd_ticker_count = (ccd_ticker_count + 1) % 100
    
    # CCD数据处理
    try:
        # 读取双CCD数据 - 注意：近端ccd.get(1)，远端ccd.get(0)
        ccd_data_upper = ccd.get(0)  # 远端CCD
        ccd_data_lower = ccd.get(1)  # 近端CCD
        
        # 使用新的CCD处理算法 - 参数顺序：近端，远端
        if ccd_data_upper or ccd_data_lower:
            new_deviation = ccd_processing(ccd_data_lower, ccd_data_upper)
            
            # 只使用近端CCD巡线
            center = MIDDLE_LINE
            if CCD1_left_flag or CCD1_right_flag:
                # 近端CCD有边界，使用近端CCD巡线
                deviation1 = Trk.middle_sideline1 - center
                line_deviation = deviation1
            else:
                # 近端CCD失效，保持上次偏差并逐渐衰减
                line_deviation *= 0.95  # 逐渐衰减，避免持续偏移
                
            # 使用PD控制器计算线路控制输出 - 参考C代码的控制逻辑
            raw_control_output = pid_line.update(0, line_deviation)  # 目标偏差为0
            
            # 限制线路控制输出
            raw_control_output = limit(raw_control_output, -6666, 6666)
            
            # 对控制输出进行低通滤波，减少电机控制突变
            global line_output_filtered, line_output_filter_alpha
            line_output_filtered = line_output_filter_alpha * raw_control_output + (1 - line_output_filter_alpha) * line_output_filtered
            
            # 将滤波后的值赋给最终的控制输出
            line_control_output = line_output_filtered
            
    except Exception as e:
        # 发生错误时逐渐减小控制输出，避免突然停止
        line_control_output *= 0.9

# 初始化定时器
pit1 = ticker(1)
pit3 = ticker(3)
pit2 = ticker(2)  # CCD处理定时器
pit1.capture_list(imu)
pit3.capture_list(encoder_l, encoder_r)
pit2.capture_list(ccd, key)  # CCD定时器捕获CCD和按键
pit1.callback(control_loop)
pit3.callback(encoder_update)
pit2.callback(ccd_process)  # CCD处理回调

# 启动系统
imu_init()
ccd_image_init()  # 初始化CCD图像处理
pit1.start(1)
pit3.start(5)
pit2.start(4)  # CCD

# 系统启动完成
print("init")

# 显示更新计数器 - 控制显示更新频率
display_counter = 0

# 主循环
while True:
    if ticker_flag:
        ticker_flag = False
    
    if ccd_ticker_flag:
        ccd_ticker_flag = False
    
    # 蜂鸣器处理 - 不阻塞主循环
    beep_process()

    # WiFi调参更新
    update_wifi_parameters()
    
    current_speed = -(kalman_l.output + kalman_r.output) / 2
    
    # 显示屏更新 - 降低更新频率，避免影响主循环性能
    display_counter = (display_counter + 1) % 2  # 每2次循环更新一次显示
    if display_counter == 0:
        try:
            # 显示远端CCD (CCD0) 在屏幕上半部分
            if ccd_data_upper:
                lcd.wave(0, 0, 128, 96, ccd_data_upper, max=4095)
            
            # 显示近端CCD (CCD1) 在屏幕下半部分
            if ccd_data_lower:
                lcd.wave(0, 96, 128, 96, ccd_data_lower, max=4095)
                
            # 显示边界线和中线 - 基于原始边界检测算法结果
            # ===== 远端CCD (上半部分) 的边界线和中线 =====
            # 画远端CCD左边界线 (红色) - 只有未丢线才显示
            if CCD2_left_flag and 0 <= Trk.left_sideline2 <= 127:
                lcd.line(Trk.left_sideline2, 0, Trk.left_sideline2, 24, color=0xF800, thick=2)
            
            # 画远端CCD右边界线 (红色) - 只有未丢线才显示
            if CCD2_right_flag and 0 <= Trk.right_sideline2 <= 127:
                lcd.line(Trk.right_sideline2, 0, Trk.right_sideline2, 24, color=0xF800, thick=2)
            
            # 画远端CCD中线 (绿色) - 始终显示
            if 0 <= int(Trk.middle_sideline2) <= 127:
                lcd.line(int(Trk.middle_sideline2), 0, int(Trk.middle_sideline2), 24, color=0x07E0, thick=2)
            
            # ===== 近端CCD (下半部分) 的边界线和中线 =====
            # 画近端CCD左边界线 (红色) - 只有未丢线才显示
            if CCD1_left_flag and 0 <= Trk.left_sideline1 <= 127:
                lcd.line(Trk.left_sideline1, 96, Trk.left_sideline1, 120, color=0xF800, thick=2)
            
            # 画近端CCD右边界线 (红色) - 只有未丢线才显示
            if CCD1_right_flag and 0 <= Trk.right_sideline1 <= 127:
                lcd.line(Trk.right_sideline1, 96, Trk.right_sideline1, 120, color=0xF800, thick=2)
            
            # 画近端CCD中线 (绿色) - 始终显示
            if 0 <= int(Trk.middle_sideline1) <= 127:
                lcd.line(int(Trk.middle_sideline1), 96, int(Trk.middle_sideline1), 120, color=0x07E0, thick=2)
            
                    # 第1行：近端边界位置
            lcd.str12(0, 195, f"L1:{Trk.left_sideline1:3d} R1:{Trk.right_sideline1:3d} M1:{Trk.middle_sideline1:4.1f}", 0xFFFF)
            
            # 第2行：远端边界位置和偏差
            lcd.str12(0, 207, f"L2:{Trk.left_sideline2:3d} R2:{Trk.right_sideline2:3d} M2:{Trk.middle_sideline2:4.1f}", 0xFFFF)
            
            # 第3行：两侧曲率和赛道宽度 (重点显示)
            # 计算宽度时使用原始边界检测结果
            left_width1 = abs(Trk.middle_sideline1 - Trk.left_sideline1) if CCD1_left_flag else 0
            right_width1 = abs(Trk.right_sideline1 - Trk.middle_sideline1) if CCD1_right_flag else 0
            lcd.str12(0, 219, f"QL:{Trk.left_qulu:4.1f} QR:{Trk.right_qulu:4.1f} W:{left_width1:.0f}/{right_width1:.0f}", 0x07FF)
            
            # 第4行：元素状态显示 - 根据当前元素类型显示相应参数
            if ring_state != NO_RING or ring_left or ring_right:
                # 环岛状态显示
                ring_status = ""
                if ring_state == NO_RING: ring_status = "NoRing"
                elif ring_state == FIND_RING: ring_status = "FOUND"
                elif ring_state == FIND_RING_STAGE2: ring_status = "FOUND2"
                elif ring_state == READY_IN_RING: ring_status = "READY"
                elif ring_state == IN_RING: ring_status = "IN_RING"
                elif ring_state == READY_OUT_RING: ring_status = "READY_OUT"
                elif ring_state == OUT_RING: ring_status = "OUT_RING"
                elif ring_state == READY_NO_RING: ring_status = "READY_NO"
                
                ring_dir = ""
                if ring_left: ring_dir = "L"
                elif ring_right: ring_dir = "R"
                
                # 显示角度积累信息
                if ring_state == IN_RING or ring_state == READY_OUT_RING:
                    # 在环岛中状态显示角度差值
                    angle_info = f" A:{abs(ring_angle - imu_data_obj.Yaw):.1f}"
                else:
                    # 其他状态显示编码器距离信息
                    angle_info = f" E:{abs(ring_encoder - encoder_integral):.0f}"
                lcd.str12(0, 231, f"Ring:{ring_status}{ring_dir}{angle_info}", 0xF800)  # 红色
                
            elif cross_flag or pre_cross_flag:
                # 十字路口状态显示
                cross_status = ""
                cross_encoder_info = ""
                
                if pre_cross_flag:
                    cross_status = "PRE_CROSS"
                    cross_encoder_info = f" E:{abs(pre_cross_encoder - encoder_integral):.0f}"
                elif cross_flag:
                    cross_status = "CROSS"
                    cross_encoder_info = f" E:{abs(cross_encoder - encoder_integral):.0f}"
                    
                # 显示四边界状态
                four_boundary = "4B" if (CCD1_left_flag and CCD1_right_flag and CCD2_left_flag and CCD2_right_flag) else "NO"
                lcd.str12(0, 231, f"{cross_status} {four_boundary}{cross_encoder_info}", 0x07E0)  # 绿色
                
            else:
                # 正常巡线状态
                element_status = "ON" if element_en else "OFF"
                lcd.str12(0, 231, f"Normal Line Element:{element_status}", 0xFFFF)  # 白色
            
            # 第5行：系统信息
            lcd.str12(0, 243, f"Pitch:{imu_data_obj.Pitch:4.1f} count:", 0x07FF)  # 青色
        except:
            # 显示出错也要尝试显示基本信息
            try:
                lcd.str12(0, 279, f"Display:ERROR", 0xF800)
                lcd.str12(0, 291, f"System:Running", 0xF800)
            except:
                pass

    # 安全保护：非WiFi模式下，电机满转时停止
    if not wifi_enabled and (current_speed > 260 or current_speed < -260):
        
        # 停止所有定时器
        pit1.stop()
        pit3.stop()
        pit2.stop()
        
        # 长响警告
        set_beep_long()
        beep_process()

        while True:
            # 立即停止电机
            motor_l.duty(0)
            motor_r.duty(0)
        
        break  # 退出主循环
    
    gc.collect()


