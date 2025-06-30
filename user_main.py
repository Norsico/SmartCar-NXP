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
        wifi = WIFI_SPI("OnePlus 13", "1234567890xia", WIFI_SPI.TCP_CONNECT, "192.168.71.9", "8086")
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
THRESHOLD_MULTIPLE_1 = 21  # 近端更灵敏
THRESHOLD_MULTIPLE_2 = 21  # 远端适中

# 二值化阈值百分比：控制黑白场景判断 (参考值: 30-60)
# - 用于判断当前区域是否为黑色场景(起跑线、停车区等)
# - 值越小越容易判断为黑色场景
THRESHOLD_1 = 40          # 二值化较松
THRESHOLD_2 = 40

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
ring_threshold_1 = 18  # 环岛内部使用的较小阈值

# 环岛检测阈值 - 参考C代码调整
RING_QULU_THRESHOLD = 7  # 环岛曲率检测阈值（参考代码使用12）

# 环岛各阶段参数 - 需要根据实际测试调整
READY_IN_RING_ENCODER = 32     # 进入环岛前的编码器距离（增大，因为积分值会更大）
IN_RING_ENCODER = 75           # 环岛内部编码器距离
NO_RING_ENCODER = 100          # 出环岛后的编码器距离

# 环岛速度等级
RING_50 = False
RING_60 = False  
RING_90 = False

# 编码器积分值（用于距离计算）
encoder_integral = 0
ring_encoder = 0        # 环岛编码器计数
ring_angle = 0          # 环岛角度计数

# 十字路口相关全局变量
cross_flag = False      # 十字路口标志
cross_encoder = 0       # 十字路口编码器计数
cross_delay_encoder = 0 # 环岛结束后的延时编码器值
cross_middle_line = MIDDLE_LINE  # 检测到十字路口时保存的中线值

# 十字路口参数 - 需要调试优化
CROSS_ENCODER = 30      # 十字路口编码器距离阈值（参考值）
CROSS_DELAY = 40        # 环岛结束后延时距离，避免误检测

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
    beep_timer = 5  # 短响100ms，20ms*5=100ms

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
                beep_timer = 5  # 间隔100ms
        elif beep_double_count == 1:  # 间隔
            if beep_timer > 0:
                beep_off()
                beep_timer -= 1
            else:
                beep_double_count = 2
                beep_timer = 5  # 第二声短响100ms
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

# PID参数 - 进一步增强响应强度
angle_kp = -1984.9 #测过了 两个都是负的 kd不是正的
angle_ki = 0
angle_kd = -180.54

roll_angle_Kp = 0.101 #纯纯脑瘫角度环 调死我了
roll_angle_Ki = 0
roll_angle_Kd = 0.1131 #0.0826 

speed_Kp = 0.047 # 0.063 老铁我发现这东西不能给大 给大了就容易震动了 速度环参数给偏小一点 速度积分也是 跑起来效果就比大的好
speed_Ki = 1.5E-06# 因为我觉得哈 这东西太大了会强迫快速到达预定速度 但是拐弯的时候就容易低头冲出去 而且震荡大
speed_Kd = 1.57 # 1.7 给小了虽然到达预定速度的时间会变长但是到达之后毕竟参数小震荡肯定好点 还是选择稳定好 要速度快可以改预定速度

# 线路跟踪PD控制器参数 - 参考C代码优化
line_kp = 13.5  # 增大比例系数，提高响应速度（参考C代码舵机控制强度）
line_squart_kp = 0.02
line_kd = 293 # 减小微分系数，避免震荡（参考C代码的平滑控制）

# 前瞻控制参数已删除 - 只使用近端CCD巡线

# 控制变量
angle_1 = speed_1 = motor1 = motor2 = 0
med_roll_angle = 58.6  # 调整平衡角度
TARGET_SPEED = 160  # 设置小的前进速度进行测试
ticker_count = 0

# WiFi调参数据存储 - 改为平衡车+巡线控制参数
wifi_data = [line_kp, line_kd, roll_angle_Kp, roll_angle_Kd, speed_Kp, line_squart_kp, speed_Kd, TARGET_SPEED]

def update_wifi_parameters():
    """更新WiFi调参数据"""
    global line_kp, line_kd, line_squart_kp, roll_angle_Kp, roll_angle_Kd
    global speed_Kp, speed_Ki, speed_Kd, TARGET_SPEED
    global pid_angle, pid_speed, pid_line, wifi_data, motor1, motor2
    
    if not wifi_enabled:
        return
    
    try:
        # 数据解析
        data_flag = wifi.data_analysis()
        
        # 检查各通道是否有数据更新
        for i in range(8):
            if data_flag[i]:
                wifi_data[i] = wifi.get_data(i)
        
        # 更新平衡车+巡线控制参数
        line_kp = wifi_data[0]              # 巡线比例控制
        line_kd = wifi_data[1]              # 巡线微分控制
        roll_angle_Kp = wifi_data[2]        # 角度环比例控制
        roll_angle_Kd = wifi_data[3]        # 角度环微分控制
        speed_Kp = wifi_data[4]             # 速度环比例控制
        line_squart_kp = wifi_data[5]       # 巡线平方项控制
        speed_Kd = wifi_data[6]             # 速度环微分控制
        TARGET_SPEED = wifi_data[7]         # 目标速度
        
        # 更新PID控制器参数
        pid_line.kp = line_kp
        pid_line.kd = line_kd
        pid_line.kp_squart = line_squart_kp
        
        pid_angle.kp = roll_angle_Kp
        pid_angle.kd = roll_angle_Kd
        
        pid_speed.kp = speed_Kp
        pid_speed.ki = speed_Ki  # speed_Ki保持原值，不从WiFi调参
        pid_speed.kd = speed_Kd
        
        # 发送示波器数据 - 显示巡线+平衡车控制相关信息
        wifi.send_oscilloscope(
            line_deviation, line_control_output, imu_data_obj.gyro_x, imu_data_obj.Pitch,
            line_kp, line_kd, line_squart_kp, TARGET_SPEED)
    
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
        output = self.kp * err  + self.kd * err_diff+ self.kp_squart * err * abs(err)
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
count_time=0
def control_loop(timer):
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2, imu_data, line_control_output,count_time
    
    ticker_flag = True
    count_time = (count_time+1)%2000
    ticker_count = (ticker_count + 1) % 10

    # 1ms: 角速度控制
    imu_data = imu.get()
    imu_process()
    
    motor1 = pid_angle_speed.update(angle_1, imu_data_obj.gyro_x)
    motor2 = motor1
    # CCD巡线控制
    motor1 += line_control_output  # 左电机增加转向控制
    motor2 -= line_control_output  # 右电机减少转向控制
    
    motor1 = limit(motor1, -6666, 6666)  # 增加电机输出限制，提高响应强度
    motor2 = limit(motor2, -6666, 6666)  # 增加电机输出限制，提高响应强度
    
    motor_l.duty(-motor1)
    motor_r.duty(-motor2)
    
    # 5ms: 角度控制
    #if ticker_count % 5 == 0:
    angle_1 = pid_angle.update(med_roll_angle - speed_1, imu_data_obj.Pitch)
    
    # 10ms: 速度控制
    if ticker_count == 5:
        avg_speed = -(kalman_l.output + kalman_r.output) / 2
        speed_1 = pid_speed.update(TARGET_SPEED, avg_speed)
        speed_1 = limit(speed_1, -10, 10)  # 限制角度偏移
    if count_time == 0:
        pid_speed.err_sum=0;

def encoder_update(timer):
    global encoder_integral
    
    # 获取原始编码器数据（每个周期的脉冲数）
    encoder_l_raw = encoder_l.get()
    encoder_r_raw = encoder_r.get()
    
    # 更新卡尔曼滤波器
    kalman_l.update(encoder_l_raw)
    kalman_r.update(encoder_r_raw)
    
    # 计算平均脉冲数（参考C代码逻辑）
    # encoder = (encoder_L + encoder_R) * 0.5
    avg_encoder = (abs(encoder_l_raw) + abs(encoder_r_raw)) * 0.5
    
    # 积分计算距离（参考C代码：encoder_integral += encoder * 0.02）
    # 这里编码器值就是脉冲数，直接乘以时间周期进行积分
    distance_increment = avg_encoder * 0.01  # 10ms定时器周期
    encoder_integral += distance_increment

def ccd_image_init():
    """CCD图像初始化"""
    global Trk, CCD1, CCD2
    Trk.middle_sideline1 = MIDDLE_LINE
    Trk.middle_sideline2 = MIDDLE_LINE
    CCD1.bin_thrd = 0
    CCD2.bin_thrd = 0

def set_ring_type(ring_type):
    """设置环岛类型"""
    global RING_50, RING_60, RING_90
    # 重置所有标志
    RING_50 = False
    RING_60 = False
    RING_90 = False
    
    # 设置对应类型
    if ring_type == 50:
        RING_50 = True
    elif ring_type == 60:
        RING_60 = True
    elif ring_type == 90:
        RING_90 = True

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
    
    # 基础中线计算
    # CCD1中线计算 - 主要巡线传感器，考虑丢线情况
    if CCD1_left_flag and CCD1_right_flag:
        # 双边都有效，正常计算
        Trk.middle_sideline1 = (Trk.left_sideline1 + Trk.right_sideline1) / 2.0
    elif CCD1_left_flag and not CCD1_right_flag:
        # 左边有效，右边丢线，使用上次右边界值计算中线
        Trk.middle_sideline1 = (Trk.left_sideline1 + Trk.right_sideline1_last) / 2.0
    elif not CCD1_left_flag and CCD1_right_flag:
        # 右边有效，左边丢线，使用上次左边界值计算中线
        Trk.middle_sideline1 = (Trk.left_sideline1_last + Trk.right_sideline1) / 2.0
    else:
        # 双边都丢线，保持上次中线值
        pass  # Trk.middle_sideline1保持不变
    
    # CCD2中线计算 - 主要用于元素检测，简单计算即可
    if CCD2_left_flag and CCD2_right_flag:
        # 双边都有效，正常计算
        Trk.middle_sideline2 = (Trk.left_sideline2 + Trk.right_sideline2) / 2.0
    else:
        # 丢线时保持上次中线值，因为CCD2主要用于元素检测而非巡线
        pass  # Trk.middle_sideline2保持不变
    
    # 宽度计算
    Trk.width1 = Trk.right_sideline1 - Trk.left_sideline1
    Trk.width2 = Trk.right_sideline2 - Trk.left_sideline2
    
    # 元素识别关闭时，跳过所有特殊中线处理，只使用基础中线
    if not element_en:
        return
    
    # 十字路口中线特殊处理 - 参考C代码注释
    # 十字路口期间，使用检测到十字路口时保存的中线值直行通过
    # 不再动态调整方向，避免受到CCD丢线影响
    if cross_flag:
        # 十字路口状态下，直接使用保存的中线值直行
        Trk.middle_sideline1 = cross_middle_line
    
    # 环岛中线特殊处理 - 参考C代码逻辑
    # 左环岛处理
    elif ring_left:
        if ring_state == FIND_RING:
            # 进入环岛阶段：基于右边界偏移计算中线
            Trk.middle_sideline1 = Trk.right_sideline1 - (CCD1_SET_WIDTH / 2)
        elif ring_state == READY_IN_RING or ring_state == IN_RING or ring_state == READY_OUT_RING:
            # 环岛内部阶段：基于左边界偏移计算中线（短响两声后立即切换到左侧巡线）
            if CCD1_left_flag:
                # 有左边界时，使用左边界偏移
                Trk.middle_sideline1 = Trk.left_sideline1 + (CCD1_SET_WIDTH / 2) + 8
            else:
                # 左边界丢失时，使用上次左边界位置
                Trk.middle_sideline1 = Trk.left_sideline1_last + (CCD1_SET_WIDTH / 2) + 8

        elif ring_state == OUT_RING:
            # 出环岛阶段：按近端CCD1右边界巡线
            if CCD1_right_flag:
                # 有右边界时，按右边界偏移计算中线
                Trk.middle_sideline1 = Trk.right_sideline1 - (CCD1_SET_WIDTH / 2)
            else:
                # 右边界也丢失时，保持上次中线
                pass
    
    # 右环岛处理
    elif ring_right:
        if ring_state == FIND_RING:
            # 进入环岛阶段：基于左边界偏移计算中线
            Trk.middle_sideline1 = Trk.left_sideline1 + (CCD1_SET_WIDTH / 2)
        elif ring_state == READY_IN_RING or ring_state == IN_RING or ring_state == READY_OUT_RING:
            # 环岛内部阶段：基于右边界偏移计算中线（短响两声后立即切换到右侧巡线）
            if CCD1_right_flag:
                # 有右边界时，使用右边界偏移
                Trk.middle_sideline1 = Trk.right_sideline1 - (CCD1_SET_WIDTH / 2) - 11
            else:
                # 右边界丢失时，使用上次右边界位置
                Trk.middle_sideline1 = Trk.right_sideline1_last - (CCD1_SET_WIDTH / 2) - 11

        elif ring_state == OUT_RING:
            # 出环岛阶段：按近端CCD1左边界巡线
            if CCD1_left_flag:
                # 有左边界时，按左边界偏移计算中线
                Trk.middle_sideline1 = Trk.left_sideline1 + (CCD1_SET_WIDTH / 2) - 5 
            else:
                # 左边界也丢失时，保持上次中线
                pass
    
    # 如果是十字路口，使用CCD2的中线（这里可以根据需要添加十字处理）
    # if cross_flag:
    #     Trk.middle_sideline1 = Trk.middle_sideline2

def ring_detection():
    """
    环岛检测 - 添加准备进入环岛状态
    0. NO_RING: 无环岛状态
    1. FIND_RING: 发现环岛（左侧丢线+右侧曲率小）
    2. READY_IN_RING: 准备进入环岛（确认环岛并记录参数）
    """
    global ring_state, ring_left, ring_right
    global CCD1_left_flag, CCD1_right_flag, CCD2_left_flag, CCD2_right_flag
    global black_write_1, black_write_2
    global ring_encoder, ring_angle, encoder_integral
    global imu_data_obj  # 使用IMU数据
    
    if ring_state == NO_RING:
        # 检测左环岛
        # 条件：近端CCD左侧丢线 + 右边界存在 + 右侧曲率小 + 远端不是全黑
        if (not CCD1_left_flag and Trk.right_sideline1 < 95 and 
            Trk.right_qulu <= RING_QULU_THRESHOLD and not black_write_2):
            ring_state = FIND_RING
            ring_left = True
            ring_right = False
            set_beep_short()  # 发现环岛：短响一声
            ring_encoder = encoder_integral  # 记录发现环岛时的编码器值
            
        # 检测右环岛
        # 条件：近端CCD右侧丢线 + 左边界存在 + 左侧曲率小 + 远端不是全黑
        elif (not CCD1_right_flag and CCD1_left_flag and Trk.left_sideline1 > 40 and
              Trk.left_qulu <= RING_QULU_THRESHOLD and not black_write_2):
            ring_state = FIND_RING
            ring_left = False
            ring_right = True
            set_beep_short()  # 发现环岛：短响一声
            ring_encoder = encoder_integral  # 记录发现环岛时的编码器值
            
        # 如果同时检测到十字（双侧丢线），排除环岛误判
        if (not CCD1_left_flag and not CCD1_right_flag and 
            not black_write_2 and not black_write_1):
            # 十字路口，不是环岛
            pass
            
    elif ring_state == FIND_RING and ring_left:
        # 阶段1→2：左环岛确认第一阶段
        # 条件1：编码器距离足够 + 前后端CCD拍摄宽度小于35
        
        if abs(ring_encoder - encoder_integral) < READY_IN_RING_ENCODER and Trk.width1 < 44 and Trk.width2 < 44:
            # 记录第一阶段完成的编码器值
            ring_encoder = encoder_integral
            ring_state = FIND_RING_STAGE2
            set_beep_short()  # 第一阶段完成：短响一声
            
        elif Trk.right_qulu > 30 or abs(ring_encoder - encoder_integral) > READY_IN_RING_ENCODER:  # 右侧曲率过大，可能是误判
            ring_state = NO_RING
            ring_left = False
            # 恢复近端CCD原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    elif ring_state == FIND_RING and ring_right:
        # 阶段1→2：右环岛确认第一阶段
        # 条件1：编码器距离足够 + 前后端CCD拍摄宽度小于40
        
        if abs(ring_encoder - encoder_integral) < READY_IN_RING_ENCODER and Trk.width1 < 40 and Trk.width2 < 40:
            # 记录第一阶段完成的编码器值
            ring_encoder = encoder_integral
            ring_state = FIND_RING_STAGE2
            set_beep_short()  # 第一阶段完成：短响一声
            
        elif Trk.left_qulu > 30 or abs(ring_encoder - encoder_integral) > READY_IN_RING_ENCODER:  # 左侧曲率过大，可能是误判
            ring_state = NO_RING
            ring_right = False
            # 恢复近端CCD原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    elif ring_state == FIND_RING_STAGE2 and ring_left:
        # 阶段2→3：左环岛确认第二阶段
        # 条件2：远端左边界位置小于27
        
        if Trk.left_sideline2 < 27:
            ring_state = READY_IN_RING
            set_beep_double_short()  # 确认环岛：短响两声
            # 只降低近端CCD1阈值，提高边界检测灵敏度
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = ring_threshold_1
            
        elif abs(ring_encoder - encoder_integral) > 50:  # 如果走了太远还没满足条件，可能是误判
            ring_state = NO_RING
            ring_left = False
            # 恢复近端CCD1原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    elif ring_state == FIND_RING_STAGE2 and ring_right:
        # 阶段2→3：右环岛确认第二阶段
        # 条件2：远端右边界位置大于100（对称于左环岛的27）
        
        if Trk.right_sideline2 > 100:
            ring_state = READY_IN_RING
            set_beep_double_short()  # 确认环岛：短响两声
            # 只降低近端CCD1阈值，提高边界检测灵敏度
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = ring_threshold_1
            
        elif abs(ring_encoder - encoder_integral) > 50:  # 如果走了太远还没满足条件，可能是误判
            ring_state = NO_RING
            ring_right = False
            # 恢复近端CCD1原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    elif ring_state == READY_IN_RING:
        # 阶段2→3：准备进入环岛 -> 在环岛中
        # 条件：编码器距离足够（走了足够远开始执行环岛策略）
        if ring_left and abs(ring_encoder - encoder_integral) < IN_RING_ENCODER and Trk.left_sideline2 < 27:
            ring_state = IN_RING
            set_beep_long()  # 进入环岛：长响一声
        elif ring_right and abs(ring_encoder - encoder_integral) < IN_RING_ENCODER and Trk.right_sideline2 > 100:
            ring_state = IN_RING
            set_beep_long()  # 进入环岛：长响一声
        elif abs(ring_encoder - encoder_integral) > IN_RING_ENCODER:   
            ring_state = NO_RING
            ring_left = False
            ring_right = False
            # 恢复近端CCD1原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
                
    elif ring_state == IN_RING:
        # 阶段3→4：在环岛中 -> 准备出环岛
        # 条件：近端CCD1双边都丢线（表示即将出环岛）
        if abs(Trk.left_sideline1 - Trk.right_sideline1) > 80 and not CCD2_left_flag and not CCD2_right_flag:
            # 恢复近端CCD1原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
            ring_state = READY_OUT_RING
            ring_encoder = encoder_integral  # 重新记录编码器值用于出环岛阶段
            set_beep_short()  # 准备出环岛：短响一声
                
    elif ring_state == READY_OUT_RING:
        # 阶段4→5：准备出环岛 -> 出环岛
        # 条件：近端CCD1重新检测到边界（出环岛开始）
        if ring_left and CCD1_right_flag:
            # 左环岛：检测到右边界表示开始出环岛
            ring_state = OUT_RING
            ring_encoder = encoder_integral  # 重新记录编码器值用于最终阶段
            set_beep_long()  # 出环岛：长响一声
        elif ring_right and CCD1_left_flag:
            # 右环岛：检测到左边界表示开始出环岛
            ring_state = OUT_RING
            ring_encoder = encoder_integral  # 重新记录编码器值用于最终阶段
            set_beep_long()  # 出环岛：长响一声

                
    elif ring_state == OUT_RING:
        # 阶段5→6：出环岛 -> 准备回到无环岛
        # 条件：编码器距离足够（出环岛后走了足够远）
        if abs(ring_encoder - encoder_integral) > NO_RING_ENCODER:
            ring_state = READY_NO_RING
            # 恢复近端CCD1原始阈值
            global THRESHOLD_MULTIPLE_1
            THRESHOLD_MULTIPLE_1 = original_threshold_1
            
    elif ring_state == READY_NO_RING:
        # 阶段6→0：准备回到无环岛 -> 无环岛
        # 直接清除所有环岛标志
        ring_state = NO_RING
        ring_left = False
        ring_right = False
        ring_encoder = 0
        ring_angle = 0
        
        # 记录环岛结束时的编码器值，用于延时
        global cross_delay_encoder
        cross_delay_encoder = encoder_integral
        
        set_beep_off()  # 停止蜂鸣器


def cross_detection():
    """十字路口检测 - 移植自C语言参考代码"""
    global cross_flag, cross_encoder, encoder_integral
    global CCD1_left_flag, CCD1_right_flag, black_write_1, black_write_2
    global ring_state, ring_left, ring_right  # 添加环岛状态检查
    global cross_delay_encoder  # 添加延时检查
    
    # 检查环岛结束后的延时
    # 如果刚结束环岛，需要等待一段距离后才能检测十字路口
    ring_end_delay_ok = (cross_delay_encoder == 0 or 
                        abs(encoder_integral - cross_delay_encoder) > CROSS_DELAY)
    
    # 十字路口检测条件：
    # 1. 当前没有环岛状态（避免出环岛时误判为十字路口）
    # 2. 环岛结束后已经等待足够距离
    # 3. 近端CCD双边都丢线或赛道很窄
    # 4. 远端CCD赛道很宽（表示前方有分叉）
    # 5. 前后端都不是黑色场景
    # 6. 当前没有十字路口标志
    if (ring_state == NO_RING and  # 关键：只有在无环岛状态时才检测十字路口
        not ring_left and not ring_right and  # 确保没有环岛标志
        ring_end_delay_ok and  # 环岛结束后延时检查
        Trk.left_sideline2 < 17 and Trk.right_sideline2 > 102 and
        abs(Trk.left_sideline1 - Trk.right_sideline1) < 50 and 
        abs(Trk.left_sideline2 - Trk.right_sideline2) > 90 and 
        not black_write_1 and not black_write_2 and
        not cross_flag):
        
        # 检测到十字路口
        cross_flag = True
        set_beep_long()  # 十字路口：长响
        
        # 保存检测到十字路口时的中线值，后续直行使用
        global cross_middle_line
        cross_middle_line = Trk.middle_sideline1
        
        # 记录当前编码器值（参考C代码：encoder_integral=100）
        # 这里保持当前积分值，记录检测点
        cross_encoder = encoder_integral
        
    # 十字路口退出条件：
    # 走过足够距离后清除十字路口标志
    if cross_flag and abs(cross_encoder - encoder_integral) > CROSS_ENCODER:
        # 退出十字路口状态
        cross_flag = False
        set_beep_off()  # 停止蜂鸣器
        
        # 重置编码器计数（参考C代码逻辑）
        cross_encoder = 0
        
        # 重置保存的中线值
        global cross_middle_line
        cross_middle_line = MIDDLE_LINE


def element_detection():
    """元素检测主函数 - 直接使用边界检测算法结果"""
    # 检查元素识别开关
    if not element_en:
        return  # 元素识别关闭，直接返回
    
    # 元素检测优先级：参考C代码的element()函数逻辑
    # 1. 优先处理环岛（环岛状态不为NO_RING时）
    # 2. 只有在完全无环岛状态时才检测十字路口
    # 3. 避免出环岛时误判为十字路口
    # 4. 十字路口期间不检测环岛，避免误判
    
    # 环岛检测和处理 - 只有在非十字路口状态时才进行
    if not cross_flag:  # 十字路口期间不检测环岛
        ring_detection()
    
    # 十字路口检测 - 只有在完全无环岛状态且无环岛标志时才检测
    # 这样可以避免出环岛阶段的误判
    if (ring_state == NO_RING and not ring_left and not ring_right):
        cross_detection()


def clear_ring_flag():
    """清除环岛和十字路口标志位"""
    global ring_state, ring_left, ring_right, ring_encoder
    global cross_flag, cross_encoder, cross_delay_encoder, cross_middle_line
    global beep_double_count, THRESHOLD_MULTIPLE_1, original_threshold_1
    
    # 清除环岛标志
    ring_state = NO_RING
    ring_left = False
    ring_right = False
    ring_encoder = 0
    beep_double_count = 0
    
    # 清除十字路口标志
    cross_flag = False
    cross_encoder = 0
    cross_delay_encoder = 0  # 重置延时编码器
    cross_middle_line = MIDDLE_LINE  # 重置保存的中线值
    
    set_beep_off()  # 设置停止蜂鸣器标志
    # 恢复近端CCD1原始阈值
    THRESHOLD_MULTIPLE_1 = original_threshold_1
    # 注意：不清零encoder_integral，保持全局距离累积

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
    
    # 6. 智能偏差计算 - 参考C代码优化
    center = MIDDLE_LINE # 赛道中心
    
    # 计算近端和远端偏差
    deviation1 = Trk.middle_sideline1 - center  # 近端偏差（当前位置）
    deviation2 = Trk.middle_sideline2 - center  # 远端偏差（前瞻位置）
    
    # 只使用近端CCD控制策略
    if CCD1_left_flag or CCD1_right_flag:
        # 近端CCD有边界，使用近端CCD控制
        deviation = deviation1
    else:
        # 近端CCD失效，保持上次偏差（添加衰减避免失控）
        global line_deviation
        deviation = line_deviation * 0.95  # 逐渐衰减，避免持续偏移
    
    # 偏差限制 - 参考舵机控制的限制策略
    deviation = max(-50, min(50, deviation))
    
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
    global key
    
    ccd_ticker_flag = True
    ccd_ticker_count = (ccd_ticker_count + 1) % 100
    
    # 初始化CCD数据变量
    ccd_data_upper = None
    ccd_data_lower = None
    
    # 按键处理 - 放在CCD回调中
    try:
        key_data = key.get()
        
        # 检查key2短按 (按键2对应索引1) - 清除环岛标志位
        if key_data[1] == 1:  # 短按
            clear_ring_flag()  # 清除环岛标志位
            key.clear(2)  # 清除按键状态
            
        # 检查key1短按 (按键1对应索引0) - 切换元素识别开关
        if key_data[0] == 1:  # 短按
            global element_en
            element_en = not element_en  # 切换开关状态
            if element_en:
                set_beep_short()  # 开启元素识别：短响
            else:
                set_beep_double_short()  # 关闭元素识别：双短响
                # 关闭元素识别时清除所有元素标志
                clear_ring_flag()
            key.clear(1)  # 清除按键状态
    except:
        pass  # 按键处理出错不影响主要功能
    
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
            line_control_output = pid_line.update(0, line_deviation)  # 目标偏差为0
            
            # 自适应控制强度 - 参考舵机控制范围调整
            # 舵机控制范围约为±450，转换为差速控制需要更大范围
            max_control_output = 4500  # 基础最大输出
            
            # 根据速度和偏差动态调整控制强度
            speed_factor = min(TARGET_SPEED / 100.0, 1.5)  # 速度系数
            deviation_factor = min(abs(line_deviation) / 20.0, 1.2)  # 偏差系数
            
            # 动态最大输出 = 基础输出 × 速度系数 × 偏差系数
            dynamic_max_output = max_control_output * speed_factor * deviation_factor
            
            # 限制线路控制输出
            line_control_output = limit(line_control_output, -dynamic_max_output, dynamic_max_output)
            
    except Exception as e:
        # 发生错误时逐渐减小控制输出，避免突然停止
        line_control_output *= 0.9
    
    # 显示屏更新 - 独立于CCD数据处理，确保始终更新
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
        lcd.str12(0, 207, f"L2:{Trk.left_sideline2:3d} R2:{Trk.right_sideline2:3d} Dev:{line_deviation:4.1f}", 0xFFFF)
        
        # 第3行：两侧曲率和赛道宽度 (重点显示)
        # 计算宽度时使用原始边界检测结果
        left_width1 = abs(Trk.middle_sideline1 - Trk.left_sideline1) if CCD1_left_flag else 0
        right_width1 = abs(Trk.right_sideline1 - Trk.middle_sideline1) if CCD1_right_flag else 0
        lcd.str12(0, 219, f"QL:{Trk.left_qulu:4.1f} QR:{Trk.right_qulu:4.1f} W:{left_width1:.0f}/{right_width1:.0f}", 0x07FF)
        
        # 第4行：巡线控制参数显示 - 新增
        lcd.str12(0, 231, f"LineKp:{line_kp:4.1f} LineKd:{line_kd:4.1f} Out:{line_control_output:.0f}", 0xFFE0)  # 黄色
        
        # 第5行：巡线模式显示 - 修改为只使用近端CCD
        lcd.str12(0, 243, f"Line Mode: Near CCD Only", 0x07FF)  # 青色
        
        # 第6行：CCD阈值状态显示
        threshold_status = f"T1:{THRESHOLD_MULTIPLE_1} T2:{THRESHOLD_MULTIPLE_2}"
        if THRESHOLD_MULTIPLE_1 == ring_threshold_1:
            threshold_status += " (Ring-T1)"
        lcd.str12(0, 255, threshold_status, 0xFFE0)  # 黄色
        
        # 第7行：CCD1边界检测状态 (基于原始边界检测算法)
        ccd1_status = ""
        ccd1_status += "L1:" + ("V" if CCD1_left_flag else "X")  # V=有效 X=丢线
        ccd1_status += " R1:" + ("V" if CCD1_right_flag else "X")
        lcd.str12(0, 267, f"CCD1 {ccd1_status}", 0xF81F)  # 紫色
        
        # 第8行：CCD2边界检测状态 (基于原始边界检测算法)
        ccd2_status = ""
        ccd2_status += "L2:" + ("V" if CCD2_left_flag else "X")
        ccd2_status += " R2:" + ("V" if CCD2_right_flag else "X")
        lcd.str12(0, 279, f"CCD2 {ccd2_status}", 0xF81F)  # 紫色
        
        # 第9行：黑白场景检测 (重点显示)
        black_status = ""
        black_status += "B1:" + ("Y" if black_write_1 else "N")
        black_status += " B2:" + ("Y" if black_write_2 else "N")
        lcd.str12(0, 291, f"Black {black_status}", 0xFFE0)  # 黄色
        
        # 第10行：环岛状态显示
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
        
        # 显示编码器距离信息
        encoder_info = ""
        if ring_state != NO_RING:
            encoder_info = f" E:{abs(ring_encoder - encoder_integral):.0f}"
        
        lcd.str12(0, 303, f"Ring:{ring_status}{ring_dir}{encoder_info} K2:Clr K1:Elm", 0xF800)  # 红色
        
        # 第11行：十字路口状态显示
        cross_status = "Cross:ON" if cross_flag else "Cross:OFF"
        cross_info = ""
        if cross_flag:
            cross_info = f" E:{abs(cross_encoder - encoder_integral):.0f} M:{cross_middle_line:.1f}"
        
        # 显示环岛结束后的延时状态
        delay_info = ""
        if cross_delay_encoder > 0:
            delay_distance = abs(encoder_integral - cross_delay_encoder)
            if delay_distance < CROSS_DELAY:
                delay_info = f" Delay:{CROSS_DELAY - delay_distance:.0f}"
        
        lcd.str12(0, 315, f"{cross_status}{cross_info}{delay_info}", 0x07FF)  # 青色
        
        # 第12行：元素识别开关状态显示
        element_status = "Element:ON" if element_en else "Element:OFF"
        lcd.str12(0, 327, element_status, 0xF81F)  # 紫色
    except:
        # 显示出错也要尝试显示基本信息
        try:
            lcd.str12(0, 279, f"Ring:ERROR Key2:Clear", 0xF800)
            lcd.str12(0, 291, f"System:Display Error", 0xF800)
        except:
            pass

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
    
    # 蜂鸣器处理 - 不阻塞主循环
    beep_process()
    
    # WiFi调参更新
    update_wifi_parameters()
    
    if end_switch.value() != end_state:
        pit1.stop()
        pit3.stop()
        pit2.stop()  # 停止CCD定时器
        break
    
    # 主循环延时，控制蜂鸣器更新频率约50Hz
    time.sleep_ms(20)
    
    gc.collect()


