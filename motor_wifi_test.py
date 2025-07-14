from machine import *
from display import *
from smartcar import *
from seekfree import *
import gc
import time

# WiFi配置 - 请根据实际情况修改
WIFI_SSID = "xyh"
WIFI_PASSWORD = "1261340160xyh"
WIFI_SERVER_IP = "192.168.43.3"
WIFI_SERVER_PORT = "8086"

# 电机PWM限制值
MAX_PWM = 10000  # 最大PWM值
MIN_PWM = -10000  # 最小PWM值

# 全局变量
wifi_enabled = False
left_motor_pwm = 0    # 左电机PWM值
right_motor_pwm = 0   # 右电机PWM值
wifi_data = [0, 0, 0, 0, 0, 0, 0, 0]  # 8通道WiFi数据

def limit_pwm(value):
    """限制PWM值在安全范围内"""
    return max(MIN_PWM, min(MAX_PWM, value))

def init_wifi():
    """初始化WiFi连接"""
    global wifi_enabled, wifi
    try:
        wifi = WIFI_SPI(WIFI_SSID, WIFI_PASSWORD, WIFI_SPI.TCP_CONNECT, WIFI_SERVER_IP, WIFI_SERVER_PORT)
        wifi.send_str("Motor PWM Control Test Ready.\r\n")
        time.sleep_ms(500)
        wifi_enabled = True
        print("WiFi模块初始化成功")
        return True
    except Exception as e:
        print(f"WiFi初始化失败: {e}")
        wifi_enabled = False
        return False

def init_motors():
    """初始化电机"""
    global motor_l, motor_r
    try:
        # 初始化左电机 (C28-PWM, C29-DIR)
        motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=True)
        # 初始化右电机 (C30-PWM, C31-DIR)
        motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=False)
        print("电机初始化成功")
        return True
    except Exception as e:
        print(f"电机初始化失败: {e}")
        return False

def init_display():
    """初始化显示屏"""
    global lcd
    try:
        # 定义片选引脚
        cs = Pin('B29', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        cs.high()
        cs.low()
        
        # 定义控制引脚
        rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        dc = Pin('B5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        
        # 创建LCD驱动实例
        drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
        lcd = LCD(drv)
        
        # 设置显示参数
        lcd.color(0xFFFF, 0x0000)  # 前景色白色，背景色黑色
        lcd.mode(0)  # 竖屏模式
        lcd.clear(0x0000)  # 清屏
        
        print("显示屏初始化成功")
        return True
    except Exception as e:
        print(f"显示屏初始化失败: {e}")
        return False

def update_wifi_data():
    """更新WiFi数据
    通道0: 左电机PWM值 (范围: -10000 到 10000)
    通道1: 右电机PWM值 (范围: -10000 到 10000)
    通道2: 预留
    通道3: 预留
    通道4: 预留
    通道5: 预留
    通道6: 预留
    通道7: 预留
    """
    global wifi_data, left_motor_pwm, right_motor_pwm
    
    if not wifi_enabled:
        return
    
    try:
        # 数据解析
        data_flag = wifi.data_analysis()
        
        # 检查各通道是否有数据更新
        for i in range(8):
            if data_flag[i]:
                wifi_data[i] = wifi.get_data(i)
        
        # 更新电机PWM值
        left_motor_pwm = limit_pwm(wifi_data[0])   # 通道0: 左电机PWM
        right_motor_pwm = limit_pwm(wifi_data[1])  # 通道1: 右电机PWM
        
        # 发送示波器数据 - 显示当前PWM值
        wifi.send_oscilloscope(left_motor_pwm, right_motor_pwm, 0, 0, 0)
        
    except Exception as e:
        print(f"WiFi数据更新失败: {e}")

def control_motors():
    """控制电机"""
    try:
        # 设置电机PWM占空比
        motor_l.duty(left_motor_pwm)
        motor_r.duty(right_motor_pwm)
    except Exception as e:
        print(f"电机控制失败: {e}")

def update_display():
    """更新显示屏"""
    try:
        # 清除文本区域
        lcd.str12(0, 10, "Motor PWM Control Test      ", 0xFFFF)
        lcd.str12(0, 30, "========================    ", 0x07E0)
        
        # WiFi状态
        wifi_status = "Connected" if wifi_enabled else "Disconnected"
        lcd.str12(0, 50, f"WiFi: {wifi_status}          ", 0x07E0 if wifi_enabled else 0xF800)
        
        # 电机PWM值显示
        lcd.str12(0, 70, f"Left Motor PWM:  {left_motor_pwm:6d}  ", 0xFFFF)
        lcd.str12(0, 90, f"Right Motor PWM: {right_motor_pwm:6d}  ", 0xFFFF)
        
        # PWM百分比显示
        left_percent = (left_motor_pwm / MAX_PWM) * 100
        right_percent = (right_motor_pwm / MAX_PWM) * 100
        lcd.str12(0, 110, f"Left:  {left_percent:5.1f}%        ", 0x07FF)
        lcd.str12(0, 130, f"Right: {right_percent:5.1f}%        ", 0x07FF)
        
        # 安全提示
        if abs(left_motor_pwm) > 5000 or abs(right_motor_pwm) > 5000:
            lcd.str12(0, 150, "WARNING: High PWM!       ", 0xF800)
        else:
            lcd.str12(0, 150, "PWM Level: Safe          ", 0x07E0)
        
        # 操作说明
        lcd.str12(0, 170, "Use WiFi to control:     ", 0xFFE0)
        lcd.str12(0, 190, "Channel 0: Left Motor    ", 0xFFE0)
        lcd.str12(0, 210, "Channel 1: Right Motor   ", 0xFFE0)
        lcd.str12(0, 230, "Range: -10000 to 10000   ", 0xFFE0)
        
        # 实时数据
        lcd.str12(0, 250, f"Data: {wifi_data[0]:.0f}, {wifi_data[1]:.0f}    ", 0x07FF)
        
    except Exception as e:
        print(f"显示更新失败: {e}")

def emergency_stop():
    """紧急停止"""
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = 0
    right_motor_pwm = 0
    try:
        motor_l.duty(0)
        motor_r.duty(0)
        print("紧急停止执行")
    except:
        pass

def main():
    """主函数"""
    print("=== 电机WiFi控制测试程序 ===")
    
    # 初始化各个模块
    if not init_motors():
        print("电机初始化失败，程序退出")
        return
    
    if not init_display():
        print("显示屏初始化失败，程序退出")
        return
    
    # 尝试初始化WiFi
    if not init_wifi():
        print("WiFi初始化失败，将使用默认PWM值")
    
    print("系统初始化完成")
    print("控制说明:")
    print("- 通道0: 左电机PWM (-10000 到 10000)")
    print("- 通道1: 右电机PWM (-10000 到 10000)")
    print("- 正值: 正转, 负值: 反转")
    print("- 按Ctrl+C停止程序")
    
    # 主循环
    loop_count = 0
    try:
        while True:
            # 更新WiFi数据
            update_wifi_data()
            
            # 控制电机
            control_motors()
            
            # 更新显示 (降低频率)
            if loop_count % 10 == 0:
                update_display()
            
            # 安全检查
            if abs(left_motor_pwm) > MAX_PWM or abs(right_motor_pwm) > MAX_PWM:
                print("PWM值超出安全范围，执行紧急停止")
                emergency_stop()
                break
            
            loop_count += 1
            time.sleep_ms(50)  # 50ms循环周期
            
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        # 程序结束时停止电机
        emergency_stop()
        print("程序结束，电机已停止")

if __name__ == "__main__":
    main() 