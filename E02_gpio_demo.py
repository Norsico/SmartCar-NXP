
# 本示例程序演示如何使用 machine 库的 Pin 类接口的外部中断
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的霍尔停车检测接口

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并且学习板上 D24 对应蜂鸣器每 500ms(0.5s) 响一次
# 并且实时输出 D8 引脚的电平
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4  是 LED
# 学习板上 D24 对应蜂鸣器
# 学习板上 D8  对应一号拨码开关
# 学习板上 D9  对应二号拨码开关

# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 引脚方向 模式配置 默认电平
# 详细内容参考 固件接口说明
led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
beep    = Pin('D24', Pin.OUT, pull = Pin.PULL_UP_47K, value = False)
switch1 = Pin('D8' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)

state1  = switch1.value()
state2  = switch2.value()

while True:
    time.sleep_ms(50)
    beep.high()
    time.sleep_ms(50)
    beep.low()
    time.sleep_ms(400)

    led.toggle()
    print("Switch D8 is {:>1d}.".format(switch1.value()))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
