from machine import *
from smartcar import ticker
from seekfree import TSL1401
from seekfree import WIFI_SPI
import gc
import time
import json

# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture/read 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
# 这里填了 10 代表 10 次 capture/read 调用才会更新一次数据
ccd = TSL1401(10)
# 调整 CCD 的采样精度为 12bit
ccd.set_resolution(TSL1401.RES_12BIT)

# wifi = WIFI_SPI("OnePlus 13", "1234567890xia", WIFI_SPI.TCP_CONNECT, "192.168.4.9", "8086")
# print("wifi success")

time.sleep_ms(500)
ticker_flag = False
ticker_count = 0
runtime_count = 0
send_interval = 5  # 每5次采集发送一次数据

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)


pit1 = ticker(1)
pit1.capture_list(ccd)
pit1.callback(time_pit_handler)
pit1.start(10)

# 将array转换为可发送的字符串
def array_to_string(arr):
    # 转换为字符串形式，使用逗号分隔
    result = ','.join([str(x) for x in arr])
    return result

def handle_arr(arr):
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
    
    # 如果有有效数据点，计算它们的中间值
    if valid_indices:
        middle_index = sum(valid_indices) / len(valid_indices)
        return middle_index
    else:
        # 如果没有有效数据点，返回-1表示未找到赛道
        return -1

# 主循环
while True:
    if ticker_flag:
        # 通过 get 接口读取数据 参数 [0,1,2,3] 对应学习板上 CCD1/2/3/4 接口
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        
        # 处理CCD数据并获取中间值
        middle_value1 = handle_arr(ccd_data1)
        middle_value2 = handle_arr(ccd_data2)
        
        # 打印数据
        print("CCD1 数据:", ccd_data1)
        print("CCD1 中间值:", middle_value1)
        print("CCD2 中间值:", middle_value2)
        
        # 每隔send_interval次采集发送一次数据
        if ticker_count % send_interval == 0:
            try:
                # 创建数据包
                data_str1 = "CCD1:" + array_to_string(ccd_data1) + "\r\n"
                # 发送数据
                # wifi.send_str(data_str1)
                
                data_str2 = "CCD2:" + array_to_string(ccd_data2) + "\r\n"
                # 发送数据
                # wifi.send_str(data_str2)
                
                # 发送中间值数据
                middle_data = f"MIDDLE:CCD1={middle_value1:.2f},CCD2={middle_value2:.2f}\r\n"
                # wifi.send_str(middle_data)
                
            except Exception as e:
                print("发送数据失败:", e)
        
        ticker_flag = False
        runtime_count = runtime_count + 1
        if 0 == runtime_count % 100:
            print("runtime_count = {:>6d}.".format(runtime_count))
    
    # 回收内存
    gc.collect()
