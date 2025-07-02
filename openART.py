import sensor, image, time
from machine import UART
from pyb import LED
white = LED(4)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 设置摄像头像素格式
sensor.set_framesize(sensor.QQVGA)   # 设置摄像头分辨率
sensor.set_brightness(1300)          # 设置摄像头亮度
sensor.skip_frames(time = 200)
clock = time.clock()

# 初始化串口
uart2 = UART(2, baudrate=115200)     # 初始化串口2 波特率设置为115200

# 根据新的RGB统计数据重新设置阈值
# 但这个范围太宽，我们需要更精确的黑色检测
black_threshold = (29, 58, -12, 3, -25, 2)  # 更精确的黑色检测阈值

# 白色赛道阈值 - 用于检测障碍物周围是否有白色赛道
white_threshold = (70, 100, -15, 15, -15, 15)  # 白色赛道检测阈值

# 获取图像尺寸用于位置判断
img_width = 160  # QQVGA宽度
img_height = 120  # QQVGA高度
center_x = img_width // 2  # 图像中心x坐标

# 定时器相关变量
flag = 0
timer_start = 0  # 定时器开始时间
timer_duration = 3000  # 定时器持续时间3秒(3000毫秒)

# 滤波相关变量
detection_buffer = []  # 存储最近几次的检测结果
buffer_size = 5  # 滤波缓冲区大小，需要连续5次检测
confirm_threshold = 4  # 确认阈值，5次中至少4次检测到才确认
last_confirmed_position = None  # 上次确认的位置

def check_track_around_obstacle(img, blob):
    """检查障碍物对侧和下方是否有白色赛道可以通行"""
    x, y, w, h = blob.rect()

    # 定义检测区域的偏移量
    check_distance = 20  # 检测距离

    # 判断障碍物在图像的左侧还是右侧
    obstacle_x = blob.cx()
    is_obstacle_on_left = obstacle_x < center_x

    white_regions_found = 0
    total_regions = 0

    # 根据障碍物位置检查对侧的白色区域
    regions_to_check = []

    if is_obstacle_on_left:
        # 障碍物在左侧，检测右侧是否有白色赛道
        right_x = min(img_width, x + w + 5)  # 从障碍物右边开始稍微偏移
        right_region = (right_x, y, min(check_distance, img_width - right_x), h)

        if right_region[2] > 5:  # 宽度大于5才检测
            regions_to_check.append(("right", right_region))
    else:
        # 障碍物在右侧，检测左侧是否有白色赛道
        left_x = max(0, x - check_distance - 5)  # 向左检测，稍微偏移
        left_width = min(check_distance, x - 5)
        left_region = (left_x, y, left_width, h)

        if left_region[2] > 5:  # 宽度大于5才检测
            regions_to_check.append(("left", left_region))

    # 添加下方白色检测区域
    bottom_y = min(img_height, y + h + 5)  # 从障碍物下方开始稍微偏移
    bottom_height = min(15, img_height - bottom_y)  # 检测下方15像素高度
    if bottom_height > 5:  # 高度大于5才检测
        bottom_region = (x, bottom_y, w, bottom_height)
        regions_to_check.append(("bottom", bottom_region))

    for direction, region in regions_to_check:
        if region[2] > 0 and region[3] > 0:  # 确保区域有效
            total_regions += 1
            try:
                # 在该区域查找白色blob
                white_blobs = img.find_blobs([white_threshold],
                                           roi=region,
                                           pixels_threshold=30,
                                           area_threshold=50,
                                           merge=True)

                if white_blobs:
                    # 计算白色区域占该方向检测区域的比例
                    total_white_area = sum(blob.area() for blob in white_blobs)
                    region_area = region[2] * region[3]
                    white_ratio = total_white_area / region_area

                    if white_ratio > 0.6:  # 白色区域占比超过60%认为有赛道
                        white_regions_found += 1
            except:
                pass

    # 需要对侧和下方都有白色赛道才认为是有效障碍物
    is_on_track = white_regions_found >= 2 and total_regions >= 2

    return is_on_track, white_regions_found, total_regions

def update_detection_buffer(obstacle_detected, position=None):
    """更新检测缓冲区并判断是否确认检测"""
    global detection_buffer, last_confirmed_position

    # 添加当前检测结果到缓冲区
    detection_buffer.append({
        'detected': obstacle_detected,
        'position': position,
        'timestamp': time.ticks_ms()
    })

    # 保持缓冲区大小
    if len(detection_buffer) > buffer_size:
        detection_buffer.pop(0)

    # 如果缓冲区未满，不进行确认
    if len(detection_buffer) < buffer_size:
        return False, None

    # 统计最近几次检测中的有效检测次数
    valid_detections = 0
    position_counts = {'LEFT': 0, 'RIGHT': 0}

    for detection in detection_buffer:
        if detection['detected']:
            valid_detections += 1
            if detection['position']:
                position_counts[detection['position']] += 1

    # 判断是否达到确认阈值
    if valid_detections >= confirm_threshold:
        # 确定主要检测位置
        confirmed_position = 'LEFT' if position_counts['LEFT'] > position_counts['RIGHT'] else 'RIGHT'

        # 检查位置是否发生变化
        position_changed = last_confirmed_position != confirmed_position
        last_confirmed_position = confirmed_position

        return True, confirmed_position
    else:
        # 重置上次确认的位置
        last_confirmed_position = None
        return False, None

while(True):
    white.on()
    clock.tick()
    img = sensor.snapshot()

    # 检查定时器是否到期
    current_time = time.ticks_ms()
    if flag == 1 and time.ticks_diff(current_time, timer_start) >= timer_duration:
        flag = 0  # 3秒后重置flag，允许重新发送

    # 查找黑色区域的blob
    blobs = img.find_blobs([black_threshold],
                           pixels_threshold=400,     # 最小像素数
                           area_threshold=400,       # 最小面积
                           merge=True)              # 合并重叠的blob

        # 检查是否检测到有效障碍物
    current_obstacle_detected = False
    current_position = None
    current_blob_data = {}  # 存储当前检测到的障碍物数据
    
    for blob in blobs:
        # 面积过滤条件
        if blob.area() > 400:
            # 检查障碍物周围是否有白色赛道
            is_on_track, white_found, total_checked = check_track_around_obstacle(img, blob)
            
            if is_on_track:  # 只有在赛道上的障碍物才处理
                current_obstacle_detected = True
                
                # 判断障碍物位置（左侧还是右侧）
                obstacle_x = blob.cx()
                if obstacle_x < center_x:
                    current_position = "LEFT"
                else:
                    current_position = "RIGHT"
                
                # 存储当前检测的障碍物信息
                current_blob_data = {
                    'blob': blob,
                    'position': current_position,
                    'cx': obstacle_x,
                    'cy': blob.cy()
                }
                
                break  # 只处理第一个检测到的有效障碍物
    
    # 使用滤波机制确认检测结果
    confirmed, confirmed_position = update_detection_buffer(current_obstacle_detected, current_position)
    
        # 只有在确认检测到障碍物且当前确实有检测结果且位置一致时才绘制
    if confirmed and current_obstacle_detected and current_blob_data and current_blob_data['position'] == confirmed_position:
        # 使用当前检测到的障碍物数据
        blob = current_blob_data['blob']
        obstacle_x = current_blob_data['cx']
        obstacle_y = current_blob_data['cy']
        
        # 设置颜色和消息
        if confirmed_position == "LEFT":
            text_color = (0, 0, 255)  # 蓝色文字表示左侧
            rect_color = (0, 0, 255)  # 蓝色边框表示左侧
            uart_message = "obstacle-left\r\n"
        else:
            text_color = (255, 255, 0)  # 黄色文字表示右侧
            rect_color = (255, 255, 0)  # 黄色边框表示右侧
            uart_message = "obstacle-right\r\n"

        # 绘制障碍物检测框
        img.draw_rectangle(blob.rect(), color=rect_color, thickness=2)
        
        # 绘制障碍物中心点
        img.draw_circle(obstacle_x, obstacle_y, 5, color=rect_color, thickness=2)
        
        # 绘制十字标记中心点
        img.draw_line(obstacle_x-8, obstacle_y, obstacle_x+8, obstacle_y, color=rect_color, thickness=2)
        img.draw_line(obstacle_x, obstacle_y-8, obstacle_x, obstacle_y+8, color=rect_color, thickness=2)
        
        # 绘制位置文字
        img.draw_string(obstacle_x-15, obstacle_y-25, confirmed_position, color=text_color, scale=1)
        
        # 绘制障碍物信息（面积、坐标）
        info_text = f"Area:{blob.area()}"
        img.draw_string(obstacle_x-20, obstacle_y+15, info_text, color=text_color, scale=1)
        
        coord_text = f"({obstacle_x},{obstacle_y})"
        img.draw_string(obstacle_x-25, obstacle_y+25, coord_text, color=text_color, scale=1)
        
        # 显示滤波状态
        filter_text = f"Confirmed"
        img.draw_string(obstacle_x-25, obstacle_y+35, filter_text, color=(0, 255, 0), scale=1)

        # 只有在flag为0时才发送消息并启动定时器
        if flag == 0:
            # 通过串口发送障碍物位置信息
            uart2.write(uart_message)
            flag = 1  # 设置flag为1，防止重复发送
            timer_start = current_time  # 记录定时器开始时间

    # 绘制图像中心线作为参考
    img.draw_line(center_x, 0, center_x, img_height-1, color=(255, 255, 255), thickness=2)  # 白色中心线


