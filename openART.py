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

def check_track_around_obstacle(img, blob):
    """检查障碍物周围是否有白色赛道"""
    x, y, w, h = blob.rect()
    
    # 定义检测区域的偏移量
    check_distance = 15  # 检测距离
    
    # 上方检测区域
    top_y = max(0, y - check_distance)
    top_region = (x, top_y, w, min(check_distance, y))
    
    # 下方检测区域  
    bottom_y = min(img_height, y + h)
    bottom_region = (x, bottom_y, w, min(check_distance, img_height - bottom_y))
    
    # 左侧检测区域
    left_x = max(0, x - check_distance)
    left_region = (left_x, y, min(check_distance, x), h)
    
    # 右侧检测区域
    right_x = min(img_width, x + w)
    right_region = (right_x, y, min(check_distance, img_width - right_x), h)
    
    white_regions_found = 0
    total_regions = 0
    
    # 检查各个方向的白色区域
    regions_to_check = []
    
    # 上方区域
    if top_region[3] > 5:  # 高度大于5才检测
        regions_to_check.append(("top", top_region))
    
    # 下方区域
    if bottom_region[3] > 5:  # 高度大于5才检测
        regions_to_check.append(("bottom", bottom_region))
    
    # 左侧区域
    if left_region[2] > 5:  # 宽度大于5才检测
        regions_to_check.append(("left", left_region))
    
    # 右侧区域
    if right_region[2] > 5:  # 宽度大于5才检测
        regions_to_check.append(("right", right_region))
    
    for direction, region in regions_to_check:
        if region[2] > 0 and region[3] > 0:  # 确保区域有效
            total_regions += 1
            try:
                # 在该区域查找白色blob
                white_blobs = img.find_blobs([white_threshold], 
                                           roi=region,
                                           pixels_threshold=20,
                                           area_threshold=30,
                                           merge=True)
                
                if white_blobs:
                    # 计算白色区域占该方向检测区域的比例
                    total_white_area = sum(blob.area() for blob in white_blobs)
                    region_area = region[2] * region[3]
                    white_ratio = total_white_area / region_area
                    
                    if white_ratio > 0.3:  # 白色区域占比超过30%认为有赛道
                        white_regions_found += 1
                        
                        # 绘制检测到白色赛道的区域（调试用）
                        img.draw_rectangle(region, color=(0, 255, 0), thickness=1)
                    else:
                        # 绘制未检测到足够白色的区域（调试用）
                        img.draw_rectangle(region, color=(255, 0, 0), thickness=1)
                else:
                    # 绘制没有白色blob的区域（调试用）
                    img.draw_rectangle(region, color=(255, 0, 0), thickness=1)
            except:
                pass
    
    # 至少需要2个方向有白色赛道才认为是有效障碍物
    is_on_track = white_regions_found >= 2 and total_regions >= 2
    
    return is_on_track, white_regions_found, total_regions

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
    obstacle_detected = False
    
    for blob in blobs:
        # 面积过滤条件
        if blob.area() > 400:
            # 检查障碍物周围是否有白色赛道
            is_on_track, white_found, total_checked = check_track_around_obstacle(img, blob)
            
            if is_on_track:  # 只有在赛道上的障碍物才处理
                obstacle_detected = True  # 标记检测到障碍物

                # 计算中心点
                obstacle_x = blob.cx()
                obstacle_y = blob.cy()
                
                # 判断障碍物位置（左侧还是右侧）
                if obstacle_x < center_x:
                    position = "LEFT"
                    text_color = (0, 0, 255)  # 蓝色文字表示左侧
                    rect_color = (0, 0, 255)  # 蓝色边框表示左侧
                    uart_message = "obstacle-left\r\n"
                else:
                    position = "RIGHT"
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
                img.draw_string(obstacle_x-15, obstacle_y-25, position, color=text_color, scale=1)
                
                # 绘制障碍物信息（面积、坐标、赛道检测结果）
                info_text = f"Area:{blob.area()}"
                img.draw_string(obstacle_x-20, obstacle_y+15, info_text, color=text_color, scale=1)
                
                coord_text = f"({obstacle_x},{obstacle_y})"
                img.draw_string(obstacle_x-25, obstacle_y+25, coord_text, color=text_color, scale=1)
                
                # 显示赛道检测结果
                track_text = f"Track:{white_found}/{total_checked}"
                img.draw_string(obstacle_x-25, obstacle_y+35, track_text, color=(0, 255, 0), scale=1)

                # 只有在flag为0时才发送消息并启动定时器
                if flag == 0:
                    # 通过串口发送障碍物位置信息
                    uart2.write(uart_message)
                    flag = 1  # 设置flag为1，防止重复发送
                    timer_start = current_time  # 记录定时器开始时间
                
                break  # 只处理第一个检测到的有效障碍物
            else:
                # 绘制无效障碍物（不在赛道上）
                img.draw_rectangle(blob.rect(), color=(128, 128, 128), thickness=1)  # 灰色表示无效
                invalid_text = f"Invalid:{white_found}/{total_checked}"
                img.draw_string(blob.cx()-30, blob.cy(), invalid_text, color=(128, 128, 128), scale=1)
    
    # 绘制图像中心线作为参考
    img.draw_line(center_x, 0, center_x, img_height-1, color=(255, 255, 255), thickness=2)  # 白色中心线
    
    # 绘制状态信息
    # 显示定时器状态
    if flag == 1:
        remaining_time = max(0, timer_duration - time.ticks_diff(current_time, timer_start))
        status_text = f"SENT - Timer:{remaining_time}ms"
        status_color = (255, 0, 0)  # 红色表示已发送，等待定时器
    else:
        status_text = "READY"
        status_color = (0, 255, 0)  # 绿色表示准备发送
    
    img.draw_string(5, 5, status_text, color=status_color, scale=1)
    
    # 显示检测状态
    if obstacle_detected:
        detect_text = "OBSTACLE ON TRACK"
        detect_color = (255, 255, 0)  # 黄色
    else:
        detect_text = "NO VALID OBSTACLE"
        detect_color = (0, 255, 255)  # 青色
    
    img.draw_string(5, 15, detect_text, color=detect_color, scale=1)
    
    # 显示帧率
    fps_text = f"FPS:{clock.fps():.1f}"
    img.draw_string(5, img_height-15, fps_text, color=(255, 255, 255), scale=1)


