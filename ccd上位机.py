import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import re
import threading
import time
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import logging
import sys
import queue

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 全局变量
MAX_CCD_CHANNELS = 4  # 最大支持的CCD通道数
CCD_DATA_LENGTH = 128  # CCD数据长度
ccd_data_dict = {}     # 存储各通道CCD数据的字典
data_queues = {}       # 各通道数据队列
data_lock = threading.Lock()  # 线程锁
running = True         # 控制线程运行的标志
display_channels = []  # 当前显示的通道列表
update_flag = False    # 数据更新标志

# 初始化所有通道的数据
for i in range(1, MAX_CCD_CHANNELS + 1):
    channel = f"CCD{i}"
    ccd_data_dict[channel] = np.zeros(CCD_DATA_LENGTH)
    data_queues[channel] = queue.Queue(maxsize=10)  # 限制队列大小，防止内存占用过多

class CCDVisualizerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CCD 数据可视化系统")
        self.root.geometry("1000x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 设置中文字体
        plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
        plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号
        
        # 创建主框架
        self.main_frame = ttk.Frame(root, padding="10")
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建顶部控制面板
        self.control_frame = ttk.Frame(self.main_frame)
        self.control_frame.pack(fill=tk.X, pady=5)
        
        # 创建通道选择区
        self.channel_frame = ttk.LabelFrame(self.control_frame, text="选择显示通道")
        self.channel_frame.pack(side=tk.LEFT, padx=5)
        
        # 通道选择复选框
        self.channel_vars = []
        for i in range(1, MAX_CCD_CHANNELS + 1):
            var = tk.BooleanVar(value=i == 1)  # 默认选中第一个通道
            cb = ttk.Checkbutton(self.channel_frame, text=f"CCD{i}", variable=var, 
                                command=self.update_display_channels)
            cb.pack(side=tk.LEFT, padx=5)
            self.channel_vars.append(var)
        
        # 创建控制按钮
        self.button_frame = ttk.Frame(self.control_frame)
        self.button_frame.pack(side=tk.RIGHT, padx=5)
        
        self.start_button = ttk.Button(self.button_frame, text="开始", command=self.start_server)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(self.button_frame, text="停止", command=self.stop_server, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        self.exit_button = ttk.Button(self.button_frame, text="退出", command=self.on_closing)
        self.exit_button.pack(side=tk.LEFT, padx=5)
        
        # 创建状态栏
        self.status_var = tk.StringVar(value="就绪")
        self.status_bar = ttk.Label(self.main_frame, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # 创建图表区域 - 使用更高的DPI可能导致卡顿，降低DPI
        self.fig, self.ax = plt.subplots(figsize=(9, 5), dpi=80)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.main_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 使用blitting技术加速动画
        self.fig.canvas.draw()
        self.bg = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        
        # 初始化图表
        self.lines = {}
        self.setup_plot()
        
        # 初始化服务器线程
        self.server_thread = None
        self.data_thread = None
        
        # 初始化显示通道
        self.update_display_channels()
        
        # 使用matplotlib动画功能而不是tkinter的after
        self.animation = FuncAnimation(
            self.fig, self.update_plot, interval=100,  # 降低刷新率到10Hz
            blit=True, cache_frame_data=False)
        
        # 定期处理数据队列，但不直接更新UI
        self.root.after(50, self.process_data_queues)
    
    def setup_plot(self):
        """设置图表"""
        self.ax.clear()
        self.lines = {}
        
        x = np.arange(CCD_DATA_LENGTH)
        colors = ['b', 'r', 'g', 'm']
        
        for i in range(1, MAX_CCD_CHANNELS + 1):
            channel = f"CCD{i}"
            line, = self.ax.plot(x, ccd_data_dict[channel], f"{colors[i-1]}-", 
                               linewidth=2, label=channel)
            self.lines[channel] = line
            
        self.ax.set_title('CCD 数据实时显示')
        self.ax.set_xlabel('像素位置')
        self.ax.set_ylabel('像素值')
        self.ax.set_ylim(0, 4500)
        self.ax.grid(True)
        self.ax.legend()
        
        # 减少绘图元素，提高性能
        self.ax.set_rasterization_zorder(1)
        for spine in self.ax.spines.values():
            spine.set_visible(False)
    
    def update_display_channels(self):
        """更新要显示的通道列表"""
        global display_channels
        display_channels = []
        
        for i, var in enumerate(self.channel_vars):
            if var.get():
                channel = f"CCD{i+1}"
                display_channels.append(channel)
        
        # 更新图例可见性
        for i in range(1, MAX_CCD_CHANNELS + 1):
            channel = f"CCD{i}"
            if channel in self.lines:
                self.lines[channel].set_visible(channel in display_channels)
        
        # 强制重绘一次
        self.canvas.draw()
        logger.info(f"显示通道更新为: {display_channels}")
    
    def process_data_queues(self):
        """处理数据队列，但不直接更新UI"""
        global update_flag
        
        if running:
            # 更新所有可见通道的数据
            data_updated = False
            for channel in display_channels:
                if channel in self.lines and not data_queues[channel].empty():
                    try:
                        # 只获取最新的一个数据
                        while data_queues[channel].qsize() > 1:
                            data_queues[channel].get_nowait()  # 丢弃旧数据
                        
                        if not data_queues[channel].empty():
                            data = data_queues[channel].get_nowait()
                            with data_lock:
                                ccd_data_dict[channel] = data.copy()
                            data_updated = True
                    except queue.Empty:
                        pass
            
            if data_updated:
                update_flag = True
            
            # 继续定期处理队列
            self.root.after(50, self.process_data_queues)
    
    def update_plot(self, frame):
        """更新图表 - 由FuncAnimation调用"""
        global update_flag
        
        # 只有在数据更新时才更新图表
        if not update_flag:
            return [line for line in self.lines.values()]
        
        # 重置更新标志
        update_flag = False
        
        # 更新所有可见通道的线条数据
        for channel in display_channels:
            if channel in self.lines:
                with data_lock:
                    self.lines[channel].set_ydata(ccd_data_dict[channel])
        
        # 返回所有更新的线条对象，用于blitting
        return [line for line in self.lines.values()]
    
    def start_server(self):
        """启动服务器"""
        global running
        running = True
        
        # 创建并启动服务器线程
        self.server_thread = threading.Thread(target=server_thread)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.status_var.set("服务器运行中...")
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
    
    def stop_server(self):
        """停止服务器"""
        global running
        running = False
        
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)
        
        self.status_var.set("服务器已停止")
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
    
    def on_closing(self):
        """窗口关闭时的处理"""
        global running
        running = False
        
        # 停止所有线程
        self.stop_server()
        
        # 停止动画
        self.animation.event_source.stop()
        
        # 清理资源
        plt.close(self.fig)
        
        # 关闭窗口
        self.root.destroy()
        
        # 确保程序完全退出
        logger.info("程序正在退出...")
        sys.exit(0)

def parse_ccd_data(message):
    """解析CCD数据字符串，返回通道名和数值数组"""
    # 查找CCDx:后面的数据
    match = re.search(r'(CCD\d+):([\d,]+)', message)
    if not match:
        return None, None
    
    channel = match.group(1)
    data_str = match.group(2)
    
    try:
        # 解析数据字符串为数值列表
        values = [int(x) for x in data_str.split(',') if x.strip()]
        
        # 确保数据长度不超过CCD_DATA_LENGTH
        return channel, values[:CCD_DATA_LENGTH]
    except Exception as e:
        logger.error(f"解析数据出错: {e}")
        return None, None

def server_thread():
    """服务器线程函数，接收数据"""
    global running
    
    HOST = '0.0.0.0'  # 监听所有可用的网络接口
    PORT = 8086       # 监听端口
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # 设置socket选项，允许地址重用
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            s.bind((HOST, PORT))
            s.listen(5)
            logger.info(f"服务器已启动，正在监听端口 {PORT}...")
            
            while running:
                # 设置超时，以便能够检查running标志
                s.settimeout(0.5)
                try:
                    conn, addr = s.accept()
                    logger.info(f"连接来自: {addr}")
                    
                    # 为每个连接创建一个处理线程
                    client_thread = threading.Thread(
                        target=handle_client_connection,
                        args=(conn, addr)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    logger.error(f"接受连接时出错: {e}")
                    
        except Exception as e:
            logger.error(f"服务器错误: {e}")
        finally:
            logger.info("服务器关闭")

def handle_client_connection(conn, addr):
    """处理客户端连接"""
    global running
    buffer = ""
    
    try:
        conn.settimeout(0.5)
        while running:
            try:
                data = conn.recv(1024)
                if not data:
                    break
                
                message = data.decode('utf-8', errors='ignore')
                buffer += message
                
                # 处理可能的分段消息
                process_buffer(buffer)
                
                # 保留最后一个可能不完整的部分
                last_ccd_pos = buffer.rfind("CCD")
                if last_ccd_pos >= 0:
                    buffer = buffer[last_ccd_pos:]
                else:
                    buffer = ""
                    
            except socket.timeout:
                continue
            except Exception as e:
                logger.error(f"接收数据时出错: {e}")
                break
    finally:
        conn.close()
        logger.info(f"连接关闭: {addr}")

def process_buffer(buffer):
    """处理数据缓冲区"""
    # 查找所有可能的CCD数据
    for channel_prefix in [f"CCD{i}" for i in range(1, MAX_CCD_CHANNELS + 1)]:
        if channel_prefix in buffer:
            parts = buffer.split(channel_prefix)
            
            for i in range(1, len(parts)):
                part = channel_prefix + parts[i]
                channel, values = parse_ccd_data(part)
                
                if channel and values and len(values) > 0:
                    # 使用队列传递数据，避免阻塞
                    try:
                        # 如果队列满了，丢弃旧数据
                        if data_queues[channel].full():
                            data_queues[channel].get_nowait()
                        
                        # 填充数据数组
                        data_array = np.zeros(CCD_DATA_LENGTH)
                        length = min(len(values), CCD_DATA_LENGTH)
                        data_array[:length] = values[:length]
                        
                        # 放入队列
                        data_queues[channel].put_nowait(data_array)
                    except Exception as e:
                        logger.error(f"处理通道 {channel} 数据时出错: {e}")

def main():
    """主函数"""
    # 设置Tkinter的事件循环优先级
    root = tk.Tk()
    root.update_idletasks()
    
    # 在Windows上可以尝试提高进程优先级
    try:
        import ctypes
        ctypes.windll.kernel32.SetThreadPriority(
            ctypes.windll.kernel32.GetCurrentThread(), 
            ctypes.c_int(2)  # THREAD_PRIORITY_ABOVE_NORMAL
        )
    except:
        pass
    
    app = CCDVisualizerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
