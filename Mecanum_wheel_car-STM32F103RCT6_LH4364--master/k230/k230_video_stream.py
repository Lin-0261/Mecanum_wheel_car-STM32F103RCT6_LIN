# K230 无线图传服务端 - 双通道方案 + UDP 速度指令接收
# Ch0: YUV→LCD 硬件直连 (800x480)
# Ch1: RGB→JPEG→TCP 图传 (640x360)
# UDP: 端口 8889 接收上位机速度指令 "Vx,Vy,w\n" 显示在 LCD OSD
# UART2: 转发速度指令到 STM32, 帧格式 0x55 0x54 Vx Vy w checksum
import time, os, sys
import socket
import struct
import network
import image
import gc

from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA

# ========== WiFi 配置 ==========
SSID = "LIN 8778"
PASSWORD = "20050214"
TCP_PORT = 8888
UDP_PORT = 8889

# 12V供电时等待电源稳定 (给DCDC降压芯片充足的启动时间)
print("等待电源稳定 (3秒)...")
time.sleep(3)

# WiFi 连接 (带重试)
MAX_WIFI_RETRIES = 3
ip = "0.0.0.0"

for retry in range(MAX_WIFI_RETRIES):
    if retry > 0:
        print("WiFi 重试 %d/%d ..." % (retry + 1, MAX_WIFI_RETRIES))
        sta = network.WLAN(network.STA_IF)
        sta.disconnect()
        time.sleep(2)

    print("正在连接 WiFi: %s ..." % SSID)
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    sta.disconnect()
    time.sleep(1)
    sta.connect(SSID, PASSWORD)

    print("等待获取 IP", end="")
    for _ in range(40):  # 增加超时到 20 秒
        ifconfig = sta.ifconfig()
        if ifconfig is not None and len(ifconfig) > 0 and ifconfig[0] != '0.0.0.0':
            ip = ifconfig[0]
            break
        print(".", end="")
        time.sleep(0.5)
    print()

    if ip != "0.0.0.0":
        break

if ip != "0.0.0.0":
    print("========================================")
    print("  WiFi 连接成功！IP: %s" % ip)
    print("========================================")
else:
    print("WiFi 连接失败 (尝试%d次)！继续运行 (无网络功能)" % MAX_WIFI_RETRIES)

# ========== 参数 ==========
LCD_W, LCD_H = 800, 480
STREAM_W, STREAM_H = 240, 180
JPEG_QUALITY = 10              # 降低画质以减小帧体积、加快发送

sensor_obj = None
server_sock = None
client_sock = None
udp_sock = None
osd_img = None

last_status = ""
last_fps_time = time.time()
frame_count = 0
fps_show = 0
osd_needs_update = True

# 速度指令变量
vx_cmd = 0.0
vy_cmd = 0.0
w_cmd = 0.0
cmd_updated = False

# ===== UART2 发送函数 (定义在前，初始化在后) =====
uart2 = None
uart_send_count = 0
uart_send_errors = 0
last_send_debug = 0

def send_to_stm32(vx, vy, w):
    """通过 UART2 发送速度指令到 STM32
    帧格式: 0x55 0x54 Vx(int16) Vy(int16) w(int16*1000) checksum
    """
    global uart_send_count, uart_send_errors, last_send_debug
    if uart2 is None:
        return
    try:
        vx_i = int(vx)
        vy_i = int(vy)
        w_i = int(w * 1000)  # rad/s → milli-rad/s (e.g. 5.0 → 5000)
        payload = struct.pack('>hhh', vx_i, vy_i, w_i)
        checksum = 0
        for b in payload:
            checksum ^= b
        checksum &= 0xFF
        frame = b'\x55\x54' + payload + bytes([checksum])
        n = uart2.write(frame)
        uart_send_count += 1

        # 每 2 秒打印一次调试信息
        now = time.ticks_ms()
        if now - last_send_debug > 2000:
            last_send_debug = now
            hex_str = ' '.join('%02X' % b for b in frame)
            print("UART2 TX #%d (%d bytes): %s" % (uart_send_count, n, hex_str))
    except Exception as e:
        uart_send_errors += 1
        if uart_send_errors <= 3:
            import sys as _sys2
            print("UART2 TX error:")
            _sys2.print_exception(e)


# ===== UART2 初始化 =====
# 参照官方示例: FPIOA 先配置引脚功能, 再创建 UART
try:
    import sys as _sys
    fpioa = FPIOA()
    # GPIO11 → UART2_TXD, GPIO12 → UART2_RXD (参照 uart2.py 示例)
    fpioa.set_function(11, fpioa.UART2_TXD)
    fpioa.set_function(12, fpioa.UART2_RXD)
    print("FPIOA: GPIO11→UART2_TX, GPIO12→UART2_RX")

    # 使用模块常量: UART.UART2, UART.EIGHTBITS, 等 (参照 uart.py 示例)
    uart2 = UART(UART.UART2, baudrate=115200,
                 bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                 stop=UART.STOPBITS_ONE)
    print("UART2 已初始化! 连接 STM32 (TX=GPIO5, RX=GPIO6)")

    # 发送测试帧验证链路
    send_to_stm32(100.0, 0.0, 0.0)
    time.sleep_ms(100)
    send_to_stm32(0.0, 100.0, 0.0)
    time.sleep_ms(100)
    send_to_stm32(0.0, 0.0, 1.0)
    print("UART2 测试帧已发送")
except Exception as e:
    _sys.print_exception(e)
    print("UART2 初始化失败!")

try:
    # ----- 初始化摄像头 -----
    print("初始化摄像头...")
    sensor_obj = Sensor()
    sensor_obj.reset()

    # Ch0: LCD 硬件直连 (YUV420SP → Display)
    sensor_obj.set_framesize(width=LCD_W, height=LCD_H, chn=CAM_CHN_ID_0)
    sensor_obj.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    bind_info = sensor_obj.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # Ch1: 图传抓帧 (RGB888 → JPEG)
    sensor_obj.set_framesize(width=STREAM_W, height=STREAM_H, chn=CAM_CHN_ID_1)
    sensor_obj.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_1)

    # LCD 初始化，开 OSD 层显示状态
    Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=True, osd_num=1)
    osd_img = image.Image(LCD_W, LCD_H, image.ARGB8888)

    sensor_obj.run()
    print("摄像头已启动")

    # ----- TCP 服务端 -----
    if ip != "0.0.0.0":
        addr = socket.getaddrinfo(ip, TCP_PORT)[0][-1]
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(addr)
        server_sock.listen(1)
        server_sock.settimeout(0)
        print("TCP 图传: %s:%d  分辨率: %dx%d" % (ip, TCP_PORT, STREAM_W, STREAM_H))

        # ----- UDP 控制指令接收 -----
        udp_addr = socket.getaddrinfo(ip, UDP_PORT)[0][-1]
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_sock.bind(udp_addr)
        udp_sock.settimeout(0)
        print("UDP 控制: %s:%d" % (ip, UDP_PORT))
    else:
        print("WiFi 未连接，仅本地显示")

    # ========== UDP 接收函数（多处调用，降低控制延迟） ==========
    def check_udp_control():
        global vx_cmd, vy_cmd, w_cmd, cmd_updated, osd_needs_update
        if not udp_sock:
            return
        try:
            data, addr_from = udp_sock.recvfrom(128)
            if data:
                try:
                    text = data.decode('utf-8').strip()
                    parts = text.split(',')
                    if len(parts) == 3:
                        vx_cmd = float(parts[0])
                        vy_cmd = float(parts[1])
                        w_cmd = float(parts[2])
                        cmd_updated = True
                        osd_needs_update = True
                        send_to_stm32(vx_cmd, vy_cmd, w_cmd)
                except Exception:
                    pass
        except Exception as e:
            if hasattr(e, 'errno') and e.errno != 11:
                pass

    # ========== 主循环（控制优先，图传为后台任务） ==========
    while True:
        os.exitpoint()

        # --- 优先级1: 检查 UDP 控制指令 ---
        check_udp_control()

        # --- 接受 TCP 客户端 ---
        if server_sock:
            try:
                res = server_sock.accept()
                if res:
                    if client_sock:
                        try: client_sock.close()
                        except: pass
                    client_sock = res[0]
                    print("客户端连接: %s" % str(res[1]))
                    client_sock.settimeout(0)
                    osd_needs_update = True
            except Exception as e:
                if hasattr(e, 'errno') and e.errno != 11:
                    pass

        # --- 优先级2: 再次检查 UDP（TCP accept 可能耗时） ---
        check_udp_control()

        # --- 图传: 抓帧 + 压缩 + 发送（后台任务） ---
        if client_sock:
            try:
                img = sensor_obj.snapshot(chn=CAM_CHN_ID_1)

                # 抓帧后立即再查一次 UDP，避免指令在压缩期间排队
                check_udp_control()

                jpeg = img.compress(quality=JPEG_QUALITY)
                jpeg_bytes = jpeg.to_bytes() if hasattr(jpeg, 'to_bytes') else bytes(jpeg)

                if jpeg_bytes and len(jpeg_bytes) > 0:
                    header = len(jpeg_bytes).to_bytes(4, 'big')
                    try:
                        client_sock.send(header + jpeg_bytes)
                        frame_count += 1
                    except:
                        print("客户端断开")
                        try: client_sock.close()
                        except: pass
                        client_sock = None
                        osd_needs_update = True

                # TCP 发送后最后查一次 UDP
                check_udp_control()

            except Exception as e:
                if hasattr(e, 'errno') and e.errno != 11:
                    pass
        else:
            # 无图传客户端时也保持检查 UDP
            check_udp_control()

        # --- FPS 计算 ---
        now = time.time()
        if now - last_fps_time >= 1.0:
            fps_show = frame_count
            frame_count = 0
            last_fps_time = now
            osd_needs_update = True

        # --- OSD 刷新 ---
        if osd_needs_update and osd_img:
            osd_needs_update = False
            osd_img.clear()

            # 顶部状态栏背景
            osd_img.draw_rectangle(0, 0, LCD_W, 56, color=(180, 0, 0, 0), fill=True)

            # 第1行: FPS + 分辨率 + 质量
            status = "FPS:%d | %dx%d | JPEG Q:%d" % (fps_show, STREAM_W, STREAM_H, JPEG_QUALITY)
            osd_img.draw_string_advanced(8, 4, 20, status, color=(255, 0, 255, 0))

            # 第2行: IP + 连接状态
            ip_info = "%s:%d (TCP)" % (ip, TCP_PORT) if ip != "0.0.0.0" else "No WiFi"
            osd_img.draw_string_advanced(8, 26, 14, ip_info, color=(255, 200, 200, 200))

            # 客户端状态
            if client_sock:
                osd_img.draw_string_advanced(650, 4, 16, "VIDEO:CONN", color=(255, 0, 255, 0))
            else:
                osd_img.draw_string_advanced(650, 4, 16, "VIDEO:WAIT", color=(255, 255, 100, 100))

            # UDP 状态
            if vx_cmd != 0 or vy_cmd != 0 or w_cmd != 0 or cmd_updated:
                osd_img.draw_string_advanced(650, 22, 16, "CTRL:ACTIVE", color=(255, 100, 255, 100))
            else:
                osd_img.draw_string_advanced(650, 22, 16, "CTRL:IDLE", color=(255, 150, 150, 150))

            # UART2 状态 + 发送计数
            if uart2:
                txt = "UART2:OK #%d" % uart_send_count
                osd_img.draw_string_advanced(650, 40, 14, txt, color=(255, 100, 255, 100))
            else:
                osd_img.draw_string_advanced(650, 40, 14, "UART2:--", color=(255, 255, 100, 100))

            # ---- 速度指令显示 (LCD 中央醒目位置) ----
            # 半透明背景面板
            osd_img.draw_rectangle(180, 180, 440, 140, color=(180, 20, 20, 40), fill=True)
            osd_img.draw_rectangle(180, 180, 440, 140, color=(255, 100, 100, 120), fill=False, thickness=2)

            # 标题
            osd_img.draw_string_advanced(340, 186, 20, "SPEED COMMAND", color=(255, 255, 255, 100))

            # Vx
            vx_color = (255, 0, 255, 0) if vx_cmd >= 0 else (255, 255, 100, 100)
            osd_img.draw_string_advanced(210, 214, 28, "Vx: %+6.0f" % vx_cmd, color=vx_color)

            # Vy
            vy_color = (255, 0, 255, 0) if vy_cmd >= 0 else (255, 255, 100, 100)
            osd_img.draw_string_advanced(210, 246, 28, "Vy: %+6.0f" % vy_cmd, color=vy_color)

            # w (角速度)
            w_color = (255, 0, 255, 255) if w_cmd >= 0 else (255, 255, 160, 100)
            osd_img.draw_string_advanced(210, 278, 28, "w : %+6.0f" % w_cmd, color=w_color)

            # 矢量方向指示 (简单箭头)
            cx_v = 530
            cy_v = 250
            if abs(vx_cmd) > 0.1 or abs(vy_cmd) > 0.1:
                mag = (vx_cmd**2 + vy_cmd**2) ** 0.5
                scale = min(45.0 / max(mag, 1.0), 45.0)
                ex = int(cx_v + vx_cmd * scale / 50.0)
                ey = int(cy_v - vy_cmd * scale / 50.0)  # Y反转
                osd_img.draw_line(cx_v, cy_v, ex, ey, color=(255, 0, 255, 0), thickness=3)
                # 箭头头部
                osd_img.draw_circle(ex, ey, 4, color=(255, 0, 255, 0), fill=True)
            osd_img.draw_circle(cx_v, cy_v, 6, color=(255, 150, 150, 150), fill=False, thickness=2)
            osd_img.draw_string_advanced(545, 234, 12, "dir", color=(255, 150, 150, 150))

            Display.show_image(osd_img, 0, 0, Display.LAYER_OSD1)

        # 周期刷新 OSD
        if frame_count % 5 == 0:
            osd_needs_update = True

except KeyboardInterrupt as e:
    print("用户停止")
except BaseException as e:
    sys.print_exception(e)
finally:
    if client_sock:
        try: client_sock.close()
        except: pass
    if server_sock:
        try: server_sock.close()
        except: pass
    if udp_sock:
        try: udp_sock.close()
        except: pass
    if isinstance(sensor_obj, Sensor):
        sensor_obj.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    print("退出。")
