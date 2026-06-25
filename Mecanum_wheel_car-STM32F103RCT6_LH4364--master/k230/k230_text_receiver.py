# K230 WiFi 文字接收 + 摄像头 LCD 显示
# 通过 TCP 接收 PC 发来的文字，显示在 LCD 的 OSD 层上
import time, os, sys
import socket
import network
import image

from media.sensor import *
from media.display import *
from media.media import *

# ========== WiFi 配置 ==========
SSID = "LH_TX"
PASSWORD = "LH544364"

print("正在连接 WiFi: %s ..." % SSID)

sta = network.WLAN(network.STA_IF)
sta.disconnect()
time.sleep(1)

sta.connect(SSID, PASSWORD)

print("等待获取 IP", end="")
for _ in range(30):
    if sta.ifconfig()[0] != '0.0.0.0':
        break
    print(".", end="")
    time.sleep(0.5)
print()

if sta.ifconfig()[0] != '0.0.0.0':
    ip, mask, gw, dns = sta.ifconfig()
    print("========================================")
    print("  WiFi 连接成功！")
    print("  IP 地址: %s" % ip)
    print("========================================")
else:
    ip = "0.0.0.0"
    print("WiFi 连接失败！")

TCP_PORT = 8080

# ========== 摄像头 + LCD + TCP 服务端 ==========
sensor_obj = None
server_sock = None
client_sock = None
osd_img = None
received_text = "等待消息..."  # 当前显示的文字

try:
    # ----- 初始化摄像头 -----
    print("初始化摄像头...")
    sensor_obj = Sensor()
    sensor_obj.reset()
    sensor_obj.set_framesize(width=800, height=480)
    sensor_obj.set_pixformat(Sensor.YUV420SP)
    bind_info = sensor_obj.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # 初始化 LCD，启用 1 个 OSD 层用于显示文字
    Display.init(Display.ST7701, width=800, height=480, to_ide=True, osd_num=1)

    # 创建 OSD 图像（用于显示接收到的文字）
    osd_img = image.Image(800, 480, image.ARGB8888)

    sensor_obj.run()
    print("摄像头已启动，LCD 显示中...")

    # ----- 初始化 TCP 服务端 -----
    if ip != "0.0.0.0":
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ai = socket.getaddrinfo(ip, TCP_PORT)
        addr = ai[0][-1]
        server_sock.bind(addr)
        server_sock.settimeout(0)  # 非阻塞模式
        server_sock.listen(1)
        print("TCP 服务端已启动: %s:%d" % (ip, TCP_PORT))
        print("等待 PC 客户端连接...")
    else:
        print("WiFi 未连接，TCP 服务端未启动，仅显示摄像头画面")

    # ========== 主循环 ==========
    while True:
        os.exitpoint()

        # --- 处理 TCP 连接和消息 ---
        if server_sock:
            try:
                res = server_sock.accept()
                if res:
                    old_client = client_sock
                    client_sock = res[0]
                    client_addr = res[1]
                    client_sock.settimeout(0)
                    print("客户端已连接:", client_addr)
                    if old_client:
                        try:
                            old_client.close()
                        except:
                            pass
            except Exception as e:
                # errno 11 = EAGAIN, 无连接等待
                if hasattr(e, 'errno') and e.errno != 11:
                    print("accept error:", e)

        if client_sock:
            try:
                data = client_sock.read()
                if data and data != b"":
                    text = data.decode("utf-8").strip()
                    print("收到:", text)
                    received_text = text  # 更新显示文字
            except Exception as e:
                if hasattr(e, 'errno') and e.errno != 11:
                    print("read error:", e)

        # --- 更新 OSD 显示 ---
        if osd_img:
            osd_img.clear()

            # 半透明背景条（顶部）
            for y in range(60):
                for x in range(800):
                    # ARGB: 半透明深色背景
                    osd_img.set_pixel(x, y, (128, 0, 0, 0))

            # 显示标题
            osd_img.draw_string_advanced(
                10, 5, 28,
                "TCP Message:",
                color=(255, 255, 255, 0)  # ARGB: 黄色
            )

            # 显示接收到的文字（自动换行处理）
            max_chars_per_line = 40
            text = received_text
            lines = []
            while len(text) > max_chars_per_line:
                lines.append(text[:max_chars_per_line])
                text = text[max_chars_per_line:]
            lines.append(text)

            y_offset = 40
            for line in lines[:12]:  # 最多显示 12 行
                osd_img.draw_string_advanced(
                    10, y_offset, 24,
                    line,
                    color=(255, 255, 255, 255)  # ARGB: 白色
                )
                y_offset += 30

            # 底部显示 IP 信息
            osd_img.draw_string_advanced(
                10, 450, 18,
                "IP: %s:%d" % (ip, TCP_PORT),
                color=(255, 180, 180, 180)  # ARGB: 灰色
            )

            Display.show_image(osd_img, 0, 0, Display.LAYER_OSD1)

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    sys.print_exception(e)
finally:
    # 清理
    if client_sock:
        try:
            client_sock.close()
        except:
            pass
    if server_sock:
        try:
            server_sock.close()
        except:
            pass
    if isinstance(sensor_obj, Sensor):
        sensor_obj.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    print("程序退出。")
