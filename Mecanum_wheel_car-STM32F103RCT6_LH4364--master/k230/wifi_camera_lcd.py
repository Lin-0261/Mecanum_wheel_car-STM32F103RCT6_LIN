# WiFi + Camera + LCD Demo for K230
# 连接WiFi，将摄像头画面显示在LCD屏幕上
import time, os, sys
import network

from media.sensor import *
from media.display import *
from media.media import *

# ========== WiFi 配置 ==========
SSID = "LH_TX"
PASSWORD = "LH544364"

print("正在连接 WiFi: %s ..." % SSID)
print("密码: %s" % PASSWORD)

# STA 模式
sta = network.WLAN(network.STA_IF)

# 先断开旧连接，清理状态
print("断开旧连接...")
sta.disconnect()
time.sleep(1)

# 打印连接前状态
print("sta.status() =", sta.status(), "(0=IDLE, 1=CONNECTING, 2=CONNECTED, 3=DISCONNECTING)")

# 尝试连接 WiFi
print("开始 connect()...")
try:
    sta.connect(SSID, PASSWORD)
    print("connect() 返回成功, sta.status() =", sta.status())
except Exception as e:
    print("connect() 异常:", e)

# 等待连接，最多 20 秒
print("等待连接", end="")
for _ in range(40):  # 20 seconds
    if sta.ifconfig()[0] != '0.0.0.0':
        break
    if sta.status() == 2:  # CONNECTED but no IP yet
        print("*", end="")
    else:
        print(".", end="")
    time.sleep(0.5)
print()

# 最终结果
status = sta.status()
print("最终 sta.status() =", status)
print("最终 sta.ifconfig() =", sta.ifconfig())
print("最终 sta.isconnected() =", sta.isconnected())

if sta.ifconfig()[0] != '0.0.0.0':
    ip, mask, gw, dns = sta.ifconfig()
    print("========================================")
    print("  WiFi 连接成功！")
    print("  SSID: %s" % SSID)
    print("  IP 地址: %s" % ip)
    print("  子网掩码: %s" % mask)
    print("  网关: %s" % gw)
    print("  DNS: %s" % dns)
    print("========================================")
else:
    print("========================================")
    print("  WiFi 连接失败！")
    print("  status=%d (0=IDLE, 1=LINKING, 2=OK, 3=DISCONNECTING)" % status)
    print("========================================")
    print("  请排查：")
    print("  1. '%s' 是否为 2.4G WiFi（K230 不支持 5G）" % SSID)
    print("  2. 开发板 WiFi 天线是否连接")
    print("  3. 尝试先运行官方示例测试：")
    print("     修改 network_wlan_sta.py 中的 SSID/PASSWORD 后运行")
    print("========================================")
    print()
    print("WiFi 未连接，继续启动摄像头...")

# ========== 摄像头 + LCD 显示 ==========
sensor = None

try:
    print("初始化摄像头...")

    # 构建 Sensor 对象
    sensor = Sensor()
    # 复位 sensor
    sensor.reset()

    # 设置通道0输出分辨率，800x480 匹配 LCD
    sensor.set_framesize(width=800, height=480)
    # 设置通道0输出格式
    sensor.set_pixformat(Sensor.YUV420SP)
    # 绑定 sensor 通道0 到显示层 video1
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # 使用 LCD (ST7701) 作为显示输出，800x480
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)

    # 启动 sensor
    sensor.run()

    print("摄像头已启动，LCD 显示中...")

    # 主循环
    while True:
        os.exitpoint()

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    sys.print_exception(e)
finally:
    # 停止 sensor
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 反初始化显示
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    print("程序退出。")
