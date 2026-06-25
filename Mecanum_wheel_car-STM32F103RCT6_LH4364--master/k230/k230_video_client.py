# PC 端无线图传客户端 - 接收 TCP+JPEG 流并实时显示
# 支持断线自动重连
#
# 依赖: pip install opencv-python numpy
#
# 使用方式:
#   python k230_video_client.py
#   python k230_video_client.py 192.168.137.36
#   python k230_video_client.py 192.168.137.36 8888

import socket
import sys
import time
import struct

try:
    import cv2
    import numpy as np
except ImportError:
    print("请先安装: pip install opencv-python numpy")
    sys.exit(1)

K230_IP = "192.168.137.197"
K230_PORT = 8888
RECONNECT_DELAY = 2  # 重连间隔（秒）


def recv_exact(sock, n):
    """精确接收 n 字节"""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("连接断开")
        buf += chunk
    return buf


def connect_to_k230(ip, port, timeout=3):
    """尝试连接 K230，成功返回 socket，失败返回 None"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((ip, port))
        sock.settimeout(8)  # 接收超时
        return sock
    except Exception:
        return None


def show_reconnect_screen(msg, attempt=0):
    """显示重连等待画面"""
    canvas = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(canvas, "Connection Lost", (140, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (100, 100, 255), 2)
    cv2.putText(canvas, msg, (100, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
    if attempt > 0:
        cv2.putText(canvas, "Retry: %d" % attempt, (240, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    cv2.putText(canvas, "Press 'q' to quit", (220, 310),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
    cv2.imshow("K230 Video Stream", canvas)
    cv2.waitKey(1)


def main(ip, port):
    sock = None
    reconnect_attempt = 0

    cv2.namedWindow("K230 Video Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("K230 Video Stream", 640, 360)

    frame_count = 0
    fps_time = time.time()
    fps_show = 0
    writer = None
    recording = False

    # ----- 首次连接 -----
    print("连接 %s:%d ..." % (ip, port))
    sock = connect_to_k230(ip, port)
    if sock:
        print("已连接！q=退出 s=截图 r=录制")
    else:
        print("首次连接失败，进入自动重连模式...")
        show_reconnect_screen("Connecting to %s:%d ..." % (ip, port))

    while True:
        # ========== 未连接状态：自动重连 ==========
        if sock is None:
            reconnect_attempt += 1
            info = "Reconnecting to %s:%d ..." % (ip, port)
            print("%s (第 %d 次)" % (info, reconnect_attempt))
            show_reconnect_screen(info, reconnect_attempt)

            # 等待期间检测退出键
            wait_start = time.time()
            while time.time() - wait_start < RECONNECT_DELAY:
                key = cv2.waitKey(100) & 0xFF
                if key == ord('q'):
                    cv2.destroyAllWindows()
                    print("退出")
                    return
                show_reconnect_screen(info, reconnect_attempt)

            sock = connect_to_k230(ip, port)
            if sock:
                print("重连成功！")
                reconnect_attempt = 0
                frame_count = 0
                fps_time = time.time()
                fps_show = 0
                # 重连后保留录制状态
                continue
            else:
                continue  # 再次重试

        # ========== 已连接状态：接收帧并显示 ==========
        try:
            # 读取帧长度
            size_bytes = recv_exact(sock, 4)
            frame_size = struct.unpack(">I", size_bytes)[0]

            if frame_size == 0 or frame_size > 5 * 1024 * 1024:
                continue

            # 读取 JPEG
            jpeg = recv_exact(sock, frame_size)

            arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # FPS
            frame_count += 1
            now = time.time()
            if now - fps_time >= 1.0:
                fps_show = frame_count
                frame_count = 0
                fps_time = now

            h, w = frame.shape[:2]

            # HUD
            cv2.putText(frame, "FPS:%d | %dx%d" % (fps_show, w, h),
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            if recording:
                cv2.putText(frame, "REC", (w - 70, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            cv2.imshow("K230 Video Stream", frame)

            if writer:
                writer.write(frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                fn = "k230_%s.jpg" % time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(fn, frame)
                print("截图:", fn)
            elif key == ord('r'):
                if not recording:
                    fn = "k230_%s.avi" % time.strftime("%Y%m%d_%H%M%S")
                    writer = cv2.VideoWriter(fn, cv2.VideoWriter_fourcc(*'XVID'), 15, (w, h))
                    recording = True
                    print("录制:", fn)
                else:
                    writer.release()
                    writer = None
                    recording = False
                    print("停止录制")

        except (ConnectionError, socket.timeout, OSError) as e:
            print("连接断开: %s" % e)
            try:
                sock.close()
            except:
                pass
            sock = None
            show_reconnect_screen("Connection lost, reconnecting...")
            continue

    # 退出
    if sock:
        try:
            sock.close()
        except:
            pass
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    print("退出")


if __name__ == "__main__":
    ip = K230_IP
    port = K230_PORT
    args = sys.argv[1:]
    if len(args) >= 1:
        ip = args[0]
    if len(args) >= 2:
        port = int(args[1])
    main(ip, port)
