# PC 端发送文字到 K230
# 使用方式: python send_text_to_k230.py
# 或: python send_text_to_k230.py "Hello World"
# 或: python send_text_to_k230.py 192.168.137.36 "Hello World"

import socket
import sys

# K230 的 IP 地址和端口（根据实际修改）
K230_IP = "192.168.137.36"
K230_PORT = 8080

def send_text(ip, port, text):
    """发送文字到 K230"""
    print("连接到 %s:%d ..." % (ip, port))
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ip, port))
        print("已连接，发送: %s" % text)
        sock.send(text.encode("utf-8"))
        sock.close()
        print("发送成功！")
    except Exception as e:
        print("发送失败: %s" % e)

if __name__ == "__main__":
    ip = K230_IP
    text = None

    # 解析命令行参数
    args = sys.argv[1:]
    if len(args) >= 2:
        # 检查第一个参数是否为 IP 地址
        parts = args[0].split(".")
        if len(parts) == 4 and all(p.isdigit() for p in parts):
            ip = args[0]
            text = " ".join(args[1:])
        else:
            text = " ".join(args)
    elif len(args) == 1:
        text = args[0]

    if text is None:
        # 交互模式
        print("=" * 40)
        print("  K230 LCD 文字发送工具")
        print("  目标: %s:%d" % (ip, K230_PORT))
        print("  输入文字后回车发送")
        print("  输入 'quit' 或 'exit' 退出")
        print("=" * 40)
        while True:
            try:
                text = input("\n请输入要发送的文字: ")
                if text.lower() in ("quit", "exit", "q"):
                    print("退出。")
                    break
                if text.strip():
                    send_text(ip, K230_PORT, text)
            except KeyboardInterrupt:
                print("\n退出。")
                break
    else:
        send_text(ip, K230_PORT, text)
