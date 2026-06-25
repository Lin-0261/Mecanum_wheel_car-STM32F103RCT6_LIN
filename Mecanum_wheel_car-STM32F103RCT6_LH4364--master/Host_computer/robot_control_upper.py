#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人/小车远程控制上位机
基于 PyQt5，深色科技风界面
功能：K230图传(实时显示)、Vx/Vy/w 速度指令(发送到K230)、方向/旋转摇杆、绝对速度控制、急停

控制模型:
  - 左摇杆(方向): 控制运动方向角度 → 解算 Vx, Vy
  - 右摇杆(旋转): 水平偏移控制 w (角速度)，水平指向时 |w| 最大
  - Q/E 键: 减小/增大绝对速度
  - 空格键: 一键停止（所有速度归零）
  - 速度指令通过 UDP 发送到 K230 (格式: "Vx,Vy,w\\n")

图传协议: TCP + JPEG, 4字节大端帧头长度 + JPEG数据
"""

import sys
import math
import socket
import struct
import time
import os

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSizePolicy, QFrame, QLineEdit
)
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QSize, QThread, pyqtSignal
from PyQt5.QtGui import (
    QPainter, QColor, QFont, QPen, QBrush, QLinearGradient,
    QRadialGradient, QFontDatabase, QMouseEvent, QKeyEvent,
    QPolygonF, QPixmap, QImage
)

# ---- 可选图像解码库 ----
try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from PIL import Image as PILImage
    import io as pil_io
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ======================== 全局配色常量 ========================
COLOR_BG_DARK      = "#1e1e2e"   # 主背景色
COLOR_BG_CARD      = "#252536"   # 卡片背景
COLOR_BG_WIDGET    = "#2a2a3c"   # 控件背景
COLOR_TEXT_PRIMARY = "#cdd6f4"   # 主文字色
COLOR_TEXT_SECOND  = "#a6adc8"   # 次文字色
COLOR_ACCENT_BLUE  = "#89b4fa"   # 强调蓝
COLOR_ACCENT_GREEN = "#a6e3a1"   # 强调绿(连接状态)
COLOR_ACCENT_RED   = "#f38ba8"   # 强调红
COLOR_ACCENT_ORANGE= "#fab387"   # 强调橙
COLOR_BORDER       = "#45475a"   # 边框色
COLOR_CHART_LINE   = "#89b4fa"   # 曲线颜色
COLOR_CHART_BG     = "#1a1a28"   # 图表背景
COLOR_JOYSTICK_BASE = QColor(40, 42, 60)
COLOR_JOYSTICK_KNOB = QColor(137, 180, 250)

FONT_FAMILY = "Microsoft YaHei, Segoe UI, sans-serif"

# 控制参数
DEFAULT_MAX_SPEED = 600      # 绝对速度最大值 (mm/s)
DEFAULT_MAX_W     = 5        # 角速度 w 最大值 (rad/s)
SPEED_STEP        = 50       # Q/E 每次调整的步长 (mm/s)
K230_CONTROL_PORT = 8889     # K230 控制指令 UDP 端口


# ======================== 自定义摇杆控件 ========================
class JoystickWidget(QWidget):
    """
    圆形摇杆控件：支持鼠标拖拽/点击 和 键盘 WASD 控制
    - 按下鼠标并拖动，内部小圆点跟随移动
    - 松开鼠标后，小圆点弹性复位到中心
    - 同时提供 keyboard_offset 属性，供外部设置键盘偏移量
    """

    def __init__(self, parent=None, label_text="摇杆"):
        super().__init__(parent)
        self._label_text = label_text

        # 摇杆状态
        self._mouse_offset = QPointF(0, 0)      # 鼠标拖拽偏移 (-1 ~ 1)
        self._keyboard_offset = QPointF(0, 0)    # 键盘偏移 (-1 ~ 1)
        self._mouse_active = False                # 鼠标是否按住

        # 动画过渡用
        self._current_display_offset = QPointF(0, 0)

        self.setMinimumSize(160, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

    # --- 公共属性 ---
    def set_keyboard_offset(self, dx, dy):
        """由外部（MainWindow）设置键盘偏移量，范围 -1..1"""
        self._keyboard_offset = QPointF(
            max(-1.0, min(1.0, dx)),
            max(-1.0, min(1.0, dy))
        )

    def effective_offset(self):
        """
        返回当前有效偏移量（鼠标优先，否则键盘）
        返回 QPointF，x/y 范围 -1..1
        """
        if self._mouse_active:
            return self._mouse_offset
        return self._keyboard_offset

    def reset(self):
        """复位摇杆到中心（急停时调用）"""
        self._mouse_offset = QPointF(0, 0)
        self._keyboard_offset = QPointF(0, 0)
        self._mouse_active = False
        self._current_display_offset = QPointF(0, 0)
        self.update()

    # --- 绘制 ---
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        side = min(w, h)
        cx = w / 2.0
        cy = h / 2.0
        radius = side / 2.0 - 10  # 留边距

        # ---- 底座渐变 ----
        base_gradient = QRadialGradient(cx, cy, radius * 1.1)
        base_gradient.setColorAt(0.0, QColor(60, 62, 80))
        base_gradient.setColorAt(0.7, QColor(35, 37, 55))
        base_gradient.setColorAt(1.0, QColor(25, 27, 42))
        painter.setPen(QPen(QColor(COLOR_BORDER), 2))
        painter.setBrush(QBrush(base_gradient))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # ---- 内圈装饰环 ----
        inner_radius = radius * 0.7
        painter.setPen(QPen(QColor(COLOR_BORDER), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), inner_radius, inner_radius)

        # ---- 十字参考线 ----
        painter.setPen(QPen(QColor(80, 82, 100), 1, Qt.DashLine))
        painter.drawLine(QPointF(cx - inner_radius, cy), QPointF(cx + inner_radius, cy))
        painter.drawLine(QPointF(cx, cy - inner_radius), QPointF(cx, cy + inner_radius))

        # ---- 计算目标偏移（平滑过渡） ----
        target = self.effective_offset()
        lerp_factor = 0.3
        self._current_display_offset = QPointF(
            self._current_display_offset.x() + (target.x() - self._current_display_offset.x()) * lerp_factor,
            self._current_display_offset.y() + (target.y() - self._current_display_offset.y()) * lerp_factor,
        )

        knob_x = cx + self._current_display_offset.x() * inner_radius
        knob_y = cy - self._current_display_offset.y() * inner_radius  # Y轴反转（上为正）

        # ---- 活动小圆点 ----
        knob_r = radius * 0.18
        knob_gradient = QRadialGradient(knob_x, knob_y, knob_r * 1.5)
        knob_gradient.setColorAt(0.0, QColor(180, 210, 255))
        knob_gradient.setColorAt(0.5, COLOR_JOYSTICK_KNOB)
        knob_gradient.setColorAt(1.0, QColor(70, 120, 200))
        painter.setPen(QPen(QColor(120, 160, 220), 1.5))
        painter.setBrush(QBrush(knob_gradient))
        painter.drawEllipse(QPointF(knob_x, knob_y), knob_r, knob_r)

        # ---- 底部标签 ----
        painter.setPen(QColor(COLOR_TEXT_PRIMARY))
        font = QFont(FONT_FAMILY.split(",")[0].strip(), 9)
        painter.setFont(font)
        text_rect = QRectF(0, h - 18, w, 16)
        painter.drawText(text_rect, Qt.AlignCenter, self._label_text)

        painter.end()

    # --- 鼠标事件 ---
    def _get_joystick_offset(self, pos):
        """根据鼠标位置计算摇杆偏移量 (-1..1)"""
        w = self.width()
        h = self.height()
        side = min(w, h)
        radius = side / 2.0 - 10
        inner_radius = radius * 0.7
        cx = w / 2.0
        cy = h / 2.0

        dx = (pos.x() - cx) / inner_radius
        dy = -(pos.y() - cy) / inner_radius  # Y反转
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 1.0:
            dx /= dist
            dy /= dist
        return QPointF(dx, dy)

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            self._mouse_active = True
            self._mouse_offset = self._get_joystick_offset(event.pos())
            self.update()

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._mouse_active:
            self._mouse_offset = self._get_joystick_offset(event.pos())
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            self._mouse_active = False
            self._mouse_offset = QPointF(0, 0)
            self.update()

    def sizeHint(self):
        return QSize(180, 180)


# ======================== 速度指令显示面板 ========================
class VelocityInfoWidget(QWidget):
    """
    速度指令显示面板：绘制 Vx/Vy 方向矢量图 + w 指示条 + 数值
    替代原来的4电机监控卡片
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._vx = 0.0
        self._vy = 0.0
        self._w = 0.0
        self._abs_speed = 0.0
        self._max_speed = DEFAULT_MAX_SPEED
        self._max_w = DEFAULT_MAX_W

        self.setMinimumSize(240, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_velocity(self, vx, vy, w, abs_speed):
        """更新速度值"""
        self._vx = vx
        self._vy = vy
        self._w = w
        self._abs_speed = abs_speed
        self.update()

    def set_limits(self, max_speed, max_w):
        self._max_speed = max_speed
        self._max_w = max_w

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        margin = 12

        # ---- 卡片背景 ----
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(COLOR_BG_CARD))
        painter.drawRoundedRect(QRectF(0, 0, w, h), 8, 8)

        # ---- 标题 ----
        painter.setPen(QColor(COLOR_TEXT_PRIMARY))
        title_font = QFont(FONT_FAMILY.split(",")[0].strip(), 11, QFont.Bold)
        painter.setFont(title_font)
        painter.drawText(QRectF(margin, 6, w - 2*margin, 24),
                         Qt.AlignLeft | Qt.AlignVCenter, "速度指令 Velocity Command")

        # ---- 布局划分 ----
        # 左侧: Vx/Vy 矢量图 (占 ~55% 宽度)
        # 右侧: 数值显示 + w 指示条 (占 ~45% 宽度)
        left_w = int(w * 0.52)
        right_x = left_w + 4
        right_w = w - right_x - margin

        top_y = 32
        bottom_h = h - top_y - 6

        # ====== 左侧: 方向矢量图 ======
        vec_cx = left_w // 2
        vec_cy = top_y + bottom_h // 2
        vec_r = min(left_w // 2 - 20, bottom_h // 2 - 10)

        # 背景圆
        painter.setPen(QPen(QColor(COLOR_BORDER), 1.5))
        painter.setBrush(QColor(COLOR_CHART_BG))
        painter.drawEllipse(QPointF(vec_cx, vec_cy), vec_r, vec_r)

        # 十字参考线
        painter.setPen(QPen(QColor(60, 62, 80), 1, Qt.DotLine))
        painter.drawLine(QPointF(vec_cx - vec_r, vec_cy), QPointF(vec_cx + vec_r, vec_cy))
        painter.drawLine(QPointF(vec_cx, vec_cy - vec_r), QPointF(vec_cx, vec_cy + vec_r))

        # 轴标签
        axis_font = QFont(FONT_FAMILY.split(",")[0].strip(), 8)
        painter.setFont(axis_font)
        painter.setPen(QColor(COLOR_TEXT_SECOND))
        painter.drawText(QRectF(vec_cx + vec_r - 16, vec_cy - 16, 18, 12), Qt.AlignCenter, "Vx")
        painter.drawText(QRectF(vec_cx - 10, vec_cy - vec_r - 4, 20, 12), Qt.AlignCenter, "Vy")

        # 速度矢量箭头
        if abs(self._vx) > 0.01 or abs(self._vy) > 0.01:
            scale = vec_r / self._max_speed if self._max_speed > 0 else 1.0
            arrow_dx = self._vx * scale
            arrow_dy = -self._vy * scale  # Y轴反转（屏幕坐标系）

            # 限制箭头长度
            arrow_len = math.sqrt(arrow_dx**2 + arrow_dy**2)
            if arrow_len > vec_r * 0.95:
                arrow_dx = arrow_dx / arrow_len * vec_r * 0.95
                arrow_dy = arrow_dy / arrow_len * vec_r * 0.95

            # 箭杆
            painter.setPen(QPen(QColor(COLOR_ACCENT_BLUE), 2.5))
            painter.drawLine(
                QPointF(vec_cx, vec_cy),
                QPointF(vec_cx + arrow_dx, vec_cy + arrow_dy)
            )

            # 箭头头部
            head_len = max(8, arrow_len * 0.25)
            angle = math.atan2(arrow_dy, arrow_dx)
            painter.setBrush(QColor(COLOR_ACCENT_BLUE))
            painter.setPen(Qt.NoPen)
            arrow_head = QPolygonF([
                QPointF(vec_cx + arrow_dx, vec_cy + arrow_dy),
                QPointF(vec_cx + arrow_dx - head_len * math.cos(angle - 0.5),
                        vec_cy + arrow_dy - head_len * math.sin(angle - 0.5)),
                QPointF(vec_cx + arrow_dx - head_len * math.cos(angle + 0.5),
                        vec_cy + arrow_dy - head_len * math.sin(angle + 0.5)),
            ])
            painter.drawPolygon(arrow_head)

        # 圆心点
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(COLOR_TEXT_SECOND))
        painter.drawEllipse(QPointF(vec_cx, vec_cy), 3, 3)

        # ====== 右侧: 数值 + w 指示条 ======
        val_x = right_x
        val_y = top_y + 6

        val_font = QFont(FONT_FAMILY.split(",")[0].strip(), 10)
        painter.setFont(val_font)

        row_h = 22

        # Vx
        painter.setPen(QColor(COLOR_TEXT_SECOND))
        painter.drawText(QRectF(val_x, val_y, 30, row_h), Qt.AlignLeft | Qt.AlignVCenter, "Vx:")
        painter.setPen(QColor(COLOR_ACCENT_BLUE))
        painter.drawText(QRectF(val_x + 28, val_y, right_w - 28, row_h),
                         Qt.AlignLeft | Qt.AlignVCenter, f"{self._vx:+.0f}")

        # Vy
        val_y += row_h
        painter.setPen(QColor(COLOR_TEXT_SECOND))
        painter.drawText(QRectF(val_x, val_y, 30, row_h), Qt.AlignLeft | Qt.AlignVCenter, "Vy:")
        painter.setPen(QColor(COLOR_ACCENT_BLUE))
        painter.drawText(QRectF(val_x + 28, val_y, right_w - 28, row_h),
                         Qt.AlignLeft | Qt.AlignVCenter, f"{self._vy:+.0f}")

        # 绝对速度
        val_y += row_h + 2
        painter.setPen(QColor(COLOR_TEXT_SECOND))
        painter.drawText(QRectF(val_x, val_y, 60, row_h), Qt.AlignLeft | Qt.AlignVCenter, "|V|:")
        painter.setPen(QColor(COLOR_ACCENT_ORANGE))
        painter.drawText(QRectF(val_x + 40, val_y, right_w - 40, row_h),
                         Qt.AlignLeft | Qt.AlignVCenter, f"{self._abs_speed:.0f}")

        # w 值
        val_y += row_h + 2
        painter.setPen(QColor(COLOR_TEXT_SECOND))
        painter.drawText(QRectF(val_x, val_y, 28, row_h), Qt.AlignLeft | Qt.AlignVCenter, "w:")
        painter.setPen(QColor(COLOR_ACCENT_GREEN))
        painter.drawText(QRectF(val_x + 26, val_y, right_w - 26, row_h),
                         Qt.AlignLeft | Qt.AlignVCenter, f"{self._w:+.0f}")

        # w 指示条
        val_y += row_h + 4
        bar_h = 12
        bar_w = right_w - 10
        bar_x = val_x

        # 背景条
        painter.setPen(QPen(QColor(COLOR_BORDER), 1))
        painter.setBrush(QColor(COLOR_CHART_BG))
        painter.drawRoundedRect(QRectF(bar_x, val_y, bar_w, bar_h), 3, 3)

        # w 指示
        if self._max_w > 0:
            w_frac = abs(self._w) / self._max_w
            mid_x = bar_x + bar_w / 2.0
            fill_w = (bar_w / 2.0) * w_frac

            painter.setPen(Qt.NoPen)
            if self._w >= 0:
                painter.setBrush(QColor(COLOR_ACCENT_GREEN))
                painter.drawRoundedRect(QRectF(mid_x, val_y, fill_w, bar_h), 3, 3)
            else:
                painter.setBrush(QColor(COLOR_ACCENT_RED))
                painter.drawRoundedRect(QRectF(mid_x - fill_w, val_y, fill_w, bar_h), 3, 3)

            # 中心线
            painter.setPen(QPen(QColor(COLOR_TEXT_PRIMARY), 1))
            painter.drawLine(QPointF(mid_x, val_y - 2), QPointF(mid_x, val_y + bar_h + 2))

        # 提示
        val_y += bar_h + 6
        hint_font = QFont(FONT_FAMILY.split(",")[0].strip(), 8)
        painter.setFont(hint_font)
        painter.setPen(QColor(COLOR_TEXT_SECOND))
        painter.drawText(QRectF(val_x, val_y, right_w, 14),
                         Qt.AlignLeft | Qt.AlignVCenter, "Q/E:速度±50  空格:急停")

        painter.end()

    def sizeHint(self):
        return QSize(380, 260)


# ======================== 图传: TCP视频流接收线程 ========================
class VideoReceiverThread(QThread):
    """
    TCP/JPEG 视频流接收线程
    通过 TCP 连接 K230，接收 4字节帧头(大端) + JPEG数据，解码后发射信号给GUI
    """
    frame_received = pyqtSignal(QPixmap)              # 新帧
    connection_changed = pyqtSignal(bool, str)          # 连接状态改变
    status_text = pyqtSignal(str)                       # 状态文字更新
    error_occurred = pyqtSignal(str)                    # 错误信息

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ip = "192.168.137.169"
        self._port = 8888
        self._running = False
        self._sock = None
        self._fps = 0

    def set_target(self, ip, port):
        self._ip = ip
        self._port = port

    def start_receive(self):
        if not self.isRunning():
            self._running = True
            self.start()
        else:
            self._running = True

    def stop_receive(self):
        self._running = False
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except:
                pass
            try:
                self._sock.close()
            except:
                pass
            self._sock = None
        self.wait(3000)
        self.connection_changed.emit(False, "已断开")
        self.status_text.emit("FPS: --")

    def get_fps(self):
        return self._fps

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            if not self._running:
                raise ConnectionError("线程停止")
            try:
                chunk = self._sock.recv(n - len(buf))
            except socket.timeout:
                if not self._running:
                    raise ConnectionError("线程停止")
                continue
            if not chunk:
                raise ConnectionError("连接断开")
            buf += chunk
        return buf

    def _decode_jpeg(self, jpeg_bytes):
        if HAS_CV2:
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return None
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
            return QPixmap.fromImage(qimg.copy())
        elif HAS_PIL:
            try:
                img = PILImage.open(pil_io.BytesIO(jpeg_bytes))
                img = img.convert("RGB")
                data = img.tobytes("raw", "RGB")
                qimg = QImage(data, img.width, img.height, QImage.Format_RGB888)
                return QPixmap.fromImage(qimg.copy())
            except Exception:
                return None
        return None

    def run(self):
        self._running = True

        while self._running:
            self.connection_changed.emit(False, f"正在连接 {self._ip}:{self._port} ...")
            self.status_text.emit("FPS: --")

            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.settimeout(5)
                self._sock.connect((self._ip, self._port))
                self._sock.settimeout(2.0)
                self.connection_changed.emit(True, f"已连接 {self._ip}:{self._port}")

                frame_count = 0
                fps_time = time.time()
                self._fps = 0

                while self._running:
                    try:
                        size_bytes = self._recv_exact(4)
                        frame_size = struct.unpack(">I", size_bytes)[0]
                        if frame_size == 0 or frame_size > 5 * 1024 * 1024:
                            continue
                        jpeg = self._recv_exact(frame_size)
                        pixmap = self._decode_jpeg(jpeg)
                        if pixmap:
                            self.frame_received.emit(pixmap)
                            frame_count += 1
                            now = time.time()
                            if now - fps_time >= 1.0:
                                self._fps = frame_count
                                self.status_text.emit(
                                    f"FPS: {frame_count} | {pixmap.width()}x{pixmap.height()}"
                                )
                                frame_count = 0
                                fps_time = now
                    except (socket.timeout, BlockingIOError):
                        continue
                    except (ConnectionError, OSError) as e:
                        if self._running:
                            self.connection_changed.emit(False, f"连接断开: {str(e)[:40]}")
                        break

            except (socket.timeout, ConnectionRefusedError, OSError) as e:
                if self._running:
                    self.connection_changed.emit(False, f"连接失败: {str(e)[:40]}")
            except Exception as e:
                if self._running:
                    self.connection_changed.emit(False, f"错误: {str(e)[:40]}")

            if self._sock:
                try:
                    self._sock.close()
                except:
                    pass
                self._sock = None

            if self._running:
                self.status_text.emit("FPS: -- | 等待重连...")
                for _ in range(20):
                    if not self._running:
                        break
                    time.sleep(0.1)

        self.connection_changed.emit(False, "已断开")
        self.status_text.emit("FPS: --")


# ======================== 图传: 视频显示控件 ========================
class VideoDisplayWidget(QWidget):
    """
    视频显示区域：绘制 QPixmap 帧，居中缩放保持宽高比
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._frame = None
        self._placeholder = "等待视频流..."
        self._overlay_text = ""
        self._overlay_color = QColor(COLOR_ACCENT_GREEN)

        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAutoFillBackground(True)

        pal = self.palette()
        pal.setColor(self.backgroundRole(), QColor("#000000"))
        self.setPalette(pal)

    def set_frame(self, pixmap):
        self._frame = pixmap
        self.update()

    def set_overlay(self, text, color=None):
        self._overlay_text = text
        if color:
            self._overlay_color = QColor(color)

    def set_placeholder(self, text):
        self._placeholder = text
        self.update()

    def clear_frame(self):
        self._frame = None
        self._overlay_text = ""
        self.update()

    def get_frame(self):
        return self._frame

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

        w = self.width()
        h = self.height()

        if self._frame and not self._frame.isNull():
            fw = self._frame.width()
            fh = self._frame.height()
            scale = min(w / fw, h / fh)
            scaled_w = int(fw * scale)
            scaled_h = int(fh * scale)
            x = (w - scaled_w) // 2
            y = (h - scaled_h) // 2
            painter.drawPixmap(QRectF(x, y, scaled_w, scaled_h), self._frame, QRectF(0, 0, fw, fh))
        else:
            painter.fillRect(QRectF(0, 0, w, h), QColor("#000000"))
            painter.setPen(QColor(COLOR_TEXT_SECOND))
            font = QFont(FONT_FAMILY.split(",")[0].strip(), 16)
            painter.setFont(font)
            painter.drawText(QRectF(0, 0, w, h), Qt.AlignCenter, self._placeholder)
            dots = int(time.time() * 2) % 4
            painter.setPen(QColor(COLOR_ACCENT_BLUE))
            font_s = QFont(FONT_FAMILY.split(",")[0].strip(), 12)
            painter.setFont(font_s)
            painter.drawText(QRectF(0, h // 2 + 24, w, 20), Qt.AlignCenter,
                             f"请先连接 K230 设备{'.' * dots}")

        if self._overlay_text:
            painter.fillRect(QRectF(8, 8, 260, 24), QColor(0, 0, 0, 120))
            painter.setPen(self._overlay_color)
            overlay_font = QFont(FONT_FAMILY.split(",")[0].strip(), 10, QFont.Bold)
            painter.setFont(overlay_font)
            painter.drawText(QRectF(14, 8, 254, 24), Qt.AlignVCenter, self._overlay_text)

        painter.end()


# ======================== 主窗口 ========================
class MainWindow(QMainWindow):
    """机器人远程控制上位机主窗口"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("机器人远程控制上位机 — K230图传 & 速度指令")
        self.resize(1200, 800)

        # ---- 键盘状态 ----
        self._key_w = False
        self._key_a = False
        self._key_s = False
        self._key_d = False
        self._key_q = False
        self._key_e = False

        # ---- 速度状态 ----
        self._absolute_speed = 0.0       # 绝对速度 (0 ~ max_speed)
        self._max_speed = DEFAULT_MAX_SPEED
        self._max_w = DEFAULT_MAX_W
        self._speed_step = SPEED_STEP
        self._vx = 0.0
        self._vy = 0.0
        self._w = 0.0

        # ---- UDP 控制发送 ----
        self._udp_sock = None
        self._control_ip = "192.168.137.169"
        self._control_port = K230_CONTROL_PORT
        self._udp_enabled = False

        # ---- 图传线程 ----
        self._video_thread = VideoReceiverThread(self)
        self._video_thread.frame_received.connect(self._on_frame_received)
        self._video_thread.connection_changed.connect(self._on_connection_changed)
        self._video_thread.status_text.connect(self._on_video_status)

        # ---- 截图 ----
        self._screenshot_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "screenshots")

        # ---- 构建UI ----
        self._setup_ui()
        self._apply_theme()

        # ---- 初始化 UDP ----
        self._init_udp()

        # ---- 定时器：20ms 刷新 (降低控制延迟) ----
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(20)

    # ============ UDP 初始化 ============
    def _init_udp(self):
        try:
            self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp_sock.settimeout(0.5)
            self._udp_enabled = True
        except Exception as e:
            print(f"UDP 初始化失败: {e}")
            self._udp_enabled = False

    # ============ UI搭建 ============
    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(8)

        # ---- 上半部分：图传（左）+ 速度指令显示（右） ----
        top_layout = QHBoxLayout()
        top_layout.setSpacing(8)

        # 图传画面区（左，约占 45% 宽度）
        self.video_panel = self._create_video_panel()
        top_layout.addWidget(self.video_panel, 45)

        # 速度指令显示区（右，约占 55% 宽度）
        self.velocity_info = VelocityInfoWidget()
        top_layout.addWidget(self.velocity_info, 55)

        root_layout.addLayout(top_layout, 55)

        # ---- 下半部分：控制区 ----
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(12)

        # 左下：方向控制摇杆
        self.joystick_dir = JoystickWidget(label_text="方向控制 (WASD)")
        bottom_layout.addWidget(self.joystick_dir, 3)

        # 中下：绝对速度面板
        self._create_speed_panel_ui()
        bottom_layout.addWidget(self._speed_panel_widget, 2)

        # 右下：旋转摇杆(w)
        self.joystick_turn = JoystickWidget(label_text="旋转控制 w (AD/←→)")
        bottom_layout.addWidget(self.joystick_turn, 3)

        # 急停按钮
        self.btn_stop = QPushButton("STOP\n空格")
        self.btn_stop.setCursor(Qt.PointingHandCursor)
        self.btn_stop.setMinimumSize(80, 80)
        self.btn_stop.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.btn_stop.clicked.connect(self._on_stop)
        bottom_layout.addWidget(self.btn_stop, 1)

        root_layout.addLayout(bottom_layout, 45)

    def _create_speed_panel_ui(self):
        """创建绝对速度显示面板"""
        panel = QWidget()
        panel.setObjectName("speedPanelWidget")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(4)

        # 标题
        title = QLabel("绝对速度")
        title.setObjectName("speedLabel")
        title.setAlignment(Qt.AlignCenter)

        # 速度值
        self._speed_value_label = QLabel("0")
        self._speed_value_label.setObjectName("speedValue")
        self._speed_value_label.setAlignment(Qt.AlignCenter)

        # 控制提示
        hint = QLabel("Q:-50  E:+50")
        hint.setObjectName("speedHint")
        hint.setAlignment(Qt.AlignCenter)

        # 上下箭头按钮（保留原速度面板功能）
        btn_up = QPushButton("▲ +50")
        btn_up.setCursor(Qt.PointingHandCursor)
        btn_up.clicked.connect(self._on_speed_increase)

        btn_down = QPushButton("▼ -50")
        btn_down.setCursor(Qt.PointingHandCursor)
        btn_down.clicked.connect(self._on_speed_decrease)

        layout.addWidget(title)
        layout.addStretch(1)
        layout.addWidget(btn_up)
        layout.addWidget(self._speed_value_label)
        layout.addWidget(btn_down)
        layout.addStretch(1)
        layout.addWidget(hint)

        self._speed_panel_widget = panel

    def _create_video_panel(self):
        """创建图传画面区域"""
        panel = QFrame()
        panel.setObjectName("videoPanel")
        panel.setMinimumSize(340, 360)
        panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # ---- 标题栏 + 连接控制 ----
        title_bar = QFrame()
        title_bar.setObjectName("videoTitleBar")
        title_bar.setFixedHeight(36)
        title_layout = QHBoxLayout(title_bar)
        title_layout.setContentsMargins(10, 4, 8, 4)
        title_layout.setSpacing(8)

        title_label = QLabel("K230图传画面")
        title_label.setObjectName("videoTitle")
        title_label.setFixedHeight(24)

        self._led_label = QLabel("●")
        self._led_label.setObjectName("videoLed")
        self._led_label.setFixedWidth(16)
        self._led_label.setAlignment(Qt.AlignCenter)

        title_layout.addWidget(title_label)
        title_layout.addWidget(self._led_label)
        title_layout.addStretch()

        layout.addWidget(title_bar)

        # ---- 连接配置行 ----
        config_bar = QFrame()
        config_bar.setObjectName("videoConfigBar")
        config_bar.setFixedHeight(36)
        config_layout = QHBoxLayout(config_bar)
        config_layout.setContentsMargins(10, 3, 8, 3)
        config_layout.setSpacing(6)

        config_layout.addWidget(QLabel("IP:"))
        self.ip_input = QLineEdit("192.168.137.169")
        self.ip_input.setObjectName("videoIpInput")
        self.ip_input.setFixedWidth(120)
        self.ip_input.setFixedHeight(24)

        config_layout.addWidget(QLabel("视频:"))
        self.port_input = QLineEdit("8888")
        self.port_input.setObjectName("videoPortInput")
        self.port_input.setFixedWidth(48)
        self.port_input.setFixedHeight(24)

        config_layout.addWidget(QLabel("控制:"))
        self.control_port_input = QLineEdit(str(K230_CONTROL_PORT))
        self.control_port_input.setObjectName("videoPortInput")
        self.control_port_input.setFixedWidth(48)
        self.control_port_input.setFixedHeight(24)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setObjectName("videoBtnConnect")
        self.btn_connect.setCursor(Qt.PointingHandCursor)
        self.btn_connect.setFixedSize(48, 26)
        self.btn_connect.clicked.connect(self._on_connect)

        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.setObjectName("videoBtnDisconnect")
        self.btn_disconnect.setCursor(Qt.PointingHandCursor)
        self.btn_disconnect.setFixedSize(48, 26)
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self._on_disconnect)

        config_layout.addWidget(self.ip_input)
        config_layout.addWidget(self.port_input)
        config_layout.addWidget(self.control_port_input)
        config_layout.addWidget(self.btn_connect)
        config_layout.addWidget(self.btn_disconnect)
        config_layout.addStretch()

        layout.addWidget(config_bar)

        # ---- 视频显示区域 ----
        self.video_display = VideoDisplayWidget()
        self.video_display.setObjectName("videoDisplay")
        layout.addWidget(self.video_display, 1)

        # ---- 底部状态栏 ----
        status_bar = QFrame()
        status_bar.setObjectName("videoStatusBar")
        status_bar.setFixedHeight(28)
        status_layout = QHBoxLayout(status_bar)
        status_layout.setContentsMargins(10, 2, 8, 2)
        status_layout.setSpacing(8)

        self._status_label = QLabel("FPS: --")
        self._status_label.setObjectName("videoStatusText")

        self.btn_screenshot = QPushButton("截图")
        self.btn_screenshot.setObjectName("videoBtnScreenshot")
        self.btn_screenshot.setCursor(Qt.PointingHandCursor)
        self.btn_screenshot.setFixedSize(48, 22)
        self.btn_screenshot.setEnabled(False)
        self.btn_screenshot.clicked.connect(self._on_screenshot)

        # UDP 状态指示
        self._udp_status_label = QLabel("")
        self._udp_status_label.setObjectName("udpStatusText")

        status_layout.addWidget(self._status_label)
        status_layout.addWidget(self._udp_status_label)
        status_layout.addStretch()
        status_layout.addWidget(self.btn_screenshot)

        layout.addWidget(status_bar)

        return panel

    # ============ 图传事件处理 ============
    def _on_connect(self):
        ip = self.ip_input.text().strip()
        port_text = self.port_input.text().strip()
        try:
            port = int(port_text)
        except ValueError:
            self._status_label.setText("错误: 端口号无效")
            return

        # 更新控制 IP
        self._control_ip = ip
        try:
            cp = int(self.control_port_input.text().strip())
            self._control_port = cp
        except ValueError:
            pass

        self._video_thread.set_target(ip, port)
        self._video_thread.start_receive()
        self._status_label.setText(f"正在连接 {ip}:{port} ...")
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self._led_label.setStyleSheet("color: #f9e2af; font-size: 14px;")

    def _on_disconnect(self):
        self._video_thread.stop_receive()
        self.video_display.clear_frame()
        self.video_display.set_placeholder("已断开连接")
        self._status_label.setText("FPS: --")
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.btn_screenshot.setEnabled(False)
        self._led_label.setStyleSheet("color: #f38ba8; font-size: 14px;")

    def _on_frame_received(self, pixmap):
        self.video_display.set_frame(pixmap)
        if not self.btn_screenshot.isEnabled():
            self.btn_screenshot.setEnabled(True)

    def _on_connection_changed(self, connected, status_text):
        if connected:
            self._led_label.setStyleSheet("color: #a6e3a1; font-size: 14px;")
        else:
            self._led_label.setStyleSheet("color: #f38ba8; font-size: 14px;")
        self._status_label.setText(status_text)

    def _on_video_status(self, text):
        self._status_label.setText(text)

    def _on_screenshot(self):
        frame = self.video_display.get_frame()
        if frame is None or frame.isNull():
            return
        os.makedirs(self._screenshot_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"k230_{timestamp}.jpg"
        filepath = os.path.join(self._screenshot_dir, filename)
        if frame.save(filepath, "JPEG", 90):
            self._status_label.setText(f"截图已保存: {filename}")
            self.video_display.set_overlay(f"📷 {filename}", COLOR_ACCENT_GREEN)
            QTimer.singleShot(2000, lambda: self.video_display.set_overlay(""))
        else:
            self._status_label.setText("截图保存失败")

    # ============ 速度控制 ============
    def _on_speed_increase(self):
        self._absolute_speed = min(self._max_speed, self._absolute_speed + SPEED_STEP)
        self._update_speed_display()

    def _on_speed_decrease(self):
        self._absolute_speed = max(0, self._absolute_speed - SPEED_STEP)
        self._update_speed_display()

    def _update_speed_display(self):
        self._speed_value_label.setText(str(int(self._absolute_speed)))

    # ============ UDP 发送速度指令 ============
    def _send_velocity_udp(self, vx, vy, w):
        """通过 UDP 发送 Vx, Vy, w 到 K230"""
        if not self._udp_enabled or not self._udp_sock:
            return
        try:
            # 格式: "Vx,Vy,w\n" (ASCII)
            msg = f"{vx:.0f},{vy:.0f},{w:.0f}\n"
            self._udp_sock.sendto(msg.encode(), (self._control_ip, self._control_port))
            self._udp_status_label.setText("UDP ✓")
            self._udp_status_label.setStyleSheet("color: #a6e3a1; font-size: 9px;")
        except Exception as e:
            self._udp_status_label.setText(f"UDP ✗")
            self._udp_status_label.setStyleSheet("color: #f38ba8; font-size: 9px;")

    # ============ 键盘事件 ============
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        if key == Qt.Key_W:
            self._key_w = True
        elif key == Qt.Key_A:
            self._key_a = True
        elif key == Qt.Key_S:
            self._key_s = True
        elif key == Qt.Key_D:
            self._key_d = True
        elif key == Qt.Key_Q:
            self._key_q = True
            self._absolute_speed = max(0, self._absolute_speed - self._speed_step)
            self._update_speed_display()
        elif key == Qt.Key_E:
            self._key_e = True
            self._absolute_speed = min(self._max_speed, self._absolute_speed + self._speed_step)
            self._update_speed_display()
        elif key == Qt.Key_Space:
            self._on_stop()
        self._sync_keyboard_to_joystick()

    def keyReleaseEvent(self, event: QKeyEvent):
        key = event.key()
        if key == Qt.Key_W:
            self._key_w = False
        elif key == Qt.Key_A:
            self._key_a = False
        elif key == Qt.Key_S:
            self._key_s = False
        elif key == Qt.Key_D:
            self._key_d = False
        elif key == Qt.Key_Q:
            self._key_q = False
        elif key == Qt.Key_E:
            self._key_e = False
        self._sync_keyboard_to_joystick()

    def _sync_keyboard_to_joystick(self):
        """将 WASD 状态转换为方向摇杆的键盘偏移"""
        dx = 0.0
        dy = 0.0
        if self._key_a:
            dx -= 1.0
        if self._key_d:
            dx += 1.0
        if self._key_w:
            dy += 1.0
        if self._key_s:
            dy -= 1.0
        if dx != 0 or dy != 0:
            mag = math.sqrt(dx * dx + dy * dy)
            dx /= mag
            dy /= mag
        self.joystick_dir.set_keyboard_offset(dx, dy)

    # ============ 定时器刷新 ============
    def _on_tick(self):
        """每50ms计算 Vx/Vy/w 并发送到 K230"""
        # 获取左摇杆（方向）偏移
        dir_offset = self.joystick_dir.effective_offset()

        # 获取右摇杆（旋转）偏移 → w
        # 使用二次曲线映射: w = x * |x| * max_w
        # 小幅度偏转时角速度极小，避免转向过敏感
        turn_offset = self.joystick_turn.effective_offset()
        x = turn_offset.x()
        self._w = x * abs(x) * self._max_w  # 二次曲线，保持符号

        # 从方向摇杆解算 Vx, Vy
        # 方向摇杆的 (dx, dy) 给出方向矢量，乘以绝对速度
        dir_x = dir_offset.x()
        dir_y = dir_offset.y()
        dir_mag = math.sqrt(dir_x**2 + dir_y**2)

        if dir_mag > 0.01:
            # 归一化方向矢量
            dir_x_norm = dir_x / dir_mag
            dir_y_norm = dir_y / dir_mag
        else:
            dir_x_norm = 0.0
            dir_y_norm = 0.0

        self._vx = self._absolute_speed * dir_x_norm
        self._vy = self._absolute_speed * dir_y_norm

        # 更新速度指令显示
        self.velocity_info.update_velocity(self._vx, self._vy, self._w, self._absolute_speed)

        # 发送 UDP 指令到 K230
        self._send_velocity_udp(self._vx, self._vy, self._w)

        # 刷新摇杆
        self.joystick_dir.update()
        self.joystick_turn.update()

    # ============ 速度显示更新 ============
    def _update_speed_display(self):
        self._speed_value_label.setText(str(int(self._absolute_speed)))

    # ============ 急停 ============
    def _on_stop(self):
        """急停：所有速度清零，摇杆复位"""
        self._absolute_speed = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._w = 0.0
        self._update_speed_display()

        self.joystick_dir.reset()
        self.joystick_turn.reset()

        # 释放所有键盘状态
        self._key_w = False
        self._key_a = False
        self._key_s = False
        self._key_d = False
        self._key_q = False
        self._key_e = False

        # 更新显示
        self.velocity_info.update_velocity(0, 0, 0, 0)

        # 发送停止指令
        self._send_velocity_udp(0, 0, 0)

    # ============ 全局主题 QSS ============
    def _apply_theme(self):
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {COLOR_BG_DARK};
            }}
            QWidget {{
                font-family: "{FONT_FAMILY}";
                color: {COLOR_TEXT_PRIMARY};
            }}
            /* 图传面板 */
            #videoPanel {{
                background-color: {COLOR_BG_CARD};
                border: 1px solid {COLOR_BORDER};
                border-radius: 8px;
            }}
            #videoTitleBar {{
                background-color: {COLOR_BG_WIDGET};
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
            }}
            #videoTitle {{
                color: {COLOR_TEXT_PRIMARY};
                font-size: 13px;
                font-weight: bold;
                background: transparent;
            }}
            #videoLed {{
                background: transparent;
                font-size: 14px;
            }}
            #videoConfigBar {{
                background-color: {COLOR_BG_WIDGET};
                border-bottom: 1px solid {COLOR_BORDER};
            }}
            #videoIpInput, #videoPortInput {{
                background-color: {COLOR_CHART_BG};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                padding: 2px 6px;
                font-size: 11px;
                selection-background-color: {COLOR_ACCENT_BLUE};
            }}
            #videoIpInput:focus, #videoPortInput:focus {{
                border-color: {COLOR_ACCENT_BLUE};
            }}
            #videoBtnConnect, #videoBtnDisconnect {{
                background-color: {COLOR_BG_WIDGET};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
                padding: 2px 8px;
            }}
            #videoBtnConnect:hover {{
                background-color: #3a5a3a;
                border-color: {COLOR_ACCENT_GREEN};
                color: {COLOR_ACCENT_GREEN};
            }}
            #videoBtnConnect:disabled {{
                color: #555;
                border-color: #444;
            }}
            #videoBtnDisconnect:hover {{
                background-color: #5a3a3a;
                border-color: {COLOR_ACCENT_RED};
                color: {COLOR_ACCENT_RED};
            }}
            #videoBtnDisconnect:disabled {{
                color: #555;
                border-color: #444;
            }}
            #videoStatusBar {{
                background-color: {COLOR_BG_WIDGET};
                border-bottom-left-radius: 8px;
                border-bottom-right-radius: 8px;
            }}
            #videoStatusText {{
                color: {COLOR_TEXT_SECOND};
                font-size: 10px;
                background: transparent;
            }}
            #udpStatusText {{
                font-size: 9px;
                background: transparent;
            }}
            #videoBtnScreenshot {{
                background-color: {COLOR_BG_WIDGET};
                color: {COLOR_TEXT_SECOND};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                font-size: 10px;
                padding: 1px 8px;
            }}
            #videoBtnScreenshot:hover {{
                background-color: #3a4a5a;
                border-color: {COLOR_ACCENT_BLUE};
                color: {COLOR_ACCENT_BLUE};
            }}
            #videoBtnScreenshot:disabled {{
                color: #555;
            }}
            /* 速度指令面板 */
            VelocityInfoWidget {{
                background-color: {COLOR_BG_CARD};
                border: 1px solid {COLOR_BORDER};
                border-radius: 8px;
            }}
            /* 速度面板 */
            #speedPanelWidget {{
                background-color: {COLOR_BG_CARD};
                border: 1px solid {COLOR_BORDER};
                border-radius: 8px;
            }}
            #speedLabel {{
                font-size: 13px;
                font-weight: bold;
                color: {COLOR_TEXT_PRIMARY};
            }}
            #speedValue {{
                font-size: 36px;
                font-weight: bold;
                color: {COLOR_ACCENT_BLUE};
            }}
            #speedHint {{
                font-size: 10px;
                color: {COLOR_TEXT_SECOND};
            }}
            QPushButton {{
                background-color: {COLOR_BG_WIDGET};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER};
                border-radius: 6px;
                padding: 6px 10px;
                font-size: 11px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #3a3a52;
                border-color: {COLOR_ACCENT_BLUE};
            }}
            QPushButton:pressed {{
                background-color: #2a2a3a;
            }}
            /* 急停按钮 */
            #btnStop {{
                background-color: #d64550;
                color: #ffffff;
                border: 2px solid #f38ba8;
                border-radius: 40px;
                font-size: 16px;
                font-weight: bold;
            }}
            #btnStop:hover {{
                background-color: #e05560;
                border-color: #ffa0b5;
            }}
            #btnStop:pressed {{
                background-color: #b0303c;
                border-color: #d06070;
            }}
        """)

        self.btn_stop.setObjectName("btnStop")

    # ============ 窗口关闭 ============
    def closeEvent(self, event):
        if self._video_thread.isRunning():
            self._video_thread.stop_receive()
        if self._udp_sock:
            try:
                self._udp_sock.close()
            except:
                pass
        event.accept()


# ======================== 全局主题 QSS ========================
# 注: QSS 在 MainWindow._apply_theme 中动态设置


# ======================== 程序入口 ========================
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("RobotControlUpper")

    font = QFont(FONT_FAMILY.split(",")[0].strip(), 10)
    app.setFont(font)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
