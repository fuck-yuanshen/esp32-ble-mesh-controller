

from __future__ import annotations

import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

from PyQt6.QtCore import QObject, Qt, QThread, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QColor, QTextCursor
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)


BAUD_RATE = 115200
PACKET_HEAD = 0xAA
FEEDBACK_HEAD = 0xBB
PACKET_TAIL = 0x55


NODES = {
    "Node 1 (0x0002)": 0x0002,
    "Node 2 (0x0003)": 0x0003,
    "Node 3 (0x0004)": 0x0004,
    "全体群组 (0xC000)": 0xC000,
}

TARGET_LABELS = {
    0x0002: "Node 1",
    0x0003: "Node 2",
    0x0004: "Node 3",
    0xC000: "全体群组",
}

DEVICE_TYPES = {
    "LED 灯": 0x01,
    "蜂鸣器": 0x02,
}

ACTION_MODES = {
    "关闭": 0x00,
    "常亮/响": 0x01,
    "5Hz闪烁": 0x02,
}


class SerialWorker(QObject):
    """串口后台线程对象，所有阻塞 I/O 都放在这里执行。"""

    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    error = pyqtSignal(str)
    log = pyqtSignal(str)
    feedback = pyqtSignal(int, int)

    def __init__(self) -> None:
        super().__init__()
        self._serial: serial.Serial | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._serial_lock = threading.Lock()

    @pyqtSlot(str)
    def open_port(self, port_name: str) -> None:
        try:
            self.close_port()
            serial_obj = serial.Serial(
                baudrate=BAUD_RATE,
                timeout=0.1,
                write_timeout=0.5,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            serial_obj.port = port_name
            serial_obj.dtr = False
            serial_obj.rts = False
            serial_obj.open()
            serial_obj.setDTR(False)
            serial_obj.setRTS(False)
            time.sleep(0.2)
            serial_obj.reset_input_buffer()
            serial_obj.reset_output_buffer()
            self._stop_event.clear()
            reader_thread = threading.Thread(
                target=self._read_loop_text,
                args=(serial_obj,),
                daemon=True,
                name=f"SerialReader-{port_name}",
            )
            with self._serial_lock:
                self._serial = serial_obj
                self._reader_thread = reader_thread
            reader_thread.start()
            self.connected.emit(port_name)
            self.log.emit(f"串口已连接：{port_name} @ {BAUD_RATE}")
        except serial.SerialException as exc:
            self.error.emit(f"打开串口失败：{exc}")

    @pyqtSlot()
    def close_port(self) -> None:
        current_thread = threading.current_thread()
        with self._serial_lock:
            serial_obj = self._serial
            reader_thread = self._reader_thread
            self._serial = None
            self._reader_thread = None
        self._stop_event.set()

        if serial_obj is not None:
            try:
                if serial_obj.is_open:
                    serial_obj.close()
            finally:
                pass

        if (
            reader_thread is not None
            and reader_thread.is_alive()
            and reader_thread is not current_thread
        ):
            reader_thread.join(timeout=1.5)

        if serial_obj is not None:
            self.disconnected.emit()

    @pyqtSlot(bytes)
    def write_packet(self, packet: bytes) -> None:
        with self._serial_lock:
            serial_obj = self._serial

        if serial_obj is None or not serial_obj.is_open:
            self.error.emit("串口未连接，无法发送指令")
            return

        try:
            serial_obj.write(packet)
            serial_obj.flush()
            self.log.emit(f"TX: {packet.hex(' ').upper()}")
        except serial.SerialException as exc:
            self.error.emit(f"串口发送失败：{exc}")

    def _read_loop_text(self, serial_obj: serial.Serial) -> None:
        line_buf = bytearray()

        while not self._stop_event.is_set() and serial_obj.is_open:
            try:
                data = serial_obj.read(256)
            except serial.SerialException as exc:
                self.error.emit(f"串口读取失败：{exc}")
                break

            if not data:
                QThread.msleep(10)
                continue

            for byte in data:
                if byte in (10, 13):
                    if not line_buf:
                        continue
                    line = line_buf.decode("utf-8", errors="replace").strip()
                    line_buf.clear()
                    if not line:
                        continue
                    if line.startswith("FB,"):
                        parts = line.split(",")
                        if len(parts) == 3:
                            try:
                                node_addr = int(parts[1], 16)
                                state = int(parts[2], 16)
                            except ValueError:
                                self.log.emit(f"RX 无效反馈行：{line}")
                                continue
                            self.log.emit(f"RX: {line} -> addr=0x{node_addr:04X}, state=0x{state:02X}")
                            self.feedback.emit(node_addr, state)
                        else:
                            self.log.emit(f"RX 无效反馈行：{line}")
                    else:
                        self.log.emit(line)
                    continue

                line_buf.append(byte)

        if not self._stop_event.is_set():
            self.error.emit("串口读取线程已退出")
        self.close_port()


class MeshControlWindow(QMainWindow):
    """上位机主界面，负责控件组织、协议封装、状态展示和日志保存。"""

    open_requested = pyqtSignal(str)
    close_requested = pyqtSignal()
    send_requested = pyqtSignal(bytes)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("BLE Mesh")
        self.resize(860, 620)

        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)

        self.open_requested.connect(self.worker.open_port)
        self.close_requested.connect(self.worker.close_port)
        self.send_requested.connect(self.worker.write_packet)
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)
        self.worker.error.connect(self._show_error)
        self.worker.log.connect(self._append_log)
        self.worker.feedback.connect(self._update_node_state)
        self.thread.start()

        self.port_combo = QComboBox()
        self.node_combo = QComboBox()
        self.device_combo = QComboBox()
        self.action_combo = QComboBox()
        self.log_view = QPlainTextEdit()
        self.connect_button = QPushButton("连接")
        self.disconnect_button = QPushButton("断开")
        self.refresh_button = QPushButton("刷新端口")
        self.send_button = QPushButton("发送指令")
        self.clear_button = QPushButton("清除日志")
        self.save_button = QPushButton("保存数据")
        self.status_labels: dict[int, QLabel] = {}

        self._build_ui()
        self._refresh_ports()
        self._set_connected_state(False)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.close_requested.emit()
        self.thread.quit()
        self.thread.wait(1500)
        super().closeEvent(event)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setSpacing(12)

        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_control_group())
        layout.addWidget(self._build_log_group(), 1)
        self.setCentralWidget(root)

        self.setStyleSheet(
            """
            QMainWindow { background: #F6F8FA; }
            QGroupBox {
                font-weight: 600;
                border: 1px solid #C9D1D9;
                border-radius: 6px;
                margin-top: 12px;
                padding: 12px;
                background: #FFFFFF;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QLabel { color: #24292F; }
            QComboBox, QPushButton, QPlainTextEdit {
                min-height: 30px;
                border: 1px solid #C9D1D9;
                border-radius: 5px;
                padding: 4px 8px;
                background: #FFFFFF;
            }
            QPushButton {
                background: #0969DA;
                color: #FFFFFF;
                font-weight: 600;
            }
            QPushButton:disabled {
                background: #D0D7DE;
                color: #57606A;
            }
            QPlainTextEdit {
                font-family: Consolas, "Courier New", monospace;
                background: #0D1117;
                color: #C9D1D9;
            }
            """
        )

    def _build_connection_group(self) -> QGroupBox:
        box = QGroupBox("连接设置")
        layout = QGridLayout(box)
        layout.setColumnStretch(1, 1)

        layout.addWidget(QLabel("端口号"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(self.refresh_button, 0, 2)
        layout.addWidget(self.connect_button, 0, 3)
        layout.addWidget(self.disconnect_button, 0, 4)

        self.refresh_button.clicked.connect(self._refresh_ports)
        self.connect_button.clicked.connect(self._connect_serial)
        self.disconnect_button.clicked.connect(self.close_requested.emit)

        return box

    def _build_control_group(self) -> QGroupBox:
        box = QGroupBox("控制面板")
        layout = QGridLayout(box)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(3, 1)

        self.node_combo.addItems(NODES.keys())
        self.device_combo.addItems(DEVICE_TYPES.keys())
        self.action_combo.addItems(ACTION_MODES.keys())

        layout.addWidget(QLabel("目标节点"), 0, 0)
        layout.addWidget(self.node_combo, 0, 1)
        layout.addWidget(QLabel("控制对象"), 0, 2)
        layout.addWidget(self.device_combo, 0, 3)
        layout.addWidget(QLabel("执行动作"), 1, 0)
        layout.addWidget(self.action_combo, 1, 1)
        layout.addWidget(self.send_button, 1, 3)

        status_box = QWidget()
        status_layout = QHBoxLayout(status_box)
        status_layout.setContentsMargins(0, 8, 0, 0)
        for addr, title in TARGET_LABELS.items():
            label = QLabel(f"{title}: 未反馈")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setMinimumHeight(34)
            self.status_labels[addr] = label
            self._apply_status_style(label, QColor("#8C959F"))
            status_layout.addWidget(label)
        layout.addWidget(status_box, 2, 0, 1, 4)

        self.send_button.clicked.connect(self.send_command)
        return box

    def _build_log_group(self) -> QGroupBox:
        box = QGroupBox("实时日志")
        layout = QVBoxLayout(box)
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view, 1)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        buttons.addWidget(self.clear_button)
        buttons.addWidget(self.save_button)
        layout.addLayout(buttons)

        self.clear_button.clicked.connect(self.log_view.clear)
        self.save_button.clicked.connect(self._save_log)
        return box

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)
        self._append_log(f"刷新端口：{', '.join(ports) if ports else '未发现可用串口'}")

    def _connect_serial(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            self._show_error("请选择串口端口")
            return
        self.open_requested.emit(port)

    def _set_connected_state(self, connected: bool) -> None:
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.send_button.setEnabled(connected)
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)

    @pyqtSlot(str)
    def _on_connected(self, port: str) -> None:
        self._set_connected_state(True)
        self._append_log(f"连接成功：{port}")

    @pyqtSlot()
    def _on_disconnected(self) -> None:
        self._set_connected_state(False)
        self._append_log("串口已断开")

    @pyqtSlot(str)
    def _show_error(self, message: str) -> None:
        self._append_log(f"ERROR: {message}")
        QMessageBox.warning(self, "提示", message)

    def _append_log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(f"[{stamp}] {message}")
        self.log_view.moveCursor(QTextCursor.MoveOperation.End)

    def _apply_status_style(self, label: QLabel, color: QColor) -> None:
        label.setStyleSheet(
            f"""
            QLabel {{
                color: #24292F;
                background: {color.name()};
                border-radius: 5px;
                padding: 4px 8px;
                font-weight: 600;
            }}
            """
        )

    @pyqtSlot(int, int)
    def _update_node_state(self, node_addr: int, state: int) -> None:
        label = self.status_labels.get(node_addr)
        if label is None:
            return

        device_type = (state >> 4) & 0x0F
        mode = state & 0x0F
        device_name = "LED" if device_type == 0x01 else "Buzzer" if device_type == 0x02 else "设备"
        mode_name = {0x00: "关闭", 0x01: "开启", 0x02: "闪烁"}.get(mode, f"0x{mode:02X}")
        label.setText(f"{TARGET_LABELS.get(node_addr, f'0x{node_addr:04X}')}: {device_name} {mode_name}")

        color = QColor("#2DA44E") if mode in (0x01, 0x02) else QColor("#8C959F")
        self._apply_status_style(label, color)

    def send_command(self) -> None:
        """将界面选择封装为 AA 开头、55 结尾的 6 字节控制包。"""
        node_addr = NODES[self.node_combo.currentText()]
        device_type = DEVICE_TYPES[self.device_combo.currentText()]
        mode = ACTION_MODES[self.action_combo.currentText()]

        packet = bytes(
            [
                PACKET_HEAD,
                (node_addr >> 8) & 0xFF,
                node_addr & 0xFF,
                device_type,
                mode,
                PACKET_TAIL,
            ]
        )
        self.send_requested.emit(packet)
        self._append_log(
            f"指令：addr=0x{node_addr:04X}, type=0x{device_type:02X}, mode=0x{mode:02X}"
        )

    def _save_log(self) -> None:
        default_path = Path.cwd() / f"mesh_log_{datetime.now():%Y%m%d_%H%M%S}.txt"
        path, _ = QFileDialog.getSaveFileName(
            self,
            "保存实验数据",
            str(default_path),
            "Text Files (*.txt);;All Files (*)",
        )
        if not path:
            return

        Path(path).write_text(self.log_view.toPlainText(), encoding="utf-8")
        self._append_log(f"日志已保存：{path}")


def main() -> int:
    app = QApplication(sys.argv)
    window = MeshControlWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
