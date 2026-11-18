#!/usr/bin/env python3
"""온도 모니터링 + 4축 로봇암 제어 GUI (PyQt5)

서버 두 포트에 TCP 클라이언트로 접속한다.
  - 센서 포트 (기본 8080): 4바이트 패킷 'A' 'T' <온도> 'D' 를 받아 숫자와 그래프로 표시
  - 로봇 포트 (기본 8081): 슬라이더 값을 "a90b45c120d60e\\n" 형식으로 전송

사용법:
  python3 gui/monitor_gui.py                      # 127.0.0.1 의 8080 / 8081 에 접속
  python3 gui/monitor_gui.py --host 192.168.0.10  # 다른 PC의 서버에 접속

서버는 integrated_server(둘 다 동시) 또는 sensor_server / robot_server(각각) 어느 쪽이든 된다.
접속이 안 되거나 끊기면 2초마다 자동으로 다시 시도한다.
"""

import argparse
import sys
import time
from collections import deque

from PyQt5 import QtCore, QtGui, QtNetwork, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

RECONNECT_MS = 2000       # 재접속 시도 간격
GRAPH_WINDOW_S = 120      # 그래프에 보여줄 시간 범위(초)
GRAPH_UPDATE_MS = 500     # 그래프 갱신 주기
STALE_AFTER_S = 3.0       # 이 시간 동안 패킷이 없으면 "데이터 없음"으로 표시
SEND_THROTTLE_MS = 50     # 슬라이더를 끄는 동안 명령을 보내는 최소 간격

# 축 설정: (이름, 프로토콜 문자, 최소 각도, 최대 각도, 시작 각도)
# 로봇암 기구에 걸리는 각도가 있으면 여기서 최소/최대를 좁히면 된다.
AXES = [
    ("Base", "a", 0, 180, 90),
    ("Shoulder", "b", 0, 180, 90),
    ("Upperarm", "c", 0, 180, 90),
    ("Forearm", "d", 0, 180, 90),
]


class ReconnectingSocket(QtCore.QObject):
    """끊기면 자동으로 다시 접속하는 TCP 소켓."""

    state_changed = QtCore.pyqtSignal(bool)   # True = 접속됨
    data_received = QtCore.pyqtSignal(bytes)

    def __init__(self, host, port, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self.sock = QtNetwork.QTcpSocket(self)
        self.sock.connected.connect(self._on_connected)
        self.sock.disconnected.connect(self._on_disconnected)
        self.sock.readyRead.connect(self._on_ready_read)

        self.retry = QtCore.QTimer(self)
        self.retry.setInterval(RECONNECT_MS)
        self.retry.timeout.connect(self._try_connect)

    def start(self):
        self._try_connect()
        self.retry.start()

    def is_connected(self):
        return self.sock.state() == QtNetwork.QAbstractSocket.ConnectedState

    def send(self, data):
        if not self.is_connected():
            return False
        self.sock.write(data)
        self.sock.flush()
        return True

    def stop(self):
        """창을 닫을 때 호출: 재접속을 멈추고 소켓을 닫는다."""
        self.retry.stop()
        self.sock.disconnected.disconnect(self._on_disconnected)
        self.sock.abort()

    def _on_connected(self):
        self.state_changed.emit(True)

    def _on_disconnected(self):
        self.state_changed.emit(False)

    def _try_connect(self):
        if self.sock.state() == QtNetwork.QAbstractSocket.UnconnectedState:
            self.sock.connectToHost(self.host, self.port)

    def _on_ready_read(self):
        self.data_received.emit(bytes(self.sock.readAll()))


class SensorPanel(QtWidgets.QGroupBox):
    """현재 온도 숫자 + 최근 온도 그래프."""

    def __init__(self, host, port, parent=None):
        super().__init__("온도 모니터", parent)
        self.buffer = bytearray()        # 아직 4바이트 패킷으로 잘리지 않은 수신 데이터
        self.latest = None               # 가장 최근 온도
        self.last_rx = 0.0               # 마지막 패킷 수신 시각
        self.samples = deque()           # (시각, 온도)

        self.status = QtWidgets.QLabel()
        self.value = QtWidgets.QLabel("-- °C")
        font = self.value.font()
        font.setPointSize(40)
        font.setBold(True)
        self.value.setFont(font)
        self.value.setAlignment(QtCore.Qt.AlignCenter)

        self.figure = Figure(figsize=(5, 3), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111)
        # 온도는 정수 단위로 오므로 계단 모양으로 그린다
        (self.line,) = self.ax.plot([], [], drawstyle="steps-post", linewidth=2)
        self.ax.set_xlabel("Seconds ago")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.set_xlim(-GRAPH_WINDOW_S, 0)
        self.ax.set_ylim(15, 35)
        self.ax.grid(True, alpha=0.3)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.value)
        layout.addWidget(self.canvas, 1)

        self.conn = ReconnectingSocket(host, port, self)
        self.conn.state_changed.connect(self._on_state)
        self.conn.data_received.connect(self._on_data)
        self._on_state(False)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(GRAPH_UPDATE_MS)
        self.timer.timeout.connect(self._refresh)

    def start(self):
        self.conn.start()
        self.timer.start()

    def _on_state(self, connected):
        where = f"{self.conn.host}:{self.conn.port}"
        if connected:
            self.status.setText(f"● 센서 서버 접속됨 ({where})")
            self.status.setStyleSheet("color: #1a7f37;")
        else:
            self.status.setText(f"○ 센서 서버 접속 대기 중... ({where})")
            self.status.setStyleSheet("color: #b35900;")
            self.buffer.clear()
            self.latest = None           # 끊기면 바로 "-- °C"로 표시

    def _on_data(self, data):
        """ABCD 프로토콜 파싱: 'A' 'T' <값> 'D' (4바이트)."""
        self.buffer.extend(data)
        while len(self.buffer) >= 4:
            if self.buffer[0] != ord("A") or self.buffer[3] != ord("D"):
                del self.buffer[0]       # 동기화가 깨졌으면 1바이트씩 버리며 재정렬
                continue
            if self.buffer[1] == ord("T"):
                value = self.buffer[2]
                if value > 127:          # signed char (영하 온도)
                    value -= 256
                self.latest = value
                self.last_rx = time.monotonic()
            del self.buffer[:4]

    def _refresh(self):
        now = time.monotonic()
        fresh = self.latest is not None and now - self.last_rx <= STALE_AFTER_S

        if fresh:
            self.value.setText(f"{self.latest} °C")
            self.samples.append((now, self.latest))
        else:
            self.value.setText("-- °C")   # 접속은 됐지만 값이 안 오는 경우 포함

        while self.samples and now - self.samples[0][0] > GRAPH_WINDOW_S:
            self.samples.popleft()

        xs = [t - now for t, _ in self.samples]
        ys = [v for _, v in self.samples]
        self.line.set_data(xs, ys)
        if ys:
            self.ax.set_ylim(min(ys) - 3, max(ys) + 3)
        self.canvas.draw_idle()


class RobotPanel(QtWidgets.QGroupBox):
    """4축 슬라이더로 로봇암 각도 명령 전송."""

    def __init__(self, host, port, parent=None):
        super().__init__("로봇암 제어", parent)
        self.sliders = []
        self.last_sent = None

        self.status = QtWidgets.QLabel()
        grid = QtWidgets.QGridLayout()
        for row, (name, _, lo, hi, start) in enumerate(AXES):
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(lo, hi)
            slider.setValue(start)
            slider.setPageStep(5)
            slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
            slider.setTickInterval(30)
            spin = QtWidgets.QSpinBox()
            spin.setRange(lo, hi)
            spin.setValue(start)
            spin.setSuffix("°")
            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)
            slider.valueChanged.connect(self._schedule_send)
            grid.addWidget(QtWidgets.QLabel(name), row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            self.sliders.append(slider)
        grid.setColumnStretch(1, 1)

        self.home_button = QtWidgets.QPushButton("90° 복귀 (똑바로 선 자세)")
        self.home_button.clicked.connect(self._go_home)
        self.sent_label = QtWidgets.QLabel("보낸 명령: (없음)")
        self.sent_label.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        note = QtWidgets.QLabel("슬라이더를 움직이면 바로 전송됩니다. 로봇암은 1°씩 천천히 따라옵니다.")
        note.setWordWrap(True)
        note.setStyleSheet("color: gray;")

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addLayout(grid)
        layout.addWidget(self.home_button)
        layout.addWidget(self.sent_label)
        layout.addWidget(note)
        layout.addStretch(1)

        # 슬라이더를 끄는 동안 명령이 수백 개씩 쏟아지지 않도록 묶어서 보낸다
        self.send_timer = QtCore.QTimer(self)
        self.send_timer.setSingleShot(True)
        self.send_timer.setInterval(SEND_THROTTLE_MS)
        self.send_timer.timeout.connect(self._send_now)

        self.conn = ReconnectingSocket(host, port, self)
        self.conn.state_changed.connect(self._on_state)
        self._on_state(False)

    def start(self):
        self.conn.start()

    def packet(self):
        """현재 슬라이더 값으로 프로토콜 문자열 생성: a<base>b<shoulder>c<upperarm>d<forearm>e\\n"""
        body = "".join(f"{letter}{slider.value()}"
                       for (_, letter, *_), slider in zip(AXES, self.sliders))
        return (body + "e\n").encode("ascii")

    def _on_state(self, connected):
        where = f"{self.conn.host}:{self.conn.port}"
        if connected:
            self.status.setText(f"● 로봇 서버 접속됨 ({where})")
            self.status.setStyleSheet("color: #1a7f37;")
        else:
            self.status.setText(f"○ 로봇 서버 접속 대기 중... ({where})")
            self.status.setStyleSheet("color: #b35900;")
        # 접속 안 된 상태에서는 조작해도 전송되지 않으므로 비활성화
        for slider in self.sliders:
            slider.setEnabled(connected)
        self.home_button.setEnabled(connected)
        self.last_sent = None

    def _schedule_send(self):
        if not self.send_timer.isActive():
            self.send_timer.start()

    def _send_now(self):
        data = self.packet()
        if data == self.last_sent:
            return
        if self.conn.send(data):
            self.last_sent = data
            self.sent_label.setText("보낸 명령: " + data.decode().strip())

    def _go_home(self):
        for (_, _, _, _, start), slider in zip(AXES, self.sliders):
            slider.setValue(start)
        self._schedule_send()


class MainWindow(QtWidgets.QWidget):
    def __init__(self, host, sensor_port, robot_port):
        super().__init__()
        self.setWindowTitle("Arduino 온도 모니터 & 로봇암 제어")
        self.sensor = SensorPanel(host, sensor_port)
        self.robot = RobotPanel(host, robot_port)

        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(self.sensor, 3)
        layout.addWidget(self.robot, 2)
        self.resize(980, 460)

    def start(self):
        self.sensor.start()
        self.robot.start()

    def closeEvent(self, event):
        self.sensor.timer.stop()
        self.sensor.conn.stop()
        self.robot.conn.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="온도 모니터링 + 로봇암 제어 GUI")
    parser.add_argument("--host", default="127.0.0.1", help="서버 IP (기본 127.0.0.1)")
    parser.add_argument("--sensor-port", type=int, default=8080, help="센서 포트 (기본 8080)")
    parser.add_argument("--robot-port", type=int, default=8081, help="로봇 포트 (기본 8081)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv[:1])
    window = MainWindow(args.host, args.sensor_port, args.robot_port)
    window.show()
    window.start()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
