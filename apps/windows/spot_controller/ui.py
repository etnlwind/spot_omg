"""Native Qt desktop shell with an owned MuJoCo process and read-only video."""
import codecs
import os
from pathlib import Path
import socket
import sys
import time
import tempfile
import uuid

from PySide6.QtCore import Qt, QTimer, Signal, QProcess, QProcessEnvironment, QSettings, QUrl, QEvent
from PySide6.QtGui import QColor, QPainter, QPen, QFont, QPixmap, QTextCursor, QFontDatabase
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkRequest
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QPlainTextEdit, QFrame, QTabWidget, QMessageBox, QCheckBox, QSpinBox,
    QFileDialog, QSizePolicy, QScrollArea, QDialog)

from . import __version__
from .connection import Connection
from .foot_lift import FootLiftEditor
from .protocol import PROFILES, NATIVE_PROFILES, SIMULATOR_NATIVE_PROFILES, READ_ONLY

STYLE = """
QMainWindow, QDialog, QWidget#shell { background: #eef2ef; color: #162e26; }
QWidget { font-family: 'Malgun Gothic'; font-size: 12px; color: #243e33; }
QFrame#card { background: white; border: 1px solid #d9e2dc; border-radius: 14px; }
QLabel#title { font-size: 28px; font-weight: 800; color: #163c2d; }
QLabel#muted { color: #698074; font-size: 11px; }
QLabel#section { font-size: 14px; font-weight: 700; }
QLabel#value { font-size: 20px; font-weight: 700; }
QPushButton { background: #f1f5f2; border: 1px solid #d6e0d9; border-radius: 7px; padding: 9px 12px; font-weight: 600; }
QPushButton:hover { background: #e0ece4; border-color: #79a88b; }
QPushButton:pressed { background: #cfe0d4; }
QPushButton:disabled { color: #a3afa7; background: #f4f6f4; border-color: #e8eee9; }
QPushButton#primary { background: #176b45; color: white; border-color: #176b45; }
QPushButton#primary:hover { background: #218456; }
QPushButton#primary:disabled { background: #8da99a; border-color: #8da99a; }
QPushButton#stop { background: #b83c36; color: white; border: none; font-size: 14px; padding: 12px 25px; }
QPushButton#stop:hover { background: #d94c42; }
QComboBox, QLineEdit, QSpinBox { background: #f7f9f7; border: 1px solid #d6e0d9; border-radius: 6px; padding: 7px; min-height: 18px; }
QComboBox:disabled, QLineEdit:disabled { color: #8b9c91; }
QComboBox QAbstractItemView { background: white; selection-background-color: #d9edde; color: #163c2d; }
QTabWidget::pane { border: none; }
QTabBar::tab { background: #e3ebe5; padding: 9px 18px; margin-right: 4px; border-top-left-radius: 7px; border-top-right-radius: 7px; }
QTabBar::tab:selected { background: #173a2b; color: #f0fff4; }
QPlainTextEdit { border: none; border-radius: 8px; background: #10261e; color: #a1e9b3; font-family: Consolas; font-size: 12px; padding: 10px; }
QLabel#error { color: #a93832; background: #fbeeea; padding: 8px; border-radius: 6px; }
"""


def label(text, name=None):
    w = QLabel(text)
    if name:
        w.setObjectName(name)
    return w


def card(title=None):
    frame = QFrame()
    frame.setObjectName("card")
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(18, 16, 18, 16)
    layout.setSpacing(10)
    if title:
        layout.addWidget(label(title, "section"))
    return frame, layout


class Joystick(QWidget):
    vectorChanged = Signal(object)
    releaseReason = Signal(str)

    def __init__(self):
        super().__init__()
        self.vector = (0., 0.)
        self.held = False
        self.setMinimumSize(230, 230)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setAccessibleName("조종 스틱: 드래그하여 이동, 놓으면 정지")

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx, cy = self.width() / 2, self.height() / 2
        radius = min(cx, cy) - 17
        p.setPen(QPen(QColor("#315b45"), 2))
        p.setBrush(QColor("#112f23"))
        p.drawEllipse(int(cx-radius), int(cy-radius), int(radius*2), int(radius*2))
        p.setPen(QPen(QColor("#315642"), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(int(cx-radius*.6), int(cy-radius*.6), int(radius*1.2), int(radius*1.2))
        p.drawLine(int(cx-radius*.85), int(cy), int(cx+radius*.85), int(cy))
        p.drawLine(int(cx), int(cy-radius*.85), int(cx), int(cy+radius*.85))
        p.setPen(QColor("#8fc8a3"))
        p.setFont(QFont("Consolas", 9))
        p.drawText(int(cx-13), int(cy-radius*.77), "FWD")
        p.drawText(int(cx-13), int(cy+radius*.85), "REV")
        x, y = self.vector
        travel = radius * .64
        size = radius * .26
        p.setPen(QPen(QColor("#b5edc7" if self.isEnabled() else "#86938c"), 2))
        p.setBrush(QColor("#6ddd99" if self.isEnabled() else "#6c7e71"))
        p.drawEllipse(int(cx+x*travel-size), int(cy-y*travel-size), int(size*2), int(size*2))

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.isEnabled():
            self.held = True
            self.setFocus()
            self._move(event)

    def mouseMoveEvent(self, event):
        if self.held:
            self._move(event)

    def _move(self, event):
        cx, cy = self.width()/2, self.height()/2
        travel = (min(cx, cy)-17)*.64
        x, y = (event.position().x()-cx)/travel, (cy-event.position().y())/travel
        distance = max(1., (x*x+y*y)**.5)
        self.vector = (x/distance, y/distance)
        self.vectorChanged.emit(self.vector)
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.held = False
            if getattr(self,"auto_return",True):self.release("mouseReleaseEvent")

    def release(self, reason="explicit release"):
        if self.held or self.vector != (0., 0.):
            self.releaseReason.emit(f"스틱 해제: {reason}; held={self.held}; buttons={QApplication.mouseButtons()}")
        self.held = False
        self.vector = (0., 0.)
        self.vectorChanged.emit(None)
        self.update()

    def changeEvent(self, event):
        if event.type() == QEvent.Type.EnabledChange and not self.isEnabled():
            self.release("joystick disabled")
        super().changeEvent(event)


def find_repo():
    for parent in [Path(__file__).resolve(), Path(sys.executable).resolve(), Path.cwd()]:
        for candidate in [parent, *parent.parents]:
            if (candidate / "simulation/mujoco/virtual_robot.py").exists():
                return candidate
    return Path.cwd()


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spot OMG · Windows Controller")
        self.setMinimumSize(960, 650)
        available = self.screen().availableGeometry()
        self.resize(min(1240, available.width()-40), available.height()-40)
        self.settings = QSettings("SpotOMG", "WindowsController")
        self.connection = Connection()
        self.connection.changed.connect(self.on_state)
        self.connection.log.connect(self.append_log)
        self.connection.finished.connect(self.on_disconnected)
        self.snapshot = {}
        self.last_profile_read = None
        self.input_vector = None
        self.keys = set()
        self.exiting = False
        self.stop_sim_pending = False
        self.sim_ready = False
        self.sim_heartbeat = None
        self.sim_pulse_at = 0.
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.sim_output)
        self.process.finished.connect(self.sim_finished)
        self.process.errorOccurred.connect(lambda _: self.show_error(self.process.errorString()))
        self.sim_decoder = codecs.getincrementaldecoder("utf-8")("replace")
        self.sim_log_tail = ""
        self.network = QNetworkAccessManager(self)
        self.video_reply = None
        self.last_frame = None
        self.last_frame_at = 0.
        self._build()
        self.pulse_timer = QTimer(self)
        self.pulse_timer.timeout.connect(self.pulse)
        self.pulse_timer.start(40)
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self.video_tick)
        self.video_timer.start(200)
        QApplication.instance().applicationStateChanged.connect(self.application_state)
        QApplication.instance().installEventFilter(self)
        self.on_state(dict(phase="offline", state={}, caps=[], connected=False, error="", can_drive=False))

    def _build(self):
        shell = QWidget()
        shell.setObjectName("shell")
        shell.setMinimumSize(1120, 920)
        scroll = QScrollArea()
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setWidget(shell)
        container = QWidget()
        root = QVBoxLayout(container)
        root.setContentsMargins(0, 0, 0, 0)
        self.battery_level = self.battery_acknowledged = 0
        self.battery_panel = QFrame()
        self.battery_panel.setObjectName("batteryWarningBanner")
        self.battery_panel.setStyleSheet("QFrame#batteryWarningBanner { background: #ae0f15; } QLabel { color: white; } QPushButton { color: #ae0f15; background: white; }")
        warning_layout = QVBoxLayout(self.battery_panel)
        warning_layout.setContentsMargins(18, 12, 18, 12)
        self.battery_title = QLabel()
        self.battery_title.setStyleSheet("font-size: 20px; font-weight: 800; color: white;")
        self.battery_title.setWordWrap(True)
        warning_layout.addWidget(self.battery_title)
        self.battery_message = QLabel()
        self.battery_message.setWordWrap(True)
        warning_layout.addWidget(self.battery_message)
        self.battery_actions = QWidget()
        warning_actions = QHBoxLayout(self.battery_actions)
        warning_actions.setContentsMargins(0, 0, 0, 0)
        self.battery_stop = QPushButton("보행 정지")
        self.battery_stop.clicked.connect(self.walk_stop)
        warning_actions.addWidget(self.battery_stop)
        warning_actions.addStretch()
        self.battery_confirm = QPushButton("확인")
        self.battery_confirm.clicked.connect(self.acknowledge_battery)
        warning_actions.addWidget(self.battery_confirm)
        warning_layout.addWidget(self.battery_actions)
        root.addWidget(self.battery_panel)
        self.motion_panel=QFrame()
        self.motion_panel.setMinimumHeight(132)
        self.motion_panel.setObjectName('motionWarningBanner')
        self.motion_panel.setStyleSheet('QFrame#motionWarningBanner { background: #8a4300; border-radius: 10px; } QLabel { color: #ffffff; background: transparent; }')
        motion_layout=QVBoxLayout(self.motion_panel)
        motion_layout.setContentsMargins(18,12,18,12)
        self.motion_title=QLabel()
        self.motion_title.setStyleSheet('font-size: 20px; font-weight: 800; color: white;')
        self.motion_message=QLabel()
        self.motion_message.setStyleSheet('font-size: 14px; color: white; background: transparent;')
        self.motion_message.setMinimumHeight(60)
        for message_label in (self.motion_title,self.motion_message):
            message_label.setWordWrap(True);motion_layout.addWidget(message_label)
        self.motion_panel.hide()
        root.addWidget(self.motion_panel)
        root.addWidget(scroll)
        self.setCentralWidget(container)
        outer = QVBoxLayout(shell)
        outer.setContentsMargins(24, 20, 24, 16)
        outer.setSpacing(16)
        header = QHBoxLayout()
        title = QVBoxLayout()
        title.addWidget(label("Spot OMG", "title"))
        title.addWidget(label(f"WINDOWS CONTROL STATION   /   v{__version__}", "muted"))
        header.addLayout(title)
        header.addStretch()
        self.connection_badge = label("●  연결 안 됨", "section")
        header.addWidget(self.connection_badge)
        self.stop_button = QPushButton("■  STOP   /   SPACE")
        self.stop_button.setObjectName("stop")
        self.stop_button.clicked.connect(self.walk_stop)
        self.stop_button.setToolTip("보행 정지 후 최초 자세 복귀 · Esc: 긴급 중단")
        header.addWidget(self.stop_button)
        outer.addLayout(header)
        content = QHBoxLayout()
        content.setSpacing(16)
        left = QVBoxLayout()
        connection_card, layout = card("01   연결 대상")
        connection_card.setFixedWidth(285)
        self.target = QComboBox()
        for name, key in [("MuJoCo · TCP", "tcp"), ("실제 로봇 · BLE", "robot"), ("가상 로봇 · BLE", "simble")]:
            self.target.addItem(name, key)
        layout.addWidget(self.target)
        row = QHBoxLayout()
        self.host = QLineEdit(str(self.settings.value("host", "127.0.0.1")))
        self.host.setPlaceholderText("시뮬레이터 IP")
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(8765)
        self.port.setFixedWidth(82)
        row.addWidget(self.host)
        row.addWidget(self.port)
        layout.addLayout(row)
        self.address = QLineEdit()
        self.address.setPlaceholderText("BLE 주소 (장치가 여러 대일 때)")
        layout.addWidget(self.address)
        self.connect_button = QPushButton("연결하기")
        self.connect_button.setObjectName("primary")
        self.connect_button.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_button)
        help_text = label("실제 로봇: iPhone 앱 연결을 먼저 해제하세요.\n연결 후 로봇 상태를 읽고 조작을 활성화합니다.", "muted")
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        left.addWidget(connection_card)
        sim_card, layout = card("02   로컬 MuJoCo")
        self.sim_status = label("시뮬레이터가 꺼져 있습니다.", "muted")
        self.sim_status.setWordWrap(True)
        layout.addWidget(self.sim_status)
        self.viewer = QCheckBox("별도 3D 뷰어도 열기")
        layout.addWidget(self.viewer)
        self.sim_button = QPushButton("▶  MuJoCo 시작 + 연결")
        self.sim_button.setObjectName("primary")
        self.sim_button.clicked.connect(self.toggle_sim)
        layout.addWidget(self.sim_button)
        self.repo = QLineEdit(str(self.settings.value("repo", str(find_repo()))))
        self.repo.setPlaceholderText("spot_omg 저장소 경로")
        layout.addWidget(label("프로젝트 / Python 실행 경로", "muted"))
        layout.addWidget(self.repo)
        default_python = sys.executable
        if getattr(sys, "frozen", False):
            default_python = str(Path.home()/"miniforge3/envs/spot_omg/python.exe")
        self.python = QLineEdit(str(self.settings.value("python", default_python)))
        layout.addWidget(self.python)
        self.video_port = QSpinBox()
        self.video_port.setRange(1, 65535)
        self.video_port.setValue(8766)
        port_row = QHBoxLayout()
        port_row.addWidget(label("영상 포트", "muted"))
        port_row.addWidget(self.video_port)
        layout.addLayout(port_row)
        left.addWidget(sim_card)
        details_card, layout = card("03   상태 동기화")
        self.firmware = label("펌웨어  —", "muted")
        self.firmware.setWordWrap(True)
        layout.addWidget(self.firmware)
        self.sync_button = QPushButton("↻  현재 상태 읽기")
        self.sync_button.clicked.connect(lambda: self.send("syncstate"))
        layout.addWidget(self.sync_button)
        self.time_button = QPushButton("로봇 시간 동기화")
        self.time_button.clicked.connect(lambda: self.send(f"log time {int(time.time()*1000)}"))
        layout.addWidget(self.time_button)
        left.addWidget(details_card)
        left.addStretch()
        content.addLayout(left)
        right = QVBoxLayout()
        metrics = QHBoxLayout()
        self.metrics = {}
        for key, name in [("pose", "자세"), ("voltage", "정지 기준 전압"), ("safety", "안전 상태"), ("profile", "보행 정책")]:
            frame, col = card()
            col.addWidget(label(name, "muted"))
            value = label("—", "value")
            value.setMinimumWidth(80)
            value.setWordWrap(True)
            col.addWidget(value)
            self.metrics[key] = value
            metrics.addWidget(frame)
        right.addLayout(metrics)
        self.foot_lift=FootLiftEditor(self.settings)
        self.foot_lift.applyRequested.connect(self.send)
        self.foot_lift_dialog=QDialog(self)
        self.foot_lift_dialog.setWindowTitle("Spot OMG · 발 들림 보정")
        self.foot_lift_dialog.resize(640,520)
        self.foot_lift_dialog.setModal(True)
        QVBoxLayout(self.foot_lift_dialog).addWidget(self.foot_lift)
        probe_card, probe_layout = card("파라미터 보행 시험 · V6.2.5")
        self.probe_mode=QComboBox();self.probe_mode.addItems(["기본 보행","파라미터 보행"])
        self.probe_mode.currentIndexChanged.connect(lambda index:self.send(f"app_parameter_mode {index}"))
        probe_layout.addWidget(self.probe_mode)
        fields = QGridLayout()
        self.probe_lift = QSpinBox(); self.probe_lift.setRange(12,40); self.probe_lift.setValue(28); self.probe_lift.setSuffix(" mm")
        self.probe_linear = QSpinBox(); self.probe_linear.setRange(1,1000); self.probe_linear.setValue(344)
        self.probe_duration = QSpinBox(); self.probe_duration.setRange(500,30000); self.probe_duration.setValue(4000); self.probe_duration.setSuffix(" ms")
        self.probe_fr_extra = QCheckBox("출발 시 FR만 추가 오므림")
        self.probe_width = QSpinBox();self.probe_width.setRange(-40,20);self.probe_width.setValue(0);self.probe_width.setSuffix(" mm")
        self.probe_legs = QComboBox(); self.probe_legs.addItems(["all","rl","rr"])
        for index, (title, widget) in enumerate([("들림",self.probe_lift),("전진 입력",self.probe_linear),("시간",self.probe_duration),("다리",self.probe_legs),("좌우 간격 · 한쪽 기준",self.probe_width),("첫걸음 옵션",self.probe_fr_extra)]):
            row, col = divmod(index,2)
            fields.addWidget(QLabel(title),row,col*2); fields.addWidget(widget,row,col*2+1)
        probe_layout.addLayout(fields)
        buttons = QHBoxLayout()
        self.probe_save=QPushButton("파라미터 저장");self.probe_save.clicked.connect(self.save_probe)
        self.probe_load=QPushButton("저장값 불러오기");self.probe_load.clicked.connect(self.load_probe)
        buttons.addWidget(self.probe_save);buttons.addWidget(self.probe_load)
        self.load_probe()
        self.probe_apply = QPushButton("설정 적용 + 조회")
        self.probe_apply.clicked.connect(lambda: self.send(f"probeconfig set {self.probe_lift.value()} {self.probe_linear.value()} {self.probe_duration.value()} {self.probe_legs.currentText()}" + (f" {self.probe_width.value()} {int(self.probe_fr_extra.isChecked())}" if self.snapshot.get("state",{}).get("rev") in ("s-native-v6-2-7-v77-t1-width", "attitudepd-v2-v78", "attitudepd-v3-v79", "attitudepd-v4-v80", "attitudepd-v4-v81", "attitudepd-v4-v82", "attitudepd-v4-v90-r1") else "")))
        self.probe_read = QPushButton("설정 조회"); self.probe_read.clicked.connect(lambda: self.send("probeconfig show"))
        self.probe_start = QPushButton("시험 시작"); self.probe_start.clicked.connect(lambda: self.send("app_probe_start"))
        for button in [self.probe_apply,self.probe_read,self.probe_start]:buttons.addWidget(button)
        probe_layout.addLayout(buttons)
        self.probe_status = label("V6.2.5 선택 → Stand → 설정 적용 → 시험 시작. 정지는 기존 Stop 버튼.", "muted")
        self.probe_status.setWordWrap(True);probe_layout.addWidget(self.probe_status)
        right.addWidget(probe_card)

        center = QHBoxLayout()
        drive_card, layout = card("조종 스틱")
        drive_card.setMinimumWidth(280)
        drive_card.setMinimumHeight(340)
        self.joystick = Joystick()
        self.joystick.vectorChanged.connect(self.set_vector)
        self.joystick.releaseReason.connect(self.connection.trace)
        self.auto_return_button=QPushButton("자동 복귀 ON");self.auto_return_button.setCheckable(True);self.auto_return_button.setChecked(True)
        self.auto_return_button.toggled.connect(self.toggle_auto_return)
        layout.addWidget(self.auto_return_button,0,Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.joystick, 1)
        self.keyboard = QCheckBox("키보드 조종 켜기  ·  WASD / 방향키")
        self.keyboard.toggled.connect(lambda _: self.release_input())
        layout.addWidget(self.keyboard)
        self.drive_status = label("중립  /  놓으면 정지", "muted")
        layout.addWidget(self.drive_status)
        center.addWidget(drive_card, 1)
        preview_card, preview_layout = card("MuJoCo · 실시간 화면")
        self.video = label("MuJoCo를 시작하면 여기에 표시됩니다.\n\n실제 로봇의 카메라 영상이 아닙니다.", "muted")
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(360, 205)
        self.video.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.video.setStyleSheet("background: #17291f; color: #94b7a0; border-radius: 8px;")
        preview_layout.addWidget(self.video, 1)
        self.video_status = label("추정 물성의 시뮬레이션 · 실기 검증과 구분", "muted")
        preview_layout.addWidget(self.video_status)
        center.addWidget(preview_card, 2)
        right.addLayout(center, 3)
        actions_card, layout = card()
        buttons = QHBoxLayout()
        self.motion_buttons = {}
        for command in ("landing", "stow", "stand", "stand11", "recover", "relax"):
            button = QPushButton(command.capitalize())
            button.clicked.connect(lambda _, c=command: self.send(c))
            buttons.addWidget(button)
            self.motion_buttons[command] = button
        layout.addLayout(buttons)
        policy_row = QHBoxLayout()
        self.profiles = QComboBox()
        self.profiles.setMinimumWidth(170)
        self.profiles.addItems(PROFILES)
        self.profile_button = QPushButton("정책 적용")
        self.profile_button.clicked.connect(self.select_profile)
        self.balance_button = QPushButton("수평 보정")
        self.balance_button.clicked.connect(self.toggle_balance)
        self.heading_button = QPushButton("직진 유지")
        self.heading_button.clicked.connect(lambda: self.send("heading " + ("off" if self.snapshot.get("state", {}).get("heading") == "on" else "on")))
        for w in (self.profiles, self.profile_button, self.balance_button, self.heading_button):
            policy_row.addWidget(w)
        self.foot_lift_button=QPushButton("발 들림 보정")
        self.foot_lift_button.clicked.connect(self.foot_lift.prepare_open)
        self.foot_lift_button.clicked.connect(self.foot_lift_dialog.open)
        policy_row.addWidget(self.foot_lift_button)
        layout.addLayout(policy_row)
        right.addWidget(actions_card)
        self.tabs = QTabWidget()
        terminal = QWidget()
        terminal_layout = QVBoxLayout(terminal)
        terminal_layout.setContentsMargins(0, 0, 0, 0)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.document().setMaximumBlockCount(800)
        self.console.setMinimumHeight(110)
        terminal_layout.addWidget(self.console)
        row = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("STM32 명령 입력 · Enter로 전송")
        self.command_input.returnPressed.connect(self.send_console)
        self.send_button = QPushButton("전송")
        self.send_button.clicked.connect(self.send_console)
        save = QPushButton("로그 저장")
        save.clicked.connect(self.save_log)
        clear = QPushButton("지우기")
        clear.clicked.connect(self.console.clear)
        for w in (self.command_input, self.send_button, save, clear):
            row.addWidget(w)
        terminal_layout.addLayout(row)
        self.tabs.addTab(terminal, "TERMINAL")
        diagnostics = QWidget()
        grid = QGridLayout(diagnostics)
        self.diagnostic_buttons = []
        for i, (name, cmd) in enumerate([
            ("관절 목표", "targets"), ("서보 검색", "scan"), ("보행 진단", "gaitdiag"),
            ("균형 진단", "baldiag"), ("IMU 진단", "imudiag"), ("이동 진단", "locomotiondiag"),
            ("저장 로그 64개", "log show 64"), ("PD 상태", "stabilize status"),
            ("개선 전진 · 3회", "trot5 3 844"), ("Trot4 · 1회", "trot4 1 1800"),
            ("Crab Left · 1회", "crab left 1 5000"), ("Crab Right · 1회", "crab right 1 5000")]):
            button = QPushButton(name)
            button.clicked.connect(lambda _, c=cmd: self.send(c))
            grid.addWidget(button, i//4, i%4)
            self.diagnostic_buttons.append((button, cmd))
        self.tabs.addTab(diagnostics, "진단 / 단일 보행")
        self.sim_console = QPlainTextEdit()
        self.sim_console.setReadOnly(True)
        self.sim_console.document().setMaximumBlockCount(400)
        self.tabs.addTab(self.sim_console, "SIMULATOR LOG")
        right.addWidget(self.tabs, 2)
        content.addLayout(right, 1)
        outer.addLayout(content, 1)
        self.error = label("", "error")
        self.error.setWordWrap(True)
        self.error.hide()
        outer.addWidget(self.error)
        footer = QHBoxLayout()
        footer.addWidget(label("200 ms heartbeat  ·  마우스/키 해제 및 앱 비활성화 시 정지", "muted"))
        footer.addStretch()
        self.footer_target = label("SIMULATION / TCP", "muted")
        footer.addWidget(self.footer_target)
        outer.addLayout(footer)
        self.target.currentIndexChanged.connect(self.target_changed)
        self.target_changed()
        for button in self.findChildren(QPushButton):
            button.setMinimumHeight(36)

    def target_changed(self):
        tcp = self.target.currentData() == "tcp"
        self.host.setVisible(tcp)
        self.port.setVisible(tcp)
        self.address.setVisible(not tcp)
        self.footer_target.setText("REAL ROBOT / BLE" if self.target.currentData() == "robot" else "SIMULATION / " + ("TCP" if tcp else "BLE"))

    def show_error(self, message):
        self.error.setText(message)
        self.error.setVisible(bool(message))

    def append_log(self, text):
        self.console.moveCursor(QTextCursor.MoveOperation.End)
        self.console.insertPlainText(text)
        self.console.ensureCursorVisible()

    def on_state(self, state):
        self.snapshot = state
        self.foot_lift.update_state(state)
        self.foot_lift_button.setEnabled(state.get("phase") in ("offline","idle"))
        phase = state["phase"]
        connected = state.get("connected", False)
        ready = connected and state.get("synced", False)
        idle = ready and phase == "idle"
        caps = set(state.get("caps", []))
        robot = state.get("state", {})
        stowed = robot.get("pose") in {"stow", "stow-paused"}
        names = {"offline": "연결 안 됨", "connecting": "연결 중", "busy": "응답 대기", "drive": "조종 중", "stopping": "정지 확인 중", "draining": "종료 응답 수신 중", "resync": "정지 후 상태 재확인", "idle": "연결됨"}
        self.connection_badge.setText("●  " + names.get(phase, phase))
        self.connect_button.setText("연결 해제" if connected else "연결 취소" if phase == "connecting" else "연결하기")
        locked = phase != "offline"
        for field in (self.target, self.host, self.port, self.address):
            field.setEnabled(not locked and not self.sim_ready)
        self.stop_button.setEnabled(connected)
        self.sync_button.setEnabled(ready and phase in {"idle", "drive"})
        self.time_button.setEnabled(idle)
        self.command_input.setEnabled(ready)
        self.send_button.setEnabled(ready)
        self.joystick.setEnabled(state.get("controls_enabled", state.get("can_drive", False)))
        self.keyboard.setEnabled(state.get("controls_enabled", state.get("can_drive", False)))
        parameter_mode=state.get('parameter_walking',False)
        self.probe_mode.blockSignals(True);self.probe_mode.setCurrentIndex(int(parameter_mode));self.probe_mode.blockSignals(False)
        self.probe_mode.setEnabled(idle)
        probe_idle = idle and parameter_mode and state.get('supports_probe',False) and not stowed
        for field in [self.probe_lift,self.probe_linear,self.probe_duration,self.probe_legs,self.probe_save,self.probe_load]:field.setEnabled(probe_idle)
        self.probe_width.setEnabled(probe_idle and robot.get("rev") in ("s-native-v6-2-7-v77-t1-width", "attitudepd-v2-v78", "attitudepd-v3-v79", "attitudepd-v4-v80", "attitudepd-v4-v81", "attitudepd-v4-v82", "attitudepd-v4-v90-r1"))
        self.probe_fr_extra.setEnabled(self.probe_width.isEnabled())
        self.probe_fr_extra.setToolTip("해제: 좌우 동일. 체크: 안쪽 간격에서 첫걸음 FR J1 변화량 2배, 다음 걸음에 복귀")
        self.probe_width.setToolTip("수직 0mm · 안쪽 음수 · 바깥 양수. 한쪽 발 기준입니다.")
        self.probe_apply.setEnabled(probe_idle); self.probe_read.setEnabled(probe_idle)
        self.probe_start.setEnabled(probe_idle and bool(state.get('probe_config')) and robot.get('pose')=='stand' and robot.get('profile')=='s_native_v6_2_5' and robot.get('safety')=='ok')
        if state.get('probe_running') and not parameter_mode:
            self.joystick.setEnabled(False);self.keyboard.setEnabled(False)
        applied=state.get('probe_config')
        self.probe_status.setText((f"적용값: 들림 {applied[0]}mm / 입력 {applied[1]} / {applied[2]}ms / {applied[3]} / 간격 {str(applied[4])+'mm' if len(applied)>4 else '기존 로직'} / FR 추가 {'ON' if len(applied)>5 and applied[5] else 'OFF'} · " if applied else "설정 조회 필요 · ")+"V6.2.5 → Stand → 시험 시작 / Stop으로 정지")

        for cmd, button in self.motion_buttons.items():
            allowed = ready and phase in {"idle", "drive"}
            if stowed:
                allowed = allowed and (cmd == "landing" or (cmd == "stow" and robot.get("pose") == "stow-paused"))
            if cmd == "stow":
                allowed = allowed and bool(caps & {"stow", "simstow"})
            button.setEnabled(allowed)
        profiles_ok = idle and not stowed and bool(caps & {"gaitprofiles", "simprofiles"})
        self.profiles.setEnabled(profiles_ok)
        self.profile_button.setEnabled(profiles_ok)
        for index, profile in enumerate(PROFILES):
            allowed = not (profile.startswith("cushion_") or profile in SIMULATOR_NATIVE_PROFILES) or state.get("simulator", False)
            allowed &= profile not in {*NATIVE_PROFILES, "attitudepd_v4", "attitudepd_v3", "attitudepd_v2", "attitudepd", "centerpivot", "arcsupport"} or profile in caps
            self.profiles.model().item(index).setEnabled(allowed)
        self.balance_button.setEnabled(idle and not stowed and bool(caps & {"balancecontrol", "simbalance"}))
        self.heading_button.setEnabled(idle and not stowed and "headinghold" in caps)
        self.balance_button.setText("수평: " + robot.get("balance", "—"))
        self.heading_button.setText("직진: " + robot.get("heading", "—"))
        for button, cmd in self.diagnostic_buttons:
            button.setEnabled(idle and (not stowed or cmd in READ_ONLY) and (not cmd.startswith("trot5") or "trot5" in caps))
        self.metrics["pose"].setText(robot.get("pose", "—").capitalize())
        self.metrics["safety"].setText(robot.get("safety", "—").upper())
        profile = robot.get("profile", "—")
        if profile in PROFILES and profile != self.last_profile_read:
            self.profiles.setCurrentText(profile)
            self.last_profile_read = profile
        if phase == 'offline':
            self.last_profile_read = None
        self.metrics["profile"].setText(profile if len(profile) <= 18 else profile[:17] + '…')
        self.metrics["profile"].setToolTip(profile)
        voltage = state.get("voltage")
        stale = time.monotonic() - (state.get("voltage_at") or 0) > 15
        self.metrics["voltage"].setText(f"≈ {voltage:.1f} V" + (" · 이전" if stale else "") if voltage else "안정 전압 측정 대기")
        self.show_battery_warning(state.get("battery_warning", {}), state.get("simulator", False))
        from .motion_warning import motion_warning
        warning_title,warning_message=motion_warning(state.get('pause_reason',''))
        self.motion_panel.setVisible(bool(warning_title) and state.get('connected',False))
        self.motion_title.setText(warning_title)
        self.motion_message.setText(warning_message)
        self.motion_panel.setAccessibleName(warning_title+' '+warning_message)
        if warning_title:self.metrics['safety'].setText('일시정지')
        self.firmware.setText("펌웨어  " + robot.get("rev", "—") + "\n토크  " + robot.get("torque", "—") +
                              (" · 전용 제어 채널" if state.get("separate_control") else ""))
        drive_text = "조종 중  /  놓으면 정지" if phase == "drive" else names.get(phase, phase)
        if ready and not state.get("controls_enabled", state.get("can_drive", False)):
            drive_text = "자세 전환 중 · 완료 후 조작 가능"
        elif ready and robot.get("safety") != "ok":
            drive_text = "보호 정지 · 스틱을 놓고 다시 조작하면 재시도"
        self.drive_status.setText(drive_text)
        self.show_error(state.get("error", ""))

    def show_battery_warning(self, warning, simulator=False):
        level = 0 if simulator else warning.get('level', 0)
        if level > self.battery_level:
            QApplication.beep()
            QApplication.alert(self, 5000)
        if not level:
            self.battery_acknowledged = 0
        self.battery_level = level
        self.battery_panel.setVisible(level > 0)
        self.battery_title.setText(('가상 로봇 · ' if simulator else '') + warning.get('title', ''))
        self.battery_message.setText(warning.get('message', ''))
        self.battery_panel.setAccessibleName(self.battery_title.text() + ' ' + self.battery_message.text())
        self.battery_stop.setEnabled(self.snapshot.get('connected', False))
        self.battery_message.setVisible(level > self.battery_acknowledged)
        self.battery_actions.setVisible(level > self.battery_acknowledged)

    def acknowledge_battery(self):
        self.battery_acknowledged = self.battery_level
        self.battery_message.hide()
        self.battery_actions.hide()

    def toggle_connection(self):
        self.release_input()
        if self.connection.running:
            self.connection.disconnect()
        else:
            self.settings.setValue("host", self.host.text().strip())
            self.connection.connect_to(self.target.currentData(), self.host.text().strip(), self.port.value(), self.address.text())

    def send(self, line):
        if line == "relax":
            self.release_input()
            if QMessageBox.question(self, "Relax", "Landing 완료와 자세 readback을 확인한 뒤 토크를 해제합니다.\n로봇 몸체를 지지했습니까?",
                                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
                return
        self.connection.command(line)

    def send_console(self):
        value = self.command_input.text().strip()
        if value:
            self.send(value)
            self.command_input.clear()

    def select_profile(self):
        key = "gaitprofile" if "gaitprofiles" in self.snapshot.get("caps", []) else "simprofile"
        self.send(key + " " + self.profiles.currentText())

    def toggle_balance(self):
        key = "balance" if "balancecontrol" in self.snapshot.get("caps", []) else "simbalance"
        enabled = self.snapshot.get("state", {}).get("balance") not in {"off", "unknown", None}
        self.send(key + (" off" if enabled else " on"))

    def save_probe(self):
        import json
        value=[self.probe_lift.value(),self.probe_linear.value(),self.probe_duration.value(),self.probe_legs.currentText(),self.probe_width.value(),int(self.probe_fr_extra.isChecked())]
        self.settings.setValue("walkingParameters",json.dumps(value));self.settings.sync()
        self.probe_status.setText("이 앱에 저장 완료 · 로봇에는 설정 적용 + 조회를 실행하세요.")

    def load_probe(self):
        import json
        from .protocol import Controller
        try:value=Controller.parse_probe(json.loads(self.settings.value("walkingParameters","[28,344,4000,\"all\",0,0]")))
        except (ValueError,TypeError):return
        self.probe_lift.setValue(value[0]);self.probe_linear.setValue(value[1]);self.probe_duration.setValue(value[2]);self.probe_legs.setCurrentText(value[3]);self.probe_width.setValue(value[4]);self.probe_fr_extra.setChecked(bool(value[5]))

    def toggle_auto_return(self, enabled):
        self.joystick.auto_return=enabled
        self.auto_return_button.setText("자동 복귀 ON" if enabled else "자동 복귀 OFF · 입력 유지")
        if enabled:self.release_input()

    def set_vector(self, vector):
        if vector is None:
            self.keys.clear()
        self.input_vector = vector
        self.joystick.vector = vector if vector is not None else (0., 0.)
        self.joystick.update()
        self.connection.pulse(vector)

    def release_input(self, reason="release_input"):
        self.keys.clear()
        self.joystick.release(reason)
        self.input_vector = None
        self.connection.pulse(None)

    def pulse(self):
        if self.keys and isinstance(QApplication.focusWidget(), (QLineEdit, QPlainTextEdit, QSpinBox, QComboBox)):
            self.release_input()
        self.connection.pulse(self.input_vector)
        if self.sim_heartbeat and time.monotonic() >= self.sim_pulse_at:
            try:
                self.sim_heartbeat.touch()
            except OSError as exc:
                self.show_error('MuJoCo heartbeat: ' + str(exc))
            self.sim_pulse_at = time.monotonic() + .4

    def walk_stop(self):
        self.release_input("Stop button")
        self.connection.stop(graceful=True)

    def emergency_stop(self):
        self.release_input("emergency button/key")
        self.connection.stop()

    def application_state(self, state):
        if state != Qt.ApplicationState.ApplicationActive:
            self.release_input(f"application state {state}")
            if self.snapshot.get("motion_active"):
                self.connection.stop()

    def eventFilter(self, watched, event):
        if event.type() in {QEvent.Type.ShortcutOverride, QEvent.Type.KeyPress, QEvent.Type.KeyRelease} and self.isActiveWindow():
            key = event.key()
            typing = isinstance(QApplication.focusWidget(), (QLineEdit, QPlainTextEdit, QSpinBox, QComboBox))
            if event.type() == QEvent.Type.KeyPress and (key == Qt.Key.Key_Escape or (key == Qt.Key.Key_Space and not typing)):
                if not event.isAutoRepeat():
                    if key == Qt.Key.Key_Escape:
                        self.emergency_stop()
                    else:
                        self.walk_stop()
                return True
            directions = {Qt.Key.Key_W: (0, 1), Qt.Key.Key_Up: (0, 1), Qt.Key.Key_S: (0, -1), Qt.Key.Key_Down: (0, -1),
                          Qt.Key.Key_A: (-1, 0), Qt.Key.Key_Left: (-1, 0), Qt.Key.Key_D: (1, 0), Qt.Key.Key_Right: (1, 0)}
            if key in directions and self.keyboard.isChecked():
                controls_robot = key in self.keys or (not typing and self.snapshot.get("controls_enabled", self.snapshot.get("can_drive")))
                if event.type() == QEvent.Type.ShortcutOverride:
                    if controls_robot:
                        event.accept()
                        return True
                    return False
                if event.isAutoRepeat():
                    # Qt sends repeated press/release pairs while a key is held.
                    # Consume them without releasing motion or scrolling widgets.
                    return bool(controls_robot)
                if event.type() == QEvent.Type.KeyRelease:
                    self.keys.discard(key)
                elif typing or not self.snapshot.get("controls_enabled", self.snapshot.get("can_drive")):
                    return False
                else:
                    self.keys.add(key)
                if not self.keys:
                    self.release_input()
                else:
                    x = sum(directions[k][0] for k in self.keys)
                    y = sum(directions[k][1] for k in self.keys)
                    magnitude = max(1., (x*x+y*y)**.5)
                    self.set_vector((x/magnitude, y/magnitude))
                return True
        return super().eventFilter(watched, event)

    def toggle_sim(self):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.stop_sim_pending = True
            self.release_input()
            if self.connection.running:
                self.connection.disconnect()
            else:
                self.stop_sim()
            return
        if self.connection.running:
            self.show_error("현재 연결을 해제한 뒤 로컬 MuJoCo를 시작하십시오.")
            return
        repo = Path(self.repo.text()).expanduser().resolve()
        python = Path(self.python.text()).expanduser()
        script = repo / "simulation/mujoco/virtual_robot.py"
        if not script.is_file() or not python.is_file():
            self.show_error("프로젝트와 Python 경로를 확인하십시오. 최초 설정: apps/windows/setup.ps1")
            return
        try:
            for port in (self.port.value(), self.video_port.value()):
                with socket.socket() as probe:
                    probe.bind(("127.0.0.1", port))
            if self.port.value() == self.video_port.value():
                raise ValueError("제어 포트와 영상 포트는 달라야 합니다.")
        except (OSError, ValueError) as exc:
            self.show_error("시뮬레이터 포트 확인: " + str(exc))
            return
        self.settings.setValue("repo", str(repo))
        self.settings.setValue("python", str(python))
        self.target.setCurrentIndex(0)
        self.host.setText("127.0.0.1")
        self.sim_log_tail = ""
        self.sim_decoder.reset()
        self.sim_console.clear()
        self.sim_ready = False
        environment = QProcessEnvironment.systemEnvironment()
        for name in ('PYTHONHOME', 'QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH'):
            environment.remove(name)
        if getattr(sys, 'frozen', False):
            bundled = str(Path(sys._MEIPASS)).lower()
            environment.insert('PATH', os.pathsep.join(p for p in environment.value('PATH').split(os.pathsep)
                                                      if not p.lower().startswith(bundled)))
        environment.insert("PYTHONUTF8", "1")
        environment.insert("PYTHONPATH", str(repo/"tools/servo_tool"))
        # Conda native DLLs (including OpenGL dependencies) must be visible.
        environment.insert("PATH", str(python.parent/"Library/bin") + os.pathsep + str(python.parent) + os.pathsep + environment.value("PATH"))
        self.process.setProcessEnvironment(environment)
        self.process.setWorkingDirectory(str(repo))
        self.sim_heartbeat = Path(tempfile.gettempdir()) / ('spot-desktop-' + uuid.uuid4().hex + '.heartbeat')
        self.sim_heartbeat.touch()
        arguments = ["-X", "utf8", "-u", str(script), "--no-ble", "--desktop-heartbeat", str(self.sim_heartbeat),
            "--viewer" if self.viewer.isChecked() else "--headless", "--host", "127.0.0.1", "--port", str(self.port.value()),
            "--video-host", "127.0.0.1", "--video-port", str(self.video_port.value()),
            "--parameters", str(repo/"simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json")]
        if os.name == 'nt' and getattr(sys, 'frozen', False):
            # The PyInstaller DLL directory is inherited by child processes.
            # Restore the system loader while launching the external Conda runtime.
            import ctypes
            set_directory = ctypes.windll.kernel32.SetDllDirectoryW
            set_directory.argtypes = [ctypes.c_wchar_p]
            set_directory(None)
            try:
                self.process.start(str(python), arguments)
                self.process.waitForStarted(3000)
            finally:
                set_directory(sys._MEIPASS)
        else:
            self.process.start(str(python), arguments)
        self.sim_button.setText("■  MuJoCo 종료")
        self.sim_status.setText("시작 중 · 첫 C 빌드에는 시간이 걸릴 수 있습니다.")
        for field in (self.repo, self.python, self.viewer, self.video_port, self.port, self.target, self.host):
            field.setEnabled(False)

    def sim_output(self):
        text = self.sim_decoder.decode(bytes(self.process.readAllStandardOutput()))
        self.sim_console.moveCursor(QTextCursor.MoveOperation.End)
        self.sim_console.insertPlainText(text)
        self.sim_log_tail = (self.sim_log_tail + text)[-10000:]
        if not self.sim_ready and "Virtual robot:" in self.sim_log_tail:
            self.sim_ready = True
            self.sim_status.setText("실행 중 · 로컬 물리 시뮬레이션")
            self.connection.connect_to("tcp", "127.0.0.1", self.port.value())

    def stop_sim(self):
        self.stop_sim_pending = False
        if self.process.state() != QProcess.ProcessState.NotRunning:
            if self.sim_heartbeat:
                self.sim_heartbeat.unlink(missing_ok=True)
                self.sim_heartbeat = None
            self.sim_status.setText("시뮬레이터 종료 중…")
            pid = self.process.processId()
            QTimer.singleShot(7000, lambda: self.kill_sim_if_needed(pid))
        elif self.exiting:
            self.close()

    def kill_sim_if_needed(self, pid):
        if self.process.state() != QProcess.ProcessState.NotRunning and self.process.processId() == pid:
            self.process.kill()

    def sim_finished(self, code, status):
        if self.sim_heartbeat:
            self.sim_heartbeat.unlink(missing_ok=True)
            self.sim_heartbeat = None
        self.sim_console.appendPlainText(f'\n[MuJoCo process exited: {code}, {status.name}]')
        self.sim_ready = False
        self.sim_button.setText("▶  MuJoCo 시작 + 연결")
        self.sim_status.setText("시뮬레이터가 종료되었습니다." if code == 0 else "실행 실패 · SIMULATOR LOG를 확인하십시오.")
        for field in (self.repo, self.python, self.viewer, self.video_port):
            field.setEnabled(True)
        self.on_state(self.snapshot)
        if self.exiting and not self.connection.running:
            QTimer.singleShot(100, self.close)

    def on_disconnected(self):
        self.release_input()
        if self.stop_sim_pending or self.exiting:
            QTimer.singleShot(50, self.stop_sim)

    def video_tick(self):
        if self.exiting:
            return
        active = self.sim_ready or (self.snapshot.get("connected") and self.target.currentData() == "tcp")
        if not active:
            self.last_frame = None
            self.video.setPixmap(QPixmap())
            self.video.setText("MuJoCo를 시작하면 여기에 표시됩니다.\n\n실제 로봇의 카메라 영상이 아닙니다.")
            return
        if time.monotonic() - self.last_frame_at > 2:
            self.video_status.setText("영상 수신 대기 / 지연 · 조종 연결과 별개")
            self.video.setPixmap(QPixmap())
            self.video.setText('새 MuJoCo 프레임을 기다리는 중…')
        if self.video_reply:
            return
        url = QUrl()
        url.setScheme("http")
        url.setHost(self.host.text().strip())
        url.setPort(self.video_port.value())
        url.setPath("/frame.jpg")
        request = QNetworkRequest(url)
        request.setTransferTimeout(1500)
        reply = self.network.get(request)
        self.video_reply = reply
        reply.downloadProgress.connect(lambda received, total: reply.abort() if received > 2_000_000 else None)
        reply.finished.connect(self.video_finished)

    def video_finished(self):
        reply, self.video_reply = self.video_reply, None
        if reply is None:
            return
        if bytes(reply.rawHeader("X-SpotOMG-Video")) == b"1" and reply.attribute(QNetworkRequest.Attribute.HttpStatusCodeAttribute) == 200:
            data = bytes(reply.readAll())
            pixmap = QPixmap()
            if len(data) < 2_000_000 and pixmap.loadFromData(data, "JPEG"):
                self.last_frame = pixmap
                self.last_frame_at = time.monotonic()
                self.video.setPixmap(pixmap.scaled(self.video.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
                self.video_status.setText("LIVE · MuJoCo · 추정 물성 / 실기 영상 아님")
        reply.deleteLater()

    def save_log(self):
        self.release_input()
        path, _ = QFileDialog.getSaveFileName(self, "콘솔 로그 저장", "spot-omg-console.txt", "Text (*.txt)")
        if path:
            try:
                Path(path).write_text(self.console.toPlainText(), encoding="utf-8")
            except OSError as exc:
                self.show_error(str(exc))

    def closeEvent(self, event):
        self.exiting = True
        self.release_input()
        if self.connection.running:
            self.connection.disconnect()
            event.ignore()
            self.connection_badge.setText("정지 및 연결 종료 중…")
            return
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.stop_sim()
            event.ignore()
            return
        self.pulse_timer.stop()
        self.video_timer.stop()
        if self.video_reply:
            self.video_reply.abort()
        event.accept()


def configure_app(app):
    # Qt's offscreen Windows font database needs explicit fonts for render tests.
    if os.name == 'nt':
        for name in ('malgun.ttf', 'consola.ttf'):
            path = Path(os.environ.get('WINDIR', r'C:\Windows'))/'Fonts'/name
            if path.is_file():
                QFontDatabase.addApplicationFont(str(path))
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Spot OMG Windows controller')
    parser.add_argument('--smoke-test', type=Path, help='Render startup to PNG, then close (no robot connection)')
    parser.add_argument('--smoke-mujoco', action='store_true', help='With --smoke-test, launch local MuJoCo and verify a video frame before closing')
    args = parser.parse_args()
    app = QApplication(sys.argv)
    configure_app(app)
    window = Window()
    if args.smoke_test:
        window.resize(1240, 980)
    window.show()
    failed = []
    if args.smoke_test:
        args.smoke_test.parent.mkdir(parents=True, exist_ok=True)
        started = time.monotonic()
        timer = QTimer(window)
        def verify():
            ready = not args.smoke_mujoco or (window.sim_ready and window.snapshot.get('synced') and window.last_frame is not None)
            if ready or time.monotonic() - started > 60:
                timer.stop()
                if not ready:
                    failed.append(True)
                window.grab().save(str(args.smoke_test))
                args.smoke_test.with_suffix('.log').write_text(window.console.toPlainText()+'\n'+window.sim_console.toPlainText(),encoding='utf-8')
                window.close()
        timer.timeout.connect(verify)
        timer.start(250)
        if args.smoke_mujoco:
            window.target.setCurrentIndex(0)
            window.port.setValue(18885)
            window.video_port.setValue(18886)
            window.viewer.setChecked(False)
            QTimer.singleShot(100, window.toggle_sim)
    result = app.exec()
    return 1 if failed else result
