"""Top-view per-foot swing-clearance editor; no joint-angle controls."""
import time
from PySide6.QtCore import Qt, Signal, QSize
from PySide6.QtGui import QColor, QPainter, QPixmap
from PySide6.QtWidgets import QWidget,QVBoxLayout,QGridLayout,QLabel,QSpinBox,QPushButton,QHBoxLayout,QSizePolicy
from .assets import resource_path

LEGS=('FL','FR','RL','RR')

class RobotTop(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(150,310)
        self.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.setAccessibleName('Spot OMG 로봇 위에서 본 모습 · 위쪽이 앞')
        self.pixmap=QPixmap(str(resource_path('robot-top.png')))
    def sizeHint(self):
        return QSize(165,350)
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QColor('#e4e9f0'))
        p.drawText(0,0,self.width(),26,Qt.AlignCenter,'↑ 로봇 앞쪽')
        if self.pixmap.isNull():
            p.drawText(self.rect().adjusted(0,28,0,0),Qt.AlignCenter,'로봇 이미지 없음')
            return
        area=self.rect().adjusted(4,30,-4,-4)
        scaled=self.pixmap.scaled(area.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation)
        p.drawPixmap(area.x()+(area.width()-scaled.width())//2,
                     area.y()+(area.height()-scaled.height())//2,scaled)

class FootLiftEditor(QWidget):
    applyRequested=Signal(str)
    def __init__(self,settings):
        super().__init__();self.settings=settings;self.pending=None;self.sent_at=None;self.pending_started=None
        self.setObjectName('footLiftEditor')
        self.setAttribute(Qt.WA_StyledBackground,True)
        self.width_supported=False;self.width_inputs=[]
        self.last_state_at=None;self.actual=None;self.baseline=None;self.ready=False
        self.setStyleSheet('''
            QWidget#footLiftEditor { background: #171c23; }
            QWidget { color: #eff2f6; }
            QLabel { color: #dce2eb; font-size: 13px; }
            QSpinBox { background: #20252d; color: #f5f7fa; border: 1px solid #424a56;
                border-radius: 7px; font-size: 16px; padding: 9px 7px; }
            QSpinBox:disabled { color: #9ca6b5; background: #191e25; }
            QLabel#liftStatus { background: #242d39; color: #e5ebf4; padding: 12px; border-radius: 8px; }
            QPushButton { background: #272e38; color: #f0f3f8; border: 1px solid #424a56;
                border-radius: 7px; padding: 9px; }
            QPushButton:hover { background: #343e4c; }
            QPushButton:disabled { background: #1e242c; color: #8893a3; border-color: #343c48; }
            QPushButton#primary { background: #f38a21; color: #15191f; border-color: #f38a21; font-weight: 700; }
            QPushButton#primary:disabled { background: #574530; color: #b2a392; border-color: #65533f; }
        ''')
        layout=QVBoxLayout(self)
        layout.setContentsMargins(16,14,16,14);layout.setSpacing(12)
        layout.addWidget(QLabel('발 위치 보정 · 추가 들림 / 좌우 간격 (mm)'))
        grid=QGridLayout();grid.setHorizontalSpacing(12);self.inputs=[]
        grid.setColumnStretch(0,1);grid.setColumnStretch(1,1);grid.setColumnStretch(2,1)
        for i,leg in enumerate(LEGS):
            column=QVBoxLayout()
            if i>=2:column.addStretch()
            column.addWidget(QLabel(f'{leg} · '+('앞' if i<2 else '뒤')+(' 왼쪽' if i%2==0 else ' 오른쪽')))
            field=QSpinBox();field.setRange(0,2147483647);field.setSuffix(' mm');field.setAccessibleName(leg+' 추가 들림')
            field.setMinimumWidth(105);field.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed)
            column.addWidget(QLabel('추가 들림'))
            column.addWidget(field)
            width=QSpinBox();width.setRange(-2147483647,2147483647);width.setSuffix(' mm')
            width.setMinimumWidth(105);width.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed)
            width.setAccessibleName(leg+' 좌우 간격 보정');width.setToolTip('모델 기본값에서 보정: - 안쪽 / + 바깥쪽')
            column.addWidget(QLabel('좌우 간격 (- / +)'))
            column.addWidget(width);self.width_inputs.append(width)
            if i<2:column.addStretch()
            grid.addLayout(column,i//2,0 if i%2==0 else 2);self.inputs.append(field)
        grid.addWidget(RobotTop(),0,1,2,1);layout.addLayout(grid)
        self.width_notice=QLabel('간격 보정은 지원 펌웨어 업데이트 후 사용할 수 있습니다.')
        self.width_notice.setWordWrap(True);layout.addWidget(self.width_notice)
        self.status=QLabel('연결 후 적용값 확인');self.status.setObjectName('liftStatus');self.status.setWordWrap(True);layout.addWidget(self.status)
        hint=QLabel('입력값을 바꾼 뒤 저장·반영을 누르세요.\nSTM32에 영구 저장되어 Windows·Mac·iPhone이 공유합니다.\n0 = 모델 기본값 / 높이: 스윙 중 추가 들림\n간격: - 안쪽, + 바깥쪽 / 앞뒤 위치와 높이를 유지하도록 계산')
        hint.setWordWrap(True);layout.addWidget(hint)
        row=QHBoxLayout();self.apply=QPushButton('저장·반영');self.load=QPushButton('로봇 값 불러오기');self.zero=QPushButton('모두 0')
        self.apply.setObjectName('primary')
        self.apply.clicked.connect(self.submit);self.load.clicked.connect(self.restore);self.zero.clicked.connect(lambda:[v.setValue(0) for v in self.fields])
        for button in (self.apply,self.load,self.zero):row.addWidget(button)
        layout.addLayout(row);self.restore();self.apply.setEnabled(False)
        for field in self.inputs+self.width_inputs:field.valueChanged.connect(self.mark_difference)
    @property
    def fields(self):
        return self.inputs+self.width_inputs if self.width_supported else self.inputs
    @staticmethod
    def summary(values):
        text='높이: '+' / '.join(f'{leg} {v}mm' for leg,v in zip(LEGS,values[:4]))
        if len(values)==8:text+='\n간격: '+' / '.join(f'{leg} {v:+d}mm' for leg,v in zip(LEGS,values[4:]))
        return text
    def mark_difference(self):
        if self.actual is not None and self.pending is None:
            values=[v.value() for v in self.fields]
            text='로봇 적용값\n'+self.summary(self.actual)
            self.status.setText(('아직 반영되지 않음 · 저장·반영을 누르세요\n' if values!=self.actual else '입력값과 로봇 적용값 일치\n')+text)
    def restore(self):
        values=self.actual or [0]*len(self.fields)
        self.baseline=self.actual
        for field,value in zip(self.fields,values):field.setValue(value)
        if self.ready and self.pending is None:self.applyRequested.emit('footlift show')
    def prepare_open(self):
        # Reopening discards edits that were never submitted.
        values=self.actual if self.actual is not None else [0]*len(self.fields)
        for field,value in zip(self.fields,values):field.setValue(value)
        if self.actual is None:
            self.status.setText('연결 후 로봇 저장값 확인')
        elif self.pending is not None:
            self.status.setText('반영 확인 중… 현재 로봇 적용값을 표시합니다.')
        else:self.mark_difference()
        if self.ready and self.pending is None:self.applyRequested.emit('footlift show')
    def submit(self):
        if not self.ready or self.pending is not None:return
        self.pending_started=time.monotonic()
        self.pending=[v.value() for v in self.fields];self.sent_at=self.last_state_at
        self.status.setText('적용값 확인 중…');self.apply.setEnabled(False)
        self.applyRequested.emit('footlift save '+' '.join(map(str,self.pending)))
    def update_state(self,snapshot):
        robot=snapshot.get('state',{});ready=snapshot.get('connected') and snapshot.get('synced')
        capable='footliftpersist' in snapshot.get('caps',[])
        self.width_supported='footwidth' in snapshot.get('caps',[])
        self.width_notice.setVisible(not self.width_supported)
        idle=ready and snapshot.get('phase')=='idle'
        self.ready=bool(ready and capable and snapshot.get('phase')=='idle')
        self.last_state_at=snapshot.get('state_at')
        try:actual=[int(robot['lift_'+leg.lower()]) for leg in LEGS]
        except (KeyError,ValueError):actual=None
        if actual is not None and any(not 0<=v<=2147483647 for v in actual):actual=None
        if self.width_supported and actual is not None:
            try:widths=[int(robot['width_'+leg.lower()]) for leg in LEGS]
            except (KeyError,ValueError):widths=None
            actual=actual+widths if widths is not None and all(-2147483647<=v<=2147483647 for v in widths) else None
        self.actual=actual if ready else None
        timed_out=self.pending is not None and time.monotonic()-self.pending_started>10
        if timed_out:self.pending=None
        if not ready:
            self.baseline=None
            self.pending=None;self.status.setText('연결 후 적용값 확인')
        elif not capable:self.status.setText('발 들림 보정 지원 펌웨어가 필요합니다 (V90-R2).')
        elif actual is not None:
            text='로봇 적용값\n'+self.summary(actual)
            result=snapshot.get('foot_lift_result')
            if self.pending is not None and idle and result is not None and result.get('values')==self.pending:
                if result.get('ok') and actual==self.pending and self.last_state_at!=self.sent_at:
                    text+=' · 로봇 저장 확인 완료'
                    for field,value in zip(self.fields,actual):field.setValue(value)
                else:text+=' · 저장 확인 실패, 로봇 값을 다시 확인하세요'
                self.pending=None
            self.status.setText(text)
            if self.pending is not None:
                self.status.setText('반영 확인 중…\n'+text)
            elif [v.value() for v in self.fields]!=actual:
                self.status.setText('아직 반영되지 않음 · 저장·반영을 누르세요\n'+text)
            if self.pending is None and (self.baseline is None or [v.value() for v in self.fields]==self.baseline):
                for field,value in zip(self.fields,actual):field.setValue(value)
            self.baseline=actual
        if timed_out:self.status.setText('저장 확인 시간 초과 · 로봇 값을 다시 불러오세요')
        self.apply.setEnabled(bool(idle and capable and actual is not None and self.pending is None))
        self.load.setEnabled(self.pending is None)
        self.zero.setEnabled(self.pending is None)
        for field in self.fields:field.setEnabled(self.pending is None)

        for field in self.width_inputs:
            field.setEnabled(self.width_supported and self.pending is None)
            if not self.width_supported:field.setValue(0)
