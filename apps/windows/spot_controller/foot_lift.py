"""Top-view per-foot swing-clearance editor; no joint-angle controls."""
import json
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget,QVBoxLayout,QGridLayout,QLabel,QSpinBox,QPushButton,QHBoxLayout

LEGS=('FL','FR','RL','RR')

class RobotTop(QWidget):
    def __init__(self):
        super().__init__();self.setMinimumSize(130,200)
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        w,h=self.width(),self.height();cx=w/2
        p.setPen(QPen(QColor('#71897e'),8))
        for side in (-1,1):
            for y in (h*.27,h*.73):
                p.drawLine(int(cx+side*25),int(y),int(cx+side*55),int(y))
                p.setBrush(QColor('#226c4b'))
                p.drawEllipse(int(cx+side*55-5),int(y-5),10,10)
        p.setBrush(QColor('#dce9e0'));p.setPen(QPen(QColor('#226c4b'),2))
        p.drawRoundedRect(int(cx-32),32,64,int(h-64),14,14)
        p.drawText(self.rect(),Qt.AlignCenter,'↑ 앞\n\nSpot\nOMG\n\n뒤')

class FootLiftEditor(QWidget):
    applyRequested=Signal(str)
    def __init__(self,settings):
        super().__init__();self.settings=settings;self.pending=None;self.sent_at=None
        self.auto_checked=False;self.last_state_at=None;self.actual=None
        self.setStyleSheet('QLabel { color: #243e33; font-size: 13px; } QSpinBox { background: white; color: #162e26; font-size: 18px; padding: 10px; } QLabel#liftStatus { background: #e1eee5; color: #173e2c; padding: 12px; border-radius: 8px; }')
        layout=QVBoxLayout(self)
        layout.addWidget(QLabel('발 추가 들림 · 설정할 값 (mm)'))
        grid=QGridLayout();self.inputs=[]
        for i,leg in enumerate(LEGS):
            column=QVBoxLayout();column.addWidget(QLabel(f'{leg} · '+('앞' if i<2 else '뒤')+(' 왼쪽' if i%2==0 else ' 오른쪽')))
            field=QSpinBox();field.setRange(0,2147483647);field.setSuffix(' mm');field.setAccessibleName(leg+' 추가 들림')
            column.addWidget(field);grid.addLayout(column,i//2,0 if i%2==0 else 2);self.inputs.append(field)
        grid.addWidget(RobotTop(),0,1,2,1);layout.addLayout(grid)
        self.status=QLabel('연결 후 적용값 확인');self.status.setObjectName('liftStatus');self.status.setWordWrap(True);layout.addWidget(self.status)
        hint=QLabel('입력값을 바꾼 뒤 저장·반영을 누르세요.\n저장된 값은 연결 후 정지 상태에서 자동 반영됩니다.\n0 = 기본 보행 / 스윙 중 추가 들림 / 로봇 기준 좌우')
        hint.setWordWrap(True);layout.addWidget(hint)
        row=QHBoxLayout();self.apply=QPushButton('저장·반영');self.load=QPushButton('저장값 불러오기');self.zero=QPushButton('모두 0')
        self.apply.clicked.connect(self.submit);self.load.clicked.connect(self.restore);self.zero.clicked.connect(lambda:[v.setValue(0) for v in self.inputs])
        for button in (self.apply,self.load,self.zero):row.addWidget(button)
        layout.addLayout(row);self.restore();self.apply.setEnabled(False)
        for field in self.inputs:field.valueChanged.connect(self.mark_difference)
    def mark_difference(self):
        if self.actual is not None and self.pending is None:
            values=[v.value() for v in self.inputs]
            text='로봇 적용값\n'+' / '.join(f'{leg} {v}mm' for leg,v in zip(LEGS,self.actual))
            self.status.setText(('아직 반영되지 않음 · 저장·반영을 누르세요\n' if values!=self.actual else '입력값과 로봇 적용값 일치\n')+text)
    def saved_values(self):
        try:
            values=json.loads(self.settings.value('footLiftMm','null'))
            if len(values)!=4 or any(type(v)!=int or not 0<=v<=2147483647 for v in values):raise ValueError()
        except (ValueError,TypeError):return None
        return values
    def restore(self):
        values=self.saved_values() or [0]*4
        for field,value in zip(self.inputs,values):field.setValue(value)
    def prepare_open(self):
        # Reopening discards edits that were never submitted.
        values=self.actual if self.actual is not None else (self.saved_values() or [0]*4)
        for field,value in zip(self.inputs,values):field.setValue(value)
        if self.actual is None:
            self.status.setText('저장값 표시 중 · 연결 후 로봇 적용값 확인')
        elif self.pending is not None:
            self.status.setText('반영 확인 중… 현재 로봇 적용값을 표시합니다.')
        else:self.mark_difference()
    def submit(self):
        self.auto_checked=True
        self.pending=[v.value() for v in self.inputs];self.sent_at=self.last_state_at
        self.status.setText('적용값 확인 중…');self.apply.setEnabled(False)
        self.applyRequested.emit('footlift set '+' '.join(map(str,self.pending)))
    def update_state(self,snapshot):
        robot=snapshot.get('state',{});ready=snapshot.get('connected') and snapshot.get('synced')
        capable='footlift' in snapshot.get('caps',[])
        idle=ready and snapshot.get('phase')=='idle'
        self.last_state_at=snapshot.get('state_at')
        try:actual=[int(robot['lift_'+leg.lower()]) for leg in LEGS]
        except (KeyError,ValueError):actual=None
        self.actual=actual if ready else None
        if not ready:
            if not snapshot.get('connected'):self.auto_checked=False
            self.pending=None;self.status.setText('연결 후 적용값 확인')
        elif not capable:self.status.setText('발 들림 보정 지원 펌웨어가 필요합니다 (V90-R1).')
        elif actual is not None:
            text='로봇 적용값\n'+' / '.join(f'{leg} {v}mm' for leg,v in zip(LEGS,actual))
            if self.pending is not None and idle and self.last_state_at!=self.sent_at:
                if actual==self.pending:
                    self.settings.setValue('footLiftMm',json.dumps(actual));self.settings.sync();text+=' · 저장 완료'
                    for field,value in zip(self.inputs,actual):field.setValue(value)
                else:text+=' · 반영 불일치, 저장하지 않음'
                self.pending=None
            self.status.setText(text)
            if self.pending is not None:
                self.status.setText('반영 확인 중…\n'+text)
            elif [v.value() for v in self.inputs]!=actual:
                self.status.setText('아직 반영되지 않음 · 저장·반영을 누르세요\n'+text)
            if idle and not self.auto_checked and self.pending is None:
                self.auto_checked=True
                saved=self.saved_values()
                if saved is not None and saved!=actual:
                    self.pending=saved;self.sent_at=self.last_state_at
                    self.status.setText('저장된 값 자동 반영 중…\n'+text)
                    self.applyRequested.emit('footlift set '+' '.join(map(str,saved)))
        self.apply.setEnabled(bool(idle and capable and self.pending is None))
        for field in self.inputs:field.setEnabled(self.pending is None)
