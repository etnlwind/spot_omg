"""Rebuild editable SVG schematics and PNG previews (Pillow, macOS Korean font)."""
from pathlib import Path
from html import escape
from PIL import Image, ImageDraw, ImageFont
OUT=Path(__file__).parent
FONT='/System/Library/Fonts/AppleSDGothicNeo.ttc'
INK='#152b43'; RED='#c84335'; BLUE='#247ba0'; GREEN='#16826b'; GRAY='#526577'
class Sheet:
 def __init__(self,title,sub,h=1280):
  self.w=1600; self.h=h; self.im=Image.new('RGB',(self.w,h),'#f5f7fa'); self.d=ImageDraw.Draw(self.im)
  self.svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="{h}" viewBox="0 0 1600 {h}"><rect width="1600" height="{h}" fill="#f5f7fa"/>']
  self.text(45,28,title,34); self.text(45,80,sub,20,GRAY)
 def text(self,x,y,s,size=22,c=INK):
  s=s.replace('−','-')
  self.d.text((x,y),s,font=ImageFont.truetype(FONT,size),fill=c)
  self.svg.append(f'<text x="{x}" y="{y+size}" font-family="Apple SD Gothic Neo, sans-serif" font-size="{size}" fill="{c}">{escape(s)}</text>')
 def rect(self,x,y,w,h,fill='white',stroke='#c7d2de'):
  self.d.rectangle((x,y,x+w,y+h),fill=fill,outline=stroke,width=2)
  self.svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
 def line(self,pts,c=INK,width=3):
  self.d.line(pts,fill=c,width=width)
  p=' '.join(f'{x},{y}' for x,y in pts); self.svg.append(f'<polyline points="{p}" fill="none" stroke="{c}" stroke-width="{width}"/>')
 def dot(self,x,y,c=RED):
  self.d.ellipse((x-5,y-5,x+5,y+5),fill=c); self.svg.append(f'<circle cx="{x}" cy="{y}" r="5" fill="{c}"/>')
 def box(self,x,y,w,h,title,lines=(),color=INK):
  self.rect(x,y,w,h); self.text(x+18,y+12,title,24,color)
  for i,t in enumerate(lines):self.text(x+18,y+52+i*29,t,20,GRAY)
 def save(self,name):
  self.im.save(OUT/(name+'.png')); (OUT/(name+'.svg')).write_text('\n'.join(self.svg+['</svg>']))

s=Sheet('Spot OMG · 전원 결선 회로도', '설계안 R1 · 2026-09-12 | DALY 공통 포트 B− / P−형 기준 · 최종 정격 / 실물 단자 확인 필요',1400)
s.box(45,130,300,175,'BAT1 · DXF 3S LiPo',['11.1V / 5250mAh','완충 12.6V · 주전원 + / −','밸런스 커넥터 4선'])
s.line([(345,180),(405,180)],RED,5)
s.box(405,140,180,90,'F1 주 퓨즈',['배터리 가까이'])
s.line([(585,180),(675,180),(765,180)],RED,5);s.dot(675,180)
s.text(595,115,'FUSED+',21,RED)
s.rect(765,130,335,210);s.text(783,140,'K1 · JD1914 12V / 40A',24)
s.line([(765,180),(820,180),(930,260)],RED,4);s.dot(820,180)
s.line([(930,180),(1100,180),(1150,180)],RED,5);s.dot(930,180)
s.line([(930,260),(1030,260)],GRAY);s.dot(930,260,GRAY)
s.text(787,194,'30 COM',20);s.text(950,194,'87 NO',20);s.text(890,278,'87a NC · 미사용 / 절연',19)
s.text(793,310,'도식은 코일 OFF 상태',17,GRAY)
s.box(1150,130,400,210,'PDB-HEX · IN+ / IN−',['IN+ ← 87 : ROBOT+','IN− ← GND_P (BMS P−)','주 출력 = 배터리 전압 그대로','Vx = 5V : 제어보드 전원'])
s.line([(345,270),(395,270),(395,420),(440,420)],BLUE,5)
s.text(360,315,'BAT−',20,BLUE)
s.box(440,370,450,140,'DALY · 3S 12V 60A',['B− : 배터리 주 음극만 연결','P− : 부하 / 충전기 공통 음극'])
s.line([(890,450),(1500,450)],INK,5);s.text(1080,413,'GND_P = 보호된 공통 GND',22)
s.line([(1450,340),(1450,450)],INK,4);s.dot(1450,450,INK)
s.line([(675,180),(675,355),(1000,355)],RED,3);s.text(1010,341,'→ FUSED+ 분기 (아래 두 회로)',19,RED)
s.text(45,535,'아래의 같은 전원 이름은 전기적으로 연결됩니다. BAT−와 GND_P를 외부 배선으로 연결하지 마십시오.',21,BLUE)
# Charge circuit net-labeled
s.box(45,590,730,225,'충전 분기 · K1 접점 앞에서 분기',['FUSED+ ─ F2 충전 분기 퓨즈 ─ DC 잭 +','GND_P ───────────────── DC 잭 −','외장 12.6V / 2A 3S LiPo CC/CV 충전기 → DC 잭','5.5×2.1mm · 중심 극성 확인 후 배선','충전 중 S1 OFF · 2A 충전기로 로봇 운전하지 않음'])
# Coil schematic actual symbols
s.rect(805,590,745,225);s.text(823,602,'K1 코일 제어 · 작은 스위치에는 코일 전류만',24)
s.text(825,649,'FUSED+',19,RED);s.line([(920,665),(950,665)],RED)
s.rect(950,650,45,28);s.text(954,681,'F3',18)
s.line([(995,665),(1020,665),(1050,646)],RED);s.dot(1020,665);s.line([(1060,665),(1140,665)],RED)
s.text(1010,699,'S1 로커',19);s.text(1080,631,'86 (+)',18)
s.rect(1140,642,75,46);s.text(1154,650,'코일',20)
s.line([(1215,665),(1435,665)],INK);s.text(1230,631,'85 (−)',18);s.text(1440,650,'GND_P',19)
s.line([(1125,665),(1125,738),(1190,738)],RED);s.line([(1290,738),(1330,738),(1330,665)],INK)
s.rect(1190,720,100,35);s.text(1204,724,'D1',20);s.line([(1200,720),(1200,755)],RED)
s.text(823,775,'D1: 캐소드(띠) → 86, 애노드 → 85 · 내장 여부 확인',19,GRAY)
# taps mapping
s.box(45,850,730,230,'셀 감지선 · 주전류가 흐르는 선이 아닙니다',['B0 → 팩 − / 첫 셀 −','B1 → 셀1 + / 셀2 − 연결점','B2 → 셀2 + / 셀3 − 연결점','B3 → 팩 + / 세 번째 셀 +','팩 밸런스 커넥터를 이용 · 선 색 대신 전압 순서 확인'])
s.box(805,850,745,230,'PDB 출력 · 서보 / 로직을 분리하여 배선',['ROBOT+ / GND_P → 다리별 퓨즈 → 3개 서보 × 4다리','직결은 12.6V 허용 서보만 · 저전압형은 별도 강압 필요','Vx 5V / G → 로직 퓨즈 → STM32 / ESP32','URT-2 전원은 보드 정격에 맞게 · 서보 전류 통과 금지','Vbat / Curr 감지선은 ADC 정격·분압 확인 전 연결 보류'])
s.box(45,1115,1505,210,'결선 조건과 추가 부품',['• F1/F2/F3/다리별/로직 퓨즈 및 홀더, DC 충전 잭, 절연된 전원 분배 하네스가 추가로 필요합니다.','• 60A BMS가 40A 릴레이·가는 서보 케이블을 보호한다고 가정하지 않습니다. 각 분기에 맞춰 퓨즈를 선정합니다.','• D1은 외장 코일 역기전력 보호용입니다. 내장 다이오드가 있으면 그 극성을 우선하고 중복 부품은 생략합니다.','• OFF는 로봇 부하 차단입니다. BMS 자체 대기 전류는 남습니다. 장기 보관은 주전원과 셀 감지선 모두 분리.','• 실제 핀 번호 / 배터리 화학계 / 충전 방식 확인 전 제작 확정 도면으로 사용하지 마십시오.'])
s.save('spot-omg-power-schematic')

s=Sheet('Spot OMG · 전체 전원 및 제어 구성', '설계안 R1 | 빨강: 전원 흐름 · 파랑: 통신 · 모든 로봇 GND는 DALY P−에 연결',1280)
s.box(45,145,380,140,'DXF 3S + DALY 60A',['배터리 11.1V / 최대 12.6V','BMS가 셀 감지와 충·방전 보호'])
s.box(45,370,380,140,'주 퓨즈 → K1 릴레이',['S1 로커 → 12V 코일 ON/OFF','30 → 87 · 87a 절연'])
s.line([(230,285),(230,370)],RED,5)
s.box(45,595,380,150,'Matek PDB-HEX',['원전압 → 서보 전력 분배','5V BEC → 로직 전력 분배','출력단에서 다리별 병렬 분기'])
s.line([(230,510),(230,595)],RED,5)
s.box(45,825,380,140,'외장 충전기 12.6V / 2A',['K1 앞의 충전 분기에 연결','로봇 OFF 상태에서 충전'])
s.text(45,990,'OFF: 로직·서보 꺼짐 / BMS는 남음',20)
s.text(45,1025,'완전 분리: 주전원 + 셀 감지선 4개',20)
s.box(1140,145,405,260,'장기 보관 · 외부 소비 0',['S1 OFF / 충전기·USB 분리','배터리 주전원 2선 분리','BMS 셀 감지선 4개 분리','실제 BMS 분리 순서 준수','배터리 자체 자연 방전은 남음'])
s.box(580,145,400,130,'iPhone / Mac 앱',['BLE → 실물 ESP32','시뮬레이터 TCP 경로와 구분'])
s.box(580,350,400,145,'ESP32-WROOM 브리지',['5V 입력은 개발보드 VIN 기준','TX17 → STM32 PC11','RX16 ← STM32 PC10'])
s.line([(780,275),(780,350)],BLUE,4);s.text(800,297,'BLE',20,BLUE)
s.box(580,575,400,155,'STM32F446RE / NUCLEO',['USART3 : 브리지 115200bps','USART1 : 서보 버스 1Mbps','외부 5V 입력 / USB 점퍼 확인'])
s.line([(780,495),(780,575)],BLUE,4);s.text(800,517,'3.3V UART',20,BLUE)
s.line([(425,640),(515,640),(515,425),(580,425)],RED,4);s.dot(515,640)
s.line([(515,640),(580,640)],RED,4);s.text(440,598,'5V',20,RED)
s.box(1140,575,405,155,'IMU 보드',['BNO055 / BNO086 구성에 맞춤','I²C / SPI · 3.3V 신호','전원은 실제 보드 정격 확인'])
s.line([(980,650),(1140,650)],BLUE,4)
s.box(580,825,400,130,'URT-2 · TTL 버스 변환',['STM32 USART1 ↔ 반이중 DATA','전원 / 로직 레벨은 보드별 확인'])
s.line([(780,730),(780,825)],BLUE,4)
s.box(1140,825,405,260,'서보 12개 · 다리별 전력 분기',['FL : 1 / 2 / 3','FR : 4 / 5 / 6','RL : 7 / 8 / 9','RR : 10 / 11 / 12','J1·J3 STS3215 ×8','J2 STS3250 ×4'])
s.line([(980,890),(1140,890)],BLUE,4);s.text(998,849,'DATA',20,BLUE)
s.line([(425,705),(470,705),(470,1130),(1340,1130),(1340,1085)],RED,5)
s.text(580,1092,'배터리 전압 전력선 · 다리별 퓨즈 / 분배 하네스 · MCU와 URT-2 경유 금지',20,RED)
s.box(45,1180,1500,65,'앱 Stop은 소프트웨어 정지 · S1 OFF는 전원 차단으로 서보의 자세 유지 토크도 사라집니다.',[],INK)
s.save('spot-omg-system-overview')
