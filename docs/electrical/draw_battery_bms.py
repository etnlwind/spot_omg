from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
p=Path(__file__).parent
im=Image.new('RGB',(1500,970),'#f5f7fa');d=ImageDraw.Draw(im)
font='/System/Library/Fonts/AppleSDGothicNeo.ttc'
def t(x,y,s,n=25,c='#192d43'):d.text((x,y),s,font=ImageFont.truetype(font,n),fill=c)
def line(points,c,w=5):d.line(points,fill=c,width=w)
def box(x,y,w,h,c):d.rounded_rectangle((x,y,x+w,y+h),radius=20,fill=c)
red='#c63735';blue='#147ca5';black='#29333e'
t(45,28,'3S 배터리 ↔ DALY BMS 연결',42)
t(45,92,'B- / P- 공통 포트형 기준 · 실제 커넥터의 좌우 순서가 아닌 전기적 연결 관계입니다.',24)
box(50,190,355,475,'#d7e9fa');t(80,216,'3S LiPo 배터리',33);t(80,266,'11.1V · 완충 12.6V',25)
box(1010,340,420,360,'#f4d5d2');t(1040,360,'DALY 3S 60A',33)
t(1040,402,'리튬이온 4.2V/셀형 기준',23)
# main positive travels outside BMS
line([(405,330),(710,330),(710,207),(1335,207)],red,9)
t(430,280,'굵은 +',27,red);t(760,155,'주 퓨즈 → 로봇 / 충전기 +',27,red)
t(755,240,'BMS에 굵은 + 단자가 없는 형식',22)
# main negative
line([(405,385),(920,385),(920,462),(1010,462)],blue,9)
t(430,344,'굵은 - → BMS B-',27,blue);t(1040,448,'B-  주전원 음극',27,blue)
# sense bundle logical terminals four separate lines
colors=[black,'#956815','#8854ab',red]
rows=[500,544,588,632]
labels=['B0  팩 - / 첫 셀 -','B1  첫 셀 +','B2  두 번째 셀 +','B3  팩 + / 세 번째 셀 +']
for i,y in enumerate(rows):
 line([(405,y),(1010,y)],colors[i],4)
 t(74,y-16,['밸런스 핀 0','밸런스 핀 1','밸런스 핀 2','밸런스 핀 3'][i],25)
 t(470,y-31,labels[i],23,colors[i])
 t(1035,y-17,['B0 (B-)','B1','B2','B3 (B+)'][i],24)
t(1120,514,'밸런스',23);t(1120,552,'커넥터',23);t(1120,596,'4핀',23)
# return output explicitly not to pack
line([(1320,700),(1320,753),(1110,753)],black,8)
t(1160,662,'P-',25);t(635,733,'로봇 / 충전기 - ← P-',26,black)
box(50,805,1380,125,'#ffffff')
t(75,821,'① 굵은 -는 B-에만 연결합니다. P-를 배터리 -에 연결하면 BMS 보호를 우회합니다.',25)
t(75,865,'② 밸런스 4핀은 B0~B3 순서·전압을 확인한 후 연결합니다. 그림의 선 색은 구분용입니다.',25)
im.save(p/'battery-to-daly-only.png')
