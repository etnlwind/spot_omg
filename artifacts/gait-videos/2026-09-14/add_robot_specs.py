from pathlib import Path
import subprocess,sys
from PIL import Image,ImageDraw,ImageFont
source=Path(sys.argv[1]).resolve()
footer=source.with_name(source.stem+'-specs.png')
im=Image.new('RGB',(1920,156),(17,24,34));d=ImageDraw.Draw(im)
font=ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc',26)
lines=[
'Spot OMG  |  총중량 2.754kg (배터리·쿠션 포함 실측)  |  중앙 알루미늄 프레임 300mm  |  몸체 PLA / PETG',
'모터 12개: J1·J3 STS3215 ×8 / J2 STS3250 ×4  |  배터리 3S LiPo, 공칭 11.1V  |  쿠션 길이 27mm / 바닥 지름 37.3mm',
'시험: centerpivot · J2 수직 정렬 · 스윙 높이 목표 32mm  |  10초 대기 → 20초 전진 100%  |  MuJoCo 물리 0.5ms / 제어 20ms',
'검증 범위: 시뮬레이션  |  총질량만 실측 반영 · 질량 분포/관성/모터 응답/쿠션 물성은 추정 포함  |  영상 30초, 25fps'
]
if 'ground-transfer-candidate' in source.name:
 lines[2]='시험: 지지 전환 준비 · 실험 (검증실패)  |  발 높이 목표 32mm / 주기 3.2초 / duty 0.70  |  10초 대기 → 전진 20초'
if 'support-transition-feedforward' in source.name:
 lines[2]='지지 전환 · 반복 보정 · 실험 (검증실패)  |  발 높이 목표 32mm / 주기 3.2초 / 출발 4초 완화  |  10초 대기 → 전진 20초'
for i,line in enumerate(lines):d.text((20,10+36*i),line,font=font,fill=(235,241,249))
im.save(footer)
out=source.with_name(source.stem+'-specs.mp4')
subprocess.run(['/opt/homebrew/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(source),'-i',str(footer),'-filter_complex','[0:v]pad=1920:1236:0:0:black[v];[v][1:v]overlay=0:1080:format=auto[out]','-map','[out]','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],check=True)
print(out)
