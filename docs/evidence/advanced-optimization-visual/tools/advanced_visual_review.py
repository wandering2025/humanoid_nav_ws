import sys,json,hashlib
from pathlib import Path
import cv2
import numpy as np
run,out=map(Path,sys.argv[1:3])
r=json.loads((run/'final-result.json').read_text());o=json.loads((run/'observation.json').read_text());v=json.loads((out/'video-verification.json').read_text())
frames=[json.loads(l) for l in (out/'frames.jsonl').read_text().splitlines()];start=frames[0]['mono'];fps=v['fps']
active=[a for a in o['obstacles'] if a['phase'] not in ('staged','finished')];hold=[a for a in active if a['phase'].startswith('hold')]
mapping=[g['result'] for g in r['goals'] if g['label'].startswith('MAPPING_')]
mid=lambda n:(mapping[n]['accepted_sim']+mapping[n]['end_sim'])/2
stages=[('A and initial LiDAR map',None),('West area: live blue plan',mid(0)),('Turn into west gallery',mid(1)),('Northward mapping: map grows',mid(2)),('Turn through north gallery',mid(3)),('East survey around occlusion',mid(4)),('Map ready: cyan A-B overview',r['navigation_start_sim']+1),('Cart enters actual route',active[0]['sim_time']+3),('Cart holds: robot waiting',hold[len(hold)//2]['sim_time']),('Waiting near end of hold',hold[-1]['sim_time']),('Cart withdraws',active[-1]['sim_time']-2),('Route clear: walking resumes',active[-1]['sim_time']+4),('Final straight approach',r['nav2']['accepted_sim']+5),('Verified B arrival',r['nav2']['end_sim']+2),('Safe-stop final view','end'),('Post-run verified evidence','card')]
review=out/'review';review.mkdir(exist_ok=False);cap=cv2.VideoCapture(v['path']);index=[];images=[]
for label,sim in stages:
 t=0. if sim is None else v['live_seconds']+3 if sim=='card' else v['live_seconds']-1 if sim=='end' else max(0.,min(o['rows'],key=lambda a:abs(a['sim']-sim))['wall']-start)
 i=min(v['decoded_frames']-1,round(t*fps));cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,img=cap.read();assert ok
 p=review/f'{len(index):02d}.png';cv2.imwrite(str(p),img);index.append(dict(label=label,sim=sim,video_frame=i,video_seconds=i/fps,path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()));images.append(img)
cap.release()
for n in range(0,len(images),4):
 thumbs=[]
 for i,img in enumerate(images[n:n+4],n):
  im=cv2.resize(img,(1024,426));bar=np.full((30,1024,3),24,np.uint8);cv2.putText(bar,f'{index[i]["video_seconds"]:.2f}s '+index[i]['label'],(10,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(240,240,240),1);thumbs.append(np.vstack((bar,im)))
 cv2.imwrite(str(review/f'contact-sheet-{n//4}.png'),np.vstack((np.hstack(thumbs[:2]),np.hstack(thumbs[2:]))))
(review/'index.json').write_text(json.dumps(dict(status='VISUAL_REVIEW_PENDING',video_sha256=v['sha256'],keyframes=index),indent=2)+'\n')
print(review)
