"""Verify kernel port release after the observer ROS context has exited."""
import json,os,sys,time
from pathlib import Path
out=Path(sys.argv[1]);code=int(sys.argv[2])
os.environ.update(T1_DEMO_DIR=str(out),T1_DEMO_RUN_ID=out.name)
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'demo'))
import demo
r=json.loads((out/'result.json').read_text())
r['dds_port_released']=demo.wait_port_reusable()
r['remaining_project_processes']=demo.project_processes()
r['status']='SUCCESS' if code==0 and r.get('validation_before_port_check') and r['dds_port_released'] and not r['remaining_project_processes'] else 'FAILED'
(out/'final-result.json').write_text(json.dumps(r,indent=2)+'\n')
print(f"[advanced] Final {r['status']} | arrived={r.get('arrived')} | contacts={r.get('contacts')} | fall={r.get('fell')} | {out}",flush=True)
raise SystemExit(0 if r['status']=='SUCCESS' else 1)
