from pathlib import Path
from datetime import datetime,timezone
import subprocess,time,csv,signal,json,sys
before=set(Path('run_data/advanced').glob('*'));log=Path(sys.argv[1]).open('w')
p=subprocess.Popen(['./scripts/run_advanced_demo.sh','--headless'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
start=time.monotonic();sent=None;run=None
while p.poll() is None and time.monotonic()-start<150:
 new=set(Path('run_data/advanced').glob('*'))-before
 if new:
  run=next(iter(new));trace=Path('run_data')/run.name/'sim_trace.csv'
  if trace.exists():
   rows=list(csv.DictReader(trace.read_text().splitlines()))
   moving=[r for r in rows[-100:] if r.get('u_pol_x') and float(r['u_pol_x'])>.1 and float(r['sim_time'])>15]
   if moving:
    sent=dict(utc=datetime.now(timezone.utc).isoformat(),runner_pid=p.pid,signal='SIGINT',observed_motion=moving[-1]);p.send_signal(signal.SIGINT);break
 time.sleep(.25)
if sent is None:
 p.send_signal(signal.SIGTERM)
 raise RuntimeError('Did not observe required physical motion before interrupt')
code=p.wait(timeout=100);log.close()
assert run
(run/'interrupt-request.json').write_text(json.dumps(dict(request=sent,wrapper_exit=code),indent=2)+'\n')
print(run, 'exit',code,'motion',sent['observed_motion']['u_pol_x'],flush=True)
