from pathlib import Path
import time,re,os,subprocess,sys
log=Path('/tmp/advanced-opt-obstacle08.log');start=time.monotonic()
while time.monotonic()-start<200:
 s=log.read_text() if log.exists() else ''
 if 'NAVIGATING_APPROACH' in s:
  ident=re.search(r'Advanced run ([^;]+);',s).group(1)
  env=dict(os.environ,T1_AUDIT_RUN_ID=ident,T1_AUDIT_SOURCE_SHA=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),T1_AUDIT_OUTPUT='/tmp/advanced-live-topics-final.json')
  raise SystemExit(subprocess.call([sys.executable,'/tmp/advanced_live_topics.py'],env=env))
 if 'Final FAILED' in s:raise SystemExit('Mission failed before audit phase')
 time.sleep(.5)
raise SystemExit('Audit phase deadline')
