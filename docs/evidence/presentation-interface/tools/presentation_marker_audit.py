"""Read-only full-run check of the actual RViz B marker and NavFn overview goal."""
import sys,time,json,math
from pathlib import Path
sys.path.insert(0,'scripts/demo');from cpu_guard import enter;enter()
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path as NavPath
run,output=map(Path,sys.argv[1:3]);start=time.monotonic()
while not (run/'presentation-goal.json').exists():
 if time.monotonic()-start>30:raise SystemExit('Snapshot deadline')
 time.sleep(.1)
goal=json.loads((run/'presentation-goal.json').read_text());x,y=goal['goal'][:2]
rclpy.init();n=Node('presentation_goal_readonly_audit');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
counts=dict(markers=0,overviews=0);positions=[];overviews=[];errors=[];phases=[];arrival=None
def markers(msg):
 for m in msg.markers:
  if m.ns=='route' and m.id==2:
   cx=(min(p.x for p in m.points)+max(p.x for p in m.points))/2;cy=(min(p.y for p in m.points)+max(p.y for p in m.points))/2
   counts['markers']+=1
   if not positions or positions[-1]!=[cx,cy]:positions.append([cx,cy])
   if m.header.frame_id!='map' or math.hypot(cx-x,cy-y)>1e-6:errors.append(dict(kind='B marker mismatch',frame=m.header.frame_id,position=[cx,cy]))
  if m.ns=='phase' and (not phases or phases[-1]!=m.text):phases.append(m.text)
def path(msg):
 counts['overviews']+=1
 if not msg.poses:errors.append(dict(kind='empty overview'));return
 p=msg.poses[-1].pose.position;overviews.append([p.x,p.y])
 if msg.header.frame_id!='map' or math.hypot(p.x-x,p.y-y)>.075:errors.append(dict(kind='overview goal mismatch',position=[p.x,p.y]))
n.create_subscription(MarkerArray,'/t1/viz/demo',markers,q)
n.create_subscription(NavPath,'/t1/viz/route_overview',path,q)
while time.monotonic()-start<350:
 rclpy.spin_once(n,timeout_sec=.1)
 try:
  f=run/'partial-result.json'
  if f.exists() and json.loads(f.read_text()).get('status')=='ARRIVED_SAFE_STOP':
   if arrival is None:arrival=time.monotonic()
   if time.monotonic()-arrival>1:break
 except ValueError:pass
 if (run/'result.json').exists():break
result=dict(run_id=run.name,goal=goal['goal'],source_sha256=goal['source_sha256'],observations=counts,marker_centres=positions,overview_endpoints=overviews,phases=phases,errors=errors,duration_wall_s=time.monotonic()-start,scope='Actual readonly ROS marker/Path subscriptions, no motion publishers. MuJoCo marker separately inspected in actual arrival screenshot and tested for physical/ray isolation.')
result['status']='PASS' if counts['markers']>10 and counts['overviews']>=1 and not errors else 'FAIL'
output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if result['status']=='PASS' else 1)
