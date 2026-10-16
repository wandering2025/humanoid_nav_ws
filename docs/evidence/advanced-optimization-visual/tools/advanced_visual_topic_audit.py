import sys,time,json,hashlib,os
from pathlib import Path as File
sys.path.insert(0,'scripts/demo');from cpu_guard import enter;enter()
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.serialization import serialize_message
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
run=File(sys.argv[1]);output=File(sys.argv[2]);rclpy.init();n=Node('advanced_display_readonly_audit')
q=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL);records=[];raw={};geometry={};seen_arrival=None
def path(topic,m):
 digest=hashlib.sha256(serialize_message(m)).hexdigest()
 g=hashlib.sha256(repr((m.header.frame_id,tuple((p.pose.position.x,p.pose.position.y,p.pose.position.z,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w) for p in m.poses))).encode()).hexdigest()
 records.append(dict(topic=topic,wall=time.monotonic(),sha256=digest,geometry_sha256=g,poses=len(m.poses),frame=m.header.frame_id))
 raw.setdefault(digest,set()).add(topic);geometry.setdefault(g,set()).add(topic)
def marker(m):records.append(dict(topic='markers',wall=time.monotonic(),texts=[v.text for v in m.markers if v.text]))
n.create_subscription(Path,'/plan',lambda m:path('planner',m),10)
n.create_subscription(Path,'/t1/viz/live_plan',lambda m:path('display',m),q)
n.create_subscription(Path,'/t1/viz/route_overview',lambda m:path('overview',m),q)
n.create_subscription(MarkerArray,'/t1/viz/demo',marker,q)
start=time.monotonic();reason='DEADLINE'
while time.monotonic()-start<350:
 rclpy.spin_once(n,timeout_sec=.1)
 try:
  p=run/'partial-result.json'
  if p.exists() and json.loads(p.read_text()).get('status')=='ARRIVED_SAFE_STOP':
   if seen_arrival is None:seen_arrival=time.monotonic()
   if time.monotonic()-seen_arrival>1:reason='ARRIVED';break
 except ValueError:pass
 if (run/'result.json').exists():reason='RUN_ENDED';break
result=dict(run_id=run.name,source_sha=os.environ['T1_AUDIT_SOURCE_SHA'],duration_wall_s=time.monotonic()-start,end_reason=reason,records=records,matched_planner_display_hashes=sum({'planner','display'}<=v for v in raw.values()),overview_messages=sum(x['topic']=='overview' for x in records),overview_geometry_never_displayed_as_execution=not any({'overview','display'}<=v for v in geometry.values()),all_display_messages_match_planner=all('planner' in v for v in raw.values() if 'display' in v),scope='Readonly full-mission subscriptions. Execution display must match actual /plan bytes; display-only A-B overview geometry must never appear on blue execution topic, independent of delivery order or stamps.')
output.write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='records'})
n.destroy_node();rclpy.shutdown()
assert result['overview_messages']>=1 and result['matched_planner_display_hashes']>=3 and result['overview_geometry_never_displayed_as_execution'] and result['all_display_messages_match_planner'],result
