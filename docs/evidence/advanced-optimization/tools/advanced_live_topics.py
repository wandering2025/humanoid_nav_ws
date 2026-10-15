import sys,time,json,hashlib,os
sys.path.insert(0,'scripts/demo');from cpu_guard import enter;enter()
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.serialization import serialize_message
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
rclpy.init();n=Node('advanced_display_readonly_audit');q=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
records=[];raw={};match=[]
def path(topic,m):
 digest=hashlib.sha256(serialize_message(m)).hexdigest();records.append(dict(topic=topic,wall=time.monotonic(),sha256=digest,poses=len(m.poses),frame=m.header.frame_id))
 raw.setdefault(digest,set()).add(topic)
def marker(m):records.append(dict(topic='markers',wall=time.monotonic(),texts=[v.text for v in m.markers if v.text]))
n.create_subscription(Path,'/plan',lambda m:path('planner',m),10)
n.create_subscription(Path,'/t1/viz/live_plan',lambda m:path('display',m),q)
n.create_subscription(Path,'/t1/viz/route_overview',lambda m:path('overview',m),q)
n.create_subscription(MarkerArray,'/t1/viz/demo',marker,q)
start=time.monotonic()
while time.monotonic()-start<30:rclpy.spin_once(n,timeout_sec=.1)
result=dict(run_id=os.environ.get('T1_AUDIT_RUN_ID'),source_sha=os.environ.get('T1_AUDIT_SOURCE_SHA'),duration_wall_s=time.monotonic()-start,records=records,matched_planner_display_hashes=sum({'planner','display'}<=v for v in raw.values()),scope='Readonly ROS subscriptions during headless run. Serialized message equality proves actual planner Path forwarded unchanged; not an RViz rendering check.')
open(os.environ.get('T1_AUDIT_OUTPUT','/tmp/advanced-live-topics.json'),'w').write(json.dumps(result,indent=2)+'\n')
print({k:v for k,v in result.items() if k!='records'});n.destroy_node();rclpy.shutdown()
