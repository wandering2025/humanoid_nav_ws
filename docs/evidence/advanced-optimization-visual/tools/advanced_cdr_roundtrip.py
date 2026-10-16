"""CPU-only reproduction: serialized CDR bytes can differ for equal Path fields."""
import sys,json
from pathlib import Path as File
sys.path.insert(0,'scripts/demo');from cpu_guard import enter;enter()
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
checks=[]
for count in (6,10,20,65,71,139,809):
 m=Path();m.header.frame_id='map';m.header.stamp.sec=99;m.header.stamp.nanosec=420000000
 for i in range(count):
  p=PoseStamped();p.header=m.header;p.pose.position.x=i*.05;p.pose.orientation.w=1.;m.poses.append(p)
 blobs=[serialize_message(m) for _ in range(25)]
 for b in list(blobs):blobs.append(serialize_message(deserialize_message(b,Path)))
 differences=sorted({i for b in blobs[1:] for i,(x,y) in enumerate(zip(blobs[0],b)) if x!=y})
 checks.append(dict(poses=count,bytes=len(blobs[0]),distinct_serializations=len(set(blobs)),decoded_messages_all_equal=all(deserialize_message(b,Path)==m for b in blobs),different_offsets=differences[:30]))
File(sys.argv[1]).write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks,indent=2))
