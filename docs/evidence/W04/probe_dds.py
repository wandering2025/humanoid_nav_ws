import rclpy,time,subprocess,sys
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy
rclpy.init();n=rclpy.create_node('probe_parent');received=[]
n.create_subscription(String,'/probe',lambda m:received.append(m.data),QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
code='''import rclpy,time
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('probe_child');p=n.create_publisher(String,'/probe',10)
end=time.monotonic()+6
while time.monotonic()<end:
 p.publish(String(data='hello'));rclpy.spin_once(n,timeout_sec=0);time.sleep(.05)
print('child nodes',n.get_node_names(),flush=True)
print('child subscriptions',[str(x) for x in n.get_subscriptions_info_by_topic('/probe')],flush=True)
n.destroy_node();rclpy.shutdown()
'''
p=subprocess.Popen([sys.executable,'-c',code]);end=time.monotonic()+7
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
p.wait();print('received',len(received));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if received else 2)
