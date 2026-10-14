"""One approximate initialization, after AMCL is active. Never reads truth."""
import argparse
import math
import time


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw', type=float, help='radians in map')
    options = parser.parse_args()
    if not all(math.isfinite(v) for v in (options.x, options.y, options.yaw)):
        parser.error('pose must be finite')
    rclpy.init()
    node = Node('t1_initial_pose')
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    client = node.create_client(GetState, '/amcl/get_state')
    try:
        deadline = time.monotonic()+20
        active = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=1)
                active = future.done() and future.result().current_state.id == 3
            if active and pub.get_subscription_count() > 0 and node.get_clock().now().nanoseconds > 0:
                break
        else:
            raise RuntimeError('AMCL must be active with /clock and /initialpose subscription')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = options.x, options.y
        msg.pose.pose.orientation.z = math.sin(options.yaw/2)
        msg.pose.pose.orientation.w = math.cos(options.yaw/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .09
        msg.pose.covariance[35] = .09
        pub.publish(msg)
        # Allow reliable delivery; publish exactly once.
        end = time.monotonic()+.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        print(f'Initialized AMCL once in map: {options.x}, {options.y}, {options.yaw}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
