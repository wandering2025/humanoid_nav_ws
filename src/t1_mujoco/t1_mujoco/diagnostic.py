"""One-shot ROS node: inspect runtime imports and paths, then exit."""

from datetime import datetime, timezone
import json
import os
import sys

from .runtime import import_dependency, inspect_runtime


def main(args=None):
    node = None
    context = None
    try:
        rclpy = import_dependency('rclpy')
        from rclpy.context import Context
        from rclpy.node import Node

        context = Context()
        rclpy.init(args=args, context=context)
        node = Node('t1_diagnostic', context=context)
        for name, default in (
            ('upstream_path', os.getenv('T1_UPSTREAM', '')),
            ('assets_path', os.getenv('T1_ASSETS', '')),
            ('mode', 'sim_only'), ('task', 't1_walk'),
        ):
            node.declare_parameter(name, default)
        values = {name: node.get_parameter(name).value for name in ('upstream_path', 'assets_path', 'mode', 'task')}
        report = inspect_runtime(**values)
        report.update(
            schema=1, status='PASS', utc=datetime.now(timezone.utc).isoformat(),
            executable=sys.executable, python=sys.version, prefix=sys.prefix,
            ros_distro=os.getenv('ROS_DISTRO', ''), ros_domain_id=os.getenv('ROS_DOMAIN_ID', '0'),
            mode=values['mode'], task=values['task'],
            use_sim_time=node.get_parameter('use_sim_time').value,
        )
        print('T1_DIAGNOSTIC ' + json.dumps(report, sort_keys=True), flush=True)
        return 0
    except Exception as exc:
        print(f'T1 diagnostic failed: {exc}', file=sys.stderr, flush=True)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()


if __name__ == '__main__':
    sys.exit(main())
