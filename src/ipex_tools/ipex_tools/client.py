"""Python client for Roxey motion primitives.

    import rclpy
    from ipex_tools.client import RoxeyClient

    rclpy.init()
    roxey = RoxeyClient()
    r = roxey.run('set_arm', arm='front', theta_deg=-10)
    print(r.success, r.failure_code, r.duration_s, r.energy_j)
    roxey.close()
    rclpy.shutdown()

Each call blocks until the primitive finishes and returns a PrimitiveResult
with time (duration_s), energy (energy_j, energy_above_idle_j, per-group
breakdown) and primitive-specific outputs. Ctrl+C during run() cancels the
primitive on the rover.
"""

import math
from dataclasses import dataclass, field

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from ipex_interfaces.action import Primitive as PrimitiveAction


ACTION_NAME = '/ipex/primitive'


class RoxeyError(RuntimeError):
    """Could not talk to the primitive server."""


@dataclass
class PrimitiveResult:
    name: str
    params: dict
    success: bool = False
    failure_code: str = ''
    message: str = ''
    duration_s: float = math.nan
    energy_j: float = math.nan
    energy_above_idle_j: float = math.nan
    idle_power_w: float = math.nan
    groups: dict = field(default_factory=dict)
    outputs: dict = field(default_factory=dict)
    tag: str = ''

    def as_row(self):
        return {
            'name': self.name, 'tag': self.tag, 'success': self.success,
            'failure_code': self.failure_code, 'duration_s': self.duration_s,
            'energy_j': self.energy_j, 'energy_above_idle_j': self.energy_above_idle_j,
            'idle_power_w': self.idle_power_w,
            **{f'{g}_j': e for g, e in self.groups.items()},
            'params': yaml.safe_dump(self.params, default_flow_style=True).strip(),
            'outputs': yaml.safe_dump(self.outputs, default_flow_style=True).strip(),
            'message': self.message,
        }


class RoxeyClient:

    def __init__(self, node=None, server_timeout_s=10.0):
        self._own_node = node is None
        self.node = node or Node('roxey_client')
        self.server_timeout_s = server_timeout_s
        self._client = ActionClient(self.node, PrimitiveAction, ACTION_NAME)

    def close(self):
        self._client.destroy()
        if self._own_node:
            self.node.destroy_node()

    def run(self, name, tag='', on_feedback=None, **params):
        """Run one primitive and block until it finishes."""
        if not self._client.wait_for_server(timeout_sec=self.server_timeout_s):
            raise RoxeyError(
                f'primitive server not available on {ACTION_NAME} '
                '(is the rover running? sudo systemctl status ipex-rover)')

        goal = PrimitiveAction.Goal()
        goal.name = name
        goal.params_yaml = yaml.safe_dump(params, default_flow_style=True).strip() if params else ''
        goal.tag = tag

        fb_cb = (lambda msg: on_feedback(msg.feedback)) if on_feedback else None
        send_future = self._client.send_goal_async(goal, feedback_callback=fb_cb)
        rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()

        if handle is None or not handle.accepted:
            return PrimitiveResult(name=name, params=params, tag=tag, failure_code='BUSY',
                                   message='goal rejected: another primitive is running')

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self.node, result_future)
        except KeyboardInterrupt:
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=3.0)
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
            if not result_future.done():
                raise

        r = result_future.result().result
        try:
            outputs = yaml.safe_load(r.outputs_yaml) or {}
        except yaml.YAMLError:
            outputs = {'raw': r.outputs_yaml}

        return PrimitiveResult(
            name=name, params=params, tag=tag,
            success=r.success, failure_code=r.failure_code, message=r.message,
            duration_s=r.duration_s, energy_j=r.energy_j,
            energy_above_idle_j=r.energy_above_idle_j, idle_power_w=r.idle_power_w,
            groups=dict(zip(r.groups, r.group_energy_j)), outputs=outputs)
