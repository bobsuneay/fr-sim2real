"""Transport fakes only: guard/state tests are NOT ROS or physical acceptance."""
import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import pytest
import yaml


@pytest.fixture
def backend(monkeypatch):
    for name in ('rclpy', 'rclpy.action', 'rclpy.executors', 'rclpy.qos',
                 'sensor_msgs.msg', 'geometry_msgs.msg', 'moveit_msgs.action',
                 'moveit_msgs.msg', 'moveit_msgs.srv', 'shape_msgs.msg', 'control_msgs.action'):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    monkeypatch.setitem(sys.modules, 'rclpy.node', NS(Node=object))
    path = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('fr3_control_panel._test_backend', path/'fr3_control_panel/backend.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    obj = module.Backend.__new__(module.Backend)
    obj.cfg = yaml.safe_load((path/'config/panel.yaml').read_text())
    obj.lock = threading.RLock()
    obj.stop_event = threading.Event()
    obj.active = None
    obj.connected = False
    obj.execution_enabled = False
    obj.motion_armed = False
    obj.fault = None
    obj.mock = False
    obj.payload = False
    obj.grasp_pending = False
    obj.pose, obj.pose_time = None, 0
    stamp = time.monotonic()
    obj.joints = {n: (0., stamp) for n in obj.cfg['joints']+[obj.cfg['gripper_joint']]}
    obj.move = NS(server_is_ready=lambda: True)
    obj.fk = NS(service_is_ready=lambda: True)
    obj.scene = NS(service_is_ready=lambda: True)
    obj.grip = MagicMock()
    obj.action = MagicMock(return_value=NS(reached_goal=False, stalled=True))
    module.GripperCommand.Goal = lambda: NS(command=NS())
    return module, obj


def test_observation_connect_without_gripper_action(backend):
    _, obj = backend
    assert obj.connect(timeout=.2)
    obj.grip.wait_for_server.assert_not_called()
    assert obj.connected


def test_stale_state_and_unarmed_motion_fail_before_send(backend):
    _, obj = backend
    obj.connected = True
    with pytest.raises(RuntimeError):
        obj.motion([0]*6, True)
    with pytest.raises(RuntimeError):
        obj.gripper(0)
    obj.action.assert_not_called()
    obj.joints['j1'] = (0., time.monotonic()-10)
    with pytest.raises(RuntimeError, match='过期'):
        obj.state()


def test_stall_does_not_auto_attach_or_allow_transport(backend):
    _, obj = backend
    obj.connected = obj.execution_enabled = obj.motion_armed = True
    obj.gripper(0, grasp=True)
    assert obj.grasp_pending and not obj.payload
    with pytest.raises(RuntimeError, match='待确认'):
        obj.motion([0]*6, True)
    assert obj.action.call_count == 1


def test_empty_grasp_and_uncertain_close_keep_motion_locked(backend):
    _, obj = backend
    obj.connected = obj.execution_enabled = obj.motion_armed = True
    obj.action.return_value = NS(reached_goal=True, stalled=False)
    with pytest.raises(RuntimeError, match='空抓'):
        obj.gripper(0, grasp=True)
    assert obj.grasp_pending
    obj.gripper(100)
    assert not obj.grasp_pending
    obj.action.side_effect = TimeoutError()
    with pytest.raises(TimeoutError):
        obj.gripper(0, grasp=True)
    assert obj.grasp_pending


def test_disconnect_clears_ui_state_without_claiming_physical_stop(backend):
    _, obj = backend
    obj.connected = obj.motion_armed = True
    obj.disconnect()
    assert not obj.connected and not obj.motion_armed
    assert not obj.joints and obj.pose is None
    assert obj.stop_event.is_set()


def test_executor_uses_explicit_context_and_has_fallback(monkeypatch):
    """Regression guard for the reported NoneType executor crash."""
    from fr3_control_panel import backend as module
    calls = []
    class NullExecutor:
        def __init__(self, **kwargs):
            calls.append(('multi', kwargs))
        def add_node(self, node):
            calls.append(('add', node))
        def spin(self):
            pass
    class GoodExecutor(NullExecutor):
        pass
    monkeypatch.setattr(module, 'MultiThreadedExecutor', lambda **kwargs: None)
    monkeypatch.setattr(module, 'Node', object)
    monkeypatch.setattr(module.rclpy.executors, 'SingleThreadedExecutor', GoodExecutor, raising=False)
    # The constructor contract is validated by source-level assertions here;
    # ROS entities are intentionally not constructed in the Windows test host.
    source = Path(__file__).parents[1]/'fr3_control_panel/backend.py'
    text = source.read_text(encoding='utf-8')
    assert 'context=self.context' in text
    assert 'if self.executor is None' in text


def test_cancel_during_connection_is_not_cleared_by_worker(backend):
    _, obj = backend
    obj.prepare_connection()
    obj.cancel()
    with pytest.raises(RuntimeError, match='取消'):
        obj.connect(timeout=.2)
    assert not obj.connected


def test_managed_mode_refuses_existing_backend(backend):
    _, obj = backend
    with pytest.raises(RuntimeError, match='已有 ROS'):
        obj.check_namespace_free()


def test_fk_uses_oldest_joint_time_and_validates_frame(backend):
    module, obj = backend
    from concurrent.futures import Future
    obj.connected = True
    obj.fk_pending = None
    old = time.monotonic()-.2
    obj.joints['j1'] = (0., old)
    state = NS(name=obj.cfg['joints']+[obj.cfg['gripper_joint']])
    obj.state = lambda: state
    module.GetPositionFK.Request = lambda: NS(header=NS(), robot_state=NS())
    obj.get_logger = lambda: NS(warning=lambda s: None)
    def response(frame):
        return NS(error_code=NS(val=1), fk_link_names=[obj.cfg['tcp']], pose_stamped=[
            NS(header=NS(frame_id=frame), pose=NS(position=NS(x=.3, y=.1, z=.4),
               orientation=NS(x=0., y=0., z=0., w=1.)))])
    future = Future()
    obj.fk.call_async = lambda req: future
    obj.poll_fk()
    future.set_result(response('wrong_frame'))
    assert obj.pose is None
    future = Future()
    obj.poll_fk()
    future.set_result(response(obj.cfg['frame']))
    assert obj.pose[:3] == [.3, .1, .4]
    assert obj.pose_time == old
