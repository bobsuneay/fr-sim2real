"""ROS executor runs independently; blocking task methods run outside Qt/ROS threads."""
from copy import deepcopy
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                             PlanningScene, CollisionObject, AttachedCollisionObject,
                             DisplayTrajectory)
from moveit_msgs.srv import GetPositionFK, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from scipy.spatial.transform import Rotation
from .core import pose_values, opening_to_joint


def ros_pose(values):
    values = pose_values(values)
    p = Pose()
    p.position.x, p.position.y, p.position.z = values[:3]
    q = Rotation.from_euler('xyz', values[3:], degrees=True).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return p


class Backend(Node):
    def __init__(self, cfg, mock=False, allow_execution=False):
        super().__init__('fr3_control_panel')
        self.cfg, self.mock = cfg, mock
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.active = None
        self.joints = {}
        self.pose = None
        self.pose_time = 0.0
        self.fk_pending = None
        self.payload = False
        self.fault = None
        self.connected = False
        self.execution_enabled = allow_execution
        self.motion_armed = False
        self.grasp_pending = False
        self.create_subscription(JointState, cfg['joint_topic'], self.on_joints,
                                 qos_profile_sensor_data)
        self.move = ActionClient(self, MoveGroup, cfg['move_action'])
        self.grip = ActionClient(self, GripperCommand, cfg['gripper_action'])
        self.fk = self.create_client(GetPositionFK, cfg['fk_service'])
        self.scene = self.create_client(ApplyPlanningScene, cfg['scene_service'])
        self.display = self.create_publisher(DisplayTrajectory, '/display_planned_path', 1)
        self.create_timer(0.2, self.poll_fk)
        # Pass the node's context explicitly.  Some Humble vendor builds return a
        # null executor when the default context is not selected after rclpy.init().
        self.executor = MultiThreadedExecutor(num_threads=2, context=self.context)
        if self.executor is None:  # defensive compatibility for patched/vendor rclpy
            from rclpy.executors import SingleThreadedExecutor
            self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def connect(self, timeout=5.0, health_check=None):
        """Observation/planning do not require an active gripper action server."""
        if self.fault:
            raise RuntimeError(self.fault)
        deadline = time.monotonic() + timeout
        missing = '等待后端'
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                raise RuntimeError('连接已取消')
            if health_check:
                health_check()
            ready = self.move.server_is_ready() and self.fk.service_is_ready() and self.scene.service_is_ready()
            if ready:
                self.connected = True
                try:
                    self.state()
                    return True
                except RuntimeError as exc:
                    missing = str(exc)
                    self.connected = False
            time.sleep(.1)
        raise RuntimeError('MoveIt / FK / 场景 / 实时反馈未就绪：'+missing)

    def prepare_connection(self):
        if self.fault or self.active is not None:
            raise RuntimeError(self.fault or '仍有活动目标，不能重新连接')
        self.stop_event.clear()

    def check_namespace_free(self):
        if self.move.server_is_ready() or self.fk.service_is_ready() or self.grip.server_is_ready():
            raise RuntimeError('已有 ROS 控制后端，请先退出；自管模式不能接管另一套后端')

    def disconnect(self):
        self.cancel()
        self.connected = False
        self.motion_armed = False
        with self.lock:
            self.joints.clear()
            self.pose = None
            self.pose_time = 0.0

    def on_joints(self, msg):
        now = time.monotonic()
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                if math.isfinite(value):
                    self.joints[name] = (value, now)

    def snapshot(self):
        with self.lock:
            return dict(self.joints), deepcopy(self.pose), self.pose_time

    def state(self):
        if not self.connected:
            raise RuntimeError('机械臂与夹爪未连接')
        if self.fault:
            raise RuntimeError(self.fault)
        values, _, _ = self.snapshot()
        now = time.monotonic()
        required = self.cfg['joints'] + [self.cfg['gripper_joint']]
        if any(n not in values or now-values[n][1] > self.cfg['feedback_timeout'] for n in required):
            raise RuntimeError('机械臂或夹爪反馈缺失/过期，拒绝规划和执行')
        msg = JointState()
        msg.name = required
        msg.position = [values[n][0] for n in required]
        return msg

    def begin(self):
        if self.fault:
            raise RuntimeError(self.fault)
        if not self.connected:
            raise RuntimeError('请先连接机械臂与夹爪')
        self.stop_event.clear()

    def require_execution(self):
        if not self.execution_enabled or not self.motion_armed:
            raise RuntimeError('运动未授权：启动时允许执行，并在界面勾选允许运动')
        if self.stop_event.is_set():
            raise RuntimeError('操作已取消')

    def capture(self):
        self.state()
        _, values, stamp = self.snapshot()
        if values is None or time.monotonic()-stamp > self.cfg['feedback_timeout']:
            raise RuntimeError('TCP 反馈尚未就绪或已过期')
        return values

    def poll_fk(self):
        if self.fk_pending is not None and not self.fk_pending.done():
            return
        if not self.fk.service_is_ready():
            return
        try:
            with self.lock:
                state = self.state()
                sampled = min(self.joints[n][1] for n in state.name)
        except RuntimeError:
            return
        req = GetPositionFK.Request()
        req.header.frame_id = self.cfg['frame']
        req.fk_link_names = [self.cfg['tcp']]
        req.robot_state.joint_state = state
        # FK age is bounded by the oldest input joint, not by service response time.
        self.fk_pending = self.fk.call_async(req)

        def receive(future):
            try:
                result = future.result()
                if result.error_code.val != 1 or not result.pose_stamped:
                    return
                if (not self.connected or result.fk_link_names != [self.cfg['tcp']]
                        or result.pose_stamped[0].header.frame_id != self.cfg['frame']):
                    return
                p = result.pose_stamped[0].pose
                q = p.orientation
                angles = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
                with self.lock:
                    self.pose = [p.position.x, p.position.y, p.position.z] + angles.tolist()
                    self.pose_time = sampled
            except Exception as exc:
                self.get_logger().warning(str(exc))
        self.fk_pending.add_done_callback(receive)

    def wait(self, future, timeout=15, check_stop=True):
        deadline = time.monotonic()+timeout
        while not future.done():
            if check_stop and self.stop_event.is_set():
                raise RuntimeError('操作已取消；已发送动作取消请求')
            if time.monotonic() > deadline:
                self.cancel()
                raise TimeoutError('ROS 操作超时，已请求取消；请确认设备状态')
            time.sleep(.02)
        return future.result()

    def cancel(self):
        self.stop_event.set()
        with self.lock:
            if self.active is not None:
                self.active.cancel_goal_async()

    def action(self, client, goal, timeout=120):
        if self.stop_event.is_set():
            raise RuntimeError('操作已取消')
        if not client.wait_for_server(timeout_sec=3):
            raise RuntimeError('控制 action 未上线：' + client._action_name)
        future = client.send_goal_async(goal)
        # A late goal acceptance must also be canceled after timeout/cancel.
        def late_cancel(f):
            handle = f.result()
            if handle and handle.accepted and self.stop_event.is_set():
                handle.cancel_goal_async()
        future.add_done_callback(late_cancel)
        try:
            handle = self.wait(future)
        except Exception:
            self.fault = '目标接收状态不确定；确认设备停止后重启面板'
            raise
        if not handle.accepted:
            raise RuntimeError('控制器拒绝目标')
        with self.lock:
            self.active = handle
        try:
            if self.stop_event.is_set():
                handle.cancel_goal_async()
                raise RuntimeError('操作已取消')
            result_future = handle.get_result_async()
            deadline = time.monotonic() + timeout
            while not result_future.done():
                self.state()
                if self.stop_event.is_set():
                    raise RuntimeError('操作已取消')
                if time.monotonic() > deadline:
                    raise TimeoutError('动作执行超时')
                time.sleep(.05)
            result = result_future.result()
            if result.status != 4:
                raise RuntimeError(f'动作未成功，状态码 {result.status}')
            return result.result
        except Exception:
            self.cancel()
            try:
                # Do not permit another motion before the old action terminates.
                self.wait(handle.get_result_async(), timeout=5, check_stop=False)
            except Exception:
                self.fault = '动作停止状态未确认；请现场确认停止后重启面板'
            raise
        finally:
            with self.lock:
                self.active = None

    def motion(self, values, execute=False):
        self.state()
        if execute:
            self.require_execution()
        if self.grasp_pending:
            raise RuntimeError('夹持状态待确认；请确认并挂载物体，或张开释放后再运动')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.cfg['group']
        req.pipeline_id = 'ompl'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = float(self.cfg['velocity'])
        req.max_acceleration_scaling_factor = float(self.cfg['acceleration'])
        # Server takes its latest complete planning-scene state, including attachments.
        req.start_state.is_diff = True
        p = ros_pose(values)
        pc = PositionConstraint()
        pc.header.frame_id = self.cfg['frame']
        pc.link_name = self.cfg['tcp']
        pc.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])
        pc.constraint_region.primitives = [sphere]
        region = deepcopy(p)
        region.orientation.x = region.orientation.y = region.orientation.z = 0.0
        region.orientation.w = 1.0
        pc.constraint_region.primitive_poses = [region]
        oc = OrientationConstraint()
        oc.header.frame_id = self.cfg['frame']
        oc.link_name = self.cfg['tcp']
        oc.orientation = p.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = .02
        oc.weight = 1.0
        req.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        goal.planning_options.plan_only = not execute
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self.action(self.move, goal)
        if result.error_code.val != 1:
            raise RuntimeError(f'MoveIt 规划/执行失败：{result.error_code.val}，未继续夹取')
        if not result.planned_trajectory.joint_trajectory.points:
            raise RuntimeError('MoveIt 返回空轨迹')
        self.display.publish(DisplayTrajectory(trajectory_start=result.trajectory_start,
                                              trajectory=[result.planned_trajectory]))

    def gripper(self, opening, grasp=False):
        self.state()
        self.require_execution()
        if self.payload and opening > 0:
            raise RuntimeError('当前挂载了物体，请使用“释放物体”保持碰撞场景一致')
        goal = GripperCommand.Goal()
        goal.command.position = opening_to_joint(opening, self.cfg['closed_position'])
        goal.command.max_effort = 0.0  # Actual force configured in HKV hardware parameters.
        if grasp:
            # Even an uncertain close result can leave an object in the fingers.
            self.grasp_pending = True
        result = self.action(self.grip, goal, 20)
        if not (result.reached_goal or result.stalled):
            raise RuntimeError('夹爪未到位且未报告接触停滞')
        if opening > 0 and not result.reached_goal:
            raise RuntimeError('夹爪张开/设定开度时停滞，未到目标；保留挂载物体模型')
        if grasp and not self.mock and not result.stalled:
            raise RuntimeError('夹爪完全闭合但没有停滞，可能空抓；不搬运，请检查并释放')
        if opening == 100 and result.reached_goal:
            self.grasp_pending = False

    def apply_scene(self, scene):
        if not self.scene.wait_for_service(timeout_sec=3):
            raise RuntimeError('MoveIt 场景服务未上线')
        response = self.wait(self.scene.call_async(ApplyPlanningScene.Request(scene=scene)))
        if not response.success:
            raise RuntimeError('碰撞场景更新失败')

    def attach(self):
        obj = CollisionObject()
        obj.header.frame_id = self.cfg['tcp']
        obj.id = 'panel_payload'
        obj.operation = CollisionObject.ADD
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                         dimensions=list(map(float, self.cfg['payload_size'])))]
        obj.primitive_poses = [ros_pose(self.cfg['payload_offset'] + [0, 0, 0])]
        attachment = AttachedCollisionObject(link_name=self.cfg['tcp'], object=obj,
                                              touch_links=self.cfg['touch_links'])
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attachment]
        self.payload = True
        try:
            self.apply_scene(scene)
            self.grasp_pending = False
        except Exception:
            self.fault = '夹取后物体碰撞模型更新失败；需核对场景和物体后重启面板'
            raise

    def release(self):
        # Keep attachment during finger opening; failure leaves conservative geometry.
        held = self.payload
        self.payload = False
        try:
            self.gripper(100)
        except Exception:
            self.payload = held
            raise
        if held:
            self.payload = True
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                link_name=self.cfg['tcp'], object=CollisionObject(id='panel_payload', operation=1))]
            # MoveIt detaches the object into the world at the current pose.
            try:
                self.apply_scene(scene)
                self.payload = False
            except Exception:
                self.fault = '释放后的场景状态不确定；确认物体和碰撞场景后重启面板'
                raise

    def obstacle(self, name, values, size):
        if not name.strip() or name == 'panel_payload' or any(x <= 0 for x in size):
            raise ValueError('障碍物名称或尺寸无效')
        obj = CollisionObject(id=name, operation=CollisionObject.ADD)
        obj.header.frame_id = self.cfg['frame']
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
        obj.primitive_poses = [ros_pose(values)]
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        self.apply_scene(scene)

    def close(self):
        self.disconnect()
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.destroy_node()
