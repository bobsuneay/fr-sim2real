# 双臂 FR3 + HKV 真机调试逐项清单

本清单按现场执行顺序编写。目标是把 `src/fr3_dual_bolt_cell/docs/COMMISSIONING.md`
和根目录 `DEPLOYMENT.md` 的要求变成可以直接照着做的步骤。

执行原则：先只读反馈，再空载低速运动，最后才允许自动任务。

## 0. 前置条件

- Ubuntu 22.04，x86_64
- ROS 2 Humble 已安装
- 本仓库已构建：

```bash
cd /path/to/fr3-inspection-sim2real
bash scripts/build_humble.sh
source install/setup.bash
```

- 用户已加入串口组：

```bash
sudo usermod -aG dialout "$USER"
```

然后注销并重新登录。

## 1. 确认控制柜软件版本

分别登录左右控制柜 WebApp，进入：

```text
系统设置 -> 关于
```

确认两台都是：

```text
3.9.7
```

如果版本不同，不要只改 `hardware.yaml` 的 `firmware` 字符串。必须先确认对应驱动版本。

## 2. 确认网络和串口

### 2.1 控制柜网络

两个控制柜必须配置成不同 IP。建议：

```text
左臂：192.168.58.11
右臂：192.168.58.12
```

从 Ubuntu 测试：

```bash
ping -c 3 LEFT_ROBOT_IP
ping -c 3 RIGHT_ROBOT_IP
```

如果两个控制柜仍是同一个默认 IP，需要先解决网络隔离或修改其中一个 IP。

### 2.2 HKV 串口

```bash
ls -l /dev/serial/by-id/
```

记下左右夹爪对应的稳定设备名，例如：

```text
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_LEFT
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_RIGHT
```

不要使用 `/dev/ttyUSB0` 这种会随插拔变化的名称。

## 3. 生成本地配置目录

```bash
cd /path/to/fr3-inspection-sim2real
bash scripts/init_config.sh /absolute/fr3-config
```

会生成：

```text
/absolute/fr3-config/
├── hardware.yaml
├── real_feedback.yaml
├── arms.yaml
├── scene.yaml
├── inspection.yaml
└── tracker.yaml
```

## 4. 填写 hardware.yaml

编辑：

```text
/absolute/fr3-config/hardware.yaml
```

必须填写：

```yaml
driver_package: fairino_hardware_v3_9_7
firmware: '3.9.7'
commissioned: false
left:
  robot_ip: '左臂实际IP'
  serial_port: /dev/serial/by-id/左夹爪设备名
right:
  robot_ip: '右臂实际IP'
  serial_port: /dev/serial/by-id/右夹爪设备名
gripper:
  baud_rate: 1000000
  timeout: 1000
  slave_address: 1
  gripper_model: TG-9801
  position_open_register: 100
  position_closed_register: 0
  position_mode_speed_register: 300
  target_force_percent: 20
```

在完成所有机械、夹爪和安全检查之前，保持：

```yaml
commissioned: false
```

## 5. 基座安装位姿标定

`arms.yaml` 中的左右基座 `xyz / rpy` 必须是实测值，不能使用示例值。

### 5.1 采集基座对应点

准备至少 4 个不共线的已知点，分别记录：

- 在 `world` 坐标系下的实际位置
- 在机械臂基座坐标系下对应的位置

### 5.2 生成安装变换

把记录写成 `survey.yaml`：

```yaml
units: metres
source_points:
  - [x1, y1, z1]
  - [x2, y2, z2]
  - [x3, y3, z3]
  - [x4, y4, z4]
target_points:
  - [X1, Y1, Z1]
  - [X2, Y2, Z2]
  - [X3, Y3, Z3]
  - [X4, Y4, Z4]
```

执行：

```bash
python3 scripts/calibrate_geometry.py registration \
  survey.yaml \
  --max-error 0.001 \
  --output base-fit.yaml
```

检查残差。若 `source` 是基座、`target` 是 `world`，输出 `xyz/rpy` 即写入 `arms.yaml` 对应侧。

## 6. TCP 标定

TCP 和法兰偏移必须以实测为准。

### 6.1 四点法或 pivot 法

让工具尖端接触同一个固定点，安全调整多个姿态，记录每次真实反馈位姿：

```text
world_from_tool
```

写成 `pivot.yaml`：

```yaml
units: metres
world_from_tool:
  - [[...], [...], [...], [...]]
  - [[...], [...], [...], [...]]
```

执行：

```bash
python3 scripts/calibrate_geometry.py pivot \
  pivot.yaml \
  --max-error 0.001 \
  --output tcp-fit.yaml
```

### 6.2 写入 arms.yaml

确认 `gripper.tcp_xyz` 的父坐标系。当前模型是 `gripper_palm`。

如果采集的是控制柜法兰坐标，先应用已测法兰到 `gripper_palm` 的变换，再填写：

```yaml
gripper:
  tcp_xyz: [实测x, 实测y, 实测z]
  tcp_rpy: [实测r, 实测p, 实测y]
```

左右夹爪不同时，必须分别标定，不能共用同一个 TCP。

## 7. 夹爪开闭端点和接触阈值

编辑：

```text
/absolute/fr3-config/real_feedback.yaml
```

需要现场记录：

```yaml
left:
  finger_open: 实测单指张开位移
  finger_closed: 实测单指闭合位移
  finger1_zero: [X, Y, Z]
  finger2_zero: [X, Y, Z]
  contact_min: 实测接触阈值下限
  contact_max: 实测接触阈值上限
  held_gap_min: 夹持时最小开度
  held_gap_max: 夹持时最大开度
right:
  finger_open: 实测单指张开位移
  finger_closed: 实测单指闭合位移
  finger1_zero: [X, Y, Z]
  finger2_zero: [X, Y, Z]
  contact_min: 实测接触阈值下限
  contact_max: 实测接触阈值上限
  held_gap_min: 夹持时最小开度
  held_gap_max: 夹持时最大开度
```

注意：

- `finger_open/closed` 是 URDF 单指位移，不是 0-100 寄存器值。
- `open_width / 2` 必须等于 `finger_open`。
- 接触阈值是去零后的原始寄存器范数，不是牛顿。
- 夹持开度按实际持物时的记录标定。

## 8. 桌子和相机

### 8.1 scene.yaml

实测并填写：

- 桌面高度 `table.top_z`
- 桌面中心 `table.center_xy`
- 桌面尺寸 `table.size_xy`
- 螺栓位置和尺寸
- 相机安装位置

### 8.2 inspection.yaml

确认：

- `table_z` 与 `scene.yaml` 完全一致
- `roi_min/roi_max` 覆盖单颗螺栓
- `inspection_center` 和 `handover_center` 经过可达性验证
- 相机话题映射到实际 RealSense 驱动

### 8.3 real_feedback.yaml 的话题映射

如果实际相机话题不同，使用：

```yaml
topic_remappings:
  /head_camera/points: /actual/points
  /waist_camera/image_raw: /actual/waist/image
```

## 9. 可选：ArUco 测试件跟踪

如果要使用刚性带标记测试件，填写：

```text
/absolute/fr3-config/tracker.yaml
```

启动：

```bash
ros2 run fr3_bolt_inspection_cell aruco_object_tracker \
  --ros-args --params-file /absolute/fr3-config/tracker.yaml
```

## 10. 离线校验配置

```bash
python3 scripts/check_deployment.py /absolute/fr3-config
```

输出应类似：

```text
Configuration and model valid: 12 robot axes, 2 gripper command joints, 0 Gazebo plugins.
```

注意：这只是文件级检查，不是标定精度认证。

## 11. 只读反馈启动

```bash
bash scripts/start_real.sh /absolute/fr3-config
```

注意：即使不带 `--execute`，驱动也可能发送保持指令，不是完全只读。

另开终端：

```bash
ros2 control list_controllers -c /left_controller_manager
ros2 control list_controllers -c /right_controller_manager
ros2 control list_hardware_interfaces -c /left_controller_manager
ros2 control list_hardware_interfaces -c /right_controller_manager
ros2 topic echo /joint_states --once
```

确认：

- 左右各 6 个 FR3 关节持续更新
- 左右各 1 个 HKV 主关节持续更新
- RViz 姿态与实物一致
- 没有重复 TF、旧 move_group 或旧 controller_manager

## 12. 空载低速运动验收

退出上一次启动后：

```bash
bash scripts/start_real.sh /absolute/fr3-config --execute
```

按顺序：

1. 左臂单关节小角度运动
2. 右臂单关节小角度运动
3. 左夹爪空开
4. 左夹爪空闭
5. 右夹爪空开
6. 右夹爪空闭
7. RViz 单臂 `Plan -> Execute`
8. 双臂联合规划，但不执行危险轨迹

任何异常立即使用物理急停。

## 13. 最后把 commissioned 改为 true

只有以下条件全部满足后：

- 版本确认为 3.9.7
- IP 和串口正确
- 基座、TCP、夹爪端点已实测
- 空载低速运动正常
- 急停有效
- 工作区域清空

再把：

```yaml
commissioned: true
```

写入 `hardware.yaml`。

## 14. 开始自动任务

```bash
bash scripts/start_real.sh /absolute/fr3-config --execute
```

启动任务：

```bash
ros2 service call /inspection/start std_srvs/srv/Trigger '{}'
```

停止：

```bash
ros2 service call /inspection/stop std_srvs/srv/Trigger '{}'
```

自动任务和单臂示教 UI 不能同时运行。

