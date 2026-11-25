# 双臂 PiPER 工作站接入

在实验室使用现场运行 `deploy/client.py` 的 PiPER 版 LeRobot Python 环境。真机包按相同的注册方式创建 `bi_piper_follower`，调用 `connect()`、`get_observation()`、`send_action()` 和 `disconnect()`。模型决策由本包启动，直接复用机器人端驱动。站点专用函数放在工作站上可导入的 Python 模块里，将模块名写入 `driver.site_module`。

从仓库根目录执行下面两条命令，第一条从驱动对象列出电机键和相机名；第二条连接后只读一条原始观测并断开。两条命令均可在填写几何标定之前使用，不会调用 `send_action()`。

```bash
python -m real_robot driver-info --config real_robot/data/configs/lab.example.json
python -m real_robot driver-info --config real_robot/data/configs/lab.example.json --sample
```

- **电机映射**。在现场配置的 `driver.motor_keys.left` 和 `right` 各填七个真实键名，每侧依次对应 URDF 的 joint1 至 joint6 和夹爪总开度。写入动作时按驱动自己的 `robot.action_features` 顺序组回完整字典。机械臂连接选项取工作站驱动的默认值，定制的端口等参数可在 `driver.robot_options` 中按该驱动的配置字段填写。
- **单位**。`state_scale` 和 `action_scale` 分别表示每个驱动单位对应的 SI 数值，两侧共用七个系数。观测读数乘 `state_scale` 得到弧度和米，SI 指令除以 `action_scale` 后发送。模板沿用部署示例的 `home.json` 的观测换算（关节为千分之一度，夹爪为微米），并按该示例输出的弧度和米设置动作系数为 1。先核对工作站当前版本、左右臂键名和单位，再下发动作。
- **同步采集**。一次 `get_observation()` 提供这一批三路 RGB 和左右臂电机读数。`read_state()` 复用同一批采样。若驱动已随观测提供 `obs["_embodied2code"]`，直接读取；否则由站点函数补入同一批数据对应的曝光与反馈元数据。时间戳使用宿主单调时钟纳秒。超过配置容差或动作完成前曝光的帧不能用于该次状态更新。
- **动作与反馈**。动作适配器将笛卡尔移动交给现场规划函数，按返回的关节路点逐个下发。启用几何碰撞检查时，把实测起点、活动臂路点和另一臂的保持目标一起提交检查。夹爪动作直接生成电机目标。每次发送后重新读取关节和夹爪反馈判定到位；默认关节容差 0.01 rad、夹爪容差 0.002 m、单次动作等待 10 s，可在 `driver` 中用 `joint_tolerance_rad`、`width_tolerance_m`、`motion_timeout_s` 调整。底层读写也应使用工作站驱动自身的超时机制。

### 工作站专用函数

| 函数 | 输入与返回 |
|---|---|
| `observation_metadata(robot, obs)` | 返回 `{"exposure_ns": {原始相机名: 曝光时间}, "arms": {"left": {...}, "right": {...}}}`。每臂含该次反馈的 `timestamp_ns` 和本臂基座下的 4×4 `T_base_flange`。若标准 PiPER 的关节零点、轴向和法兰与随包 URDF 一致，可设 `driver.flange_fk` 为 `"piper_urdf"` 并省去法兰矩阵，程序将按同次关节反馈计算 link6 位姿。仍需提供真实时间戳。 |
| `rectify_image(camera_name, rgb)` | 当驱动返回的 RGB 需要去畸变时，实现此函数。输出为与标定内参一致的 H×W×3 uint8 RGB。若驱动已输出校正后的图像可省略。 |
| `plan_move(robot, arm, T_base_flange, measured)` | 调用现场 IK／轨迹规划器，返回所选臂按执行顺序排列的 N×6 绝对关节角（弧度）。`measured[arm]` 中前六项为关节角，最后一项为夹爪开度（米）。同一规划交给路径检查和下发；空列表表示受阻。 |
| `stop(robot)` | 调用现场控制器打断两臂正在执行的命令。须能从另一线程调用并可重复调用。配置此函数后才可下发动作。 |
| `grip_contact(robot, arm, target_width_m, measured)` | 可选。闭合中夹住物体而提前停住时，根据控制器实际接触／夹持反馈返回布尔值。省略时以实测开度到位为准。 |

工作站提供当前驱动源码或带时间戳的录制后，可以继续离线对接专用函数。相机外参、两臂基座位置、工具变换、真实夹持反馈和停止行为需要现场核验。可参阅 [PiPER 工具说明](../real_robot/PIPER.zh-CN.md) 中的 SDK V2 反馈和录制导入工具。

## 填写现场配置

模板沿用实验室部署包 `deploy/configs/pi05.json` 的 `bi_piper_follower`、`bi_piper` 和三路相机。三个任务均可调用左右两条从臂。世界坐标系 W 取右臂基座，左臂目标按实测的两基座变换换算。现场核对 `left_follower` 和 `right_follower` 的 CAN 映射。插管、装箱、搭塔通过 `--task insert|pack|tower` 选择，现场道具和目标可用 `--instruction` 描述。

| 视角 | 驱动层采集设置（沿用学长配置） | CodeWorld 使用的标定 |
|---|---|---|
| `top` | `/dev/top`，640 × 480，旋转 0°，MJPG，配置帧率 120 | 顶视相机相对右臂基座的变换和输出图像内参。模板按 640 × 480 填输出尺寸，收到样帧后核对。 |
| `l_wrist` | `/dev/l_wrist`，配置宽 480、高 640，旋转 −90°，MJPG，配置帧率 120 | 填左腕相机相对左法兰的变换及旋转后的图像标定。 |
| `r_wrist` | `/dev/r_wrist`，配置宽 480、高 640，旋转 +90°，MJPG，配置帧率 120 | 腕相机相对右臂法兰的变换。按驱动最终返回的去畸变、已旋转 RGB 填写 `image_size` 和对应的 `K`；外参须对应同一光学坐标系。 |

`driver.cameras` 是传给机器人端驱动的采集配置。顶层 `cameras` 保存最终送入 CodeWorld 的视图标定。

无硬件时可从仓库根目录运行 `python -m pip install -e ".[test]"`，再执行 `python -m pytest -q tests/test_lab_adapter.py tests/test_real_robot.py` 检查设备适配器和双臂三视角接口。`python -m real_robot demo` 可复现原单臂录制。现场配置保留需要测量的空值，回放数据和标定只用于合成示例。拿到现场数据后，从仓库根目录复制模板：

```bash
cp real_robot/data/configs/lab.example.json real_robot/data/configs/lab.local.json
```

配置文件中的相对路径从它所在目录解析。

- 在 `arms.left.T_W_base` 填左基座相对右基座的实测变换。右基座的 `T_W_base` 为单位矩阵。分别填两臂的 `T_flange_tcp` 和最大夹爪开度。策略在 W 系发出目标，如 `robo.move_to(xyz, arm="left")`、`robo.set_gripper(width_m, arm="right")`。每步明确选择一臂，另一臂可以保持抓握。
- 顶相机外参填 `T_W_camera`。每个腕相机用 `arm` 指定所属臂，填 `T_flange_camera`。三路各填对应的 `K` 和实际 `image_size`。采集端按曝光时刻对齐双臂反馈和三路图像，同步容差按现场采集系统填写。
- 模板按左右臂各绑定一份标准 PiPER 的 URDF、网格和关节。两份模型和腕相机使用 `arms` 中同一组基座标定，每个夹爪用本臂实测开度驱动两个指节。
- `reconstruction.runtime` 填原有 RCWM 渲染运行时的路径。可以复用服务器上已有的运行时，也可在 Linux 控制／计算机执行下列命令生成一个。重建还需要可用的 Codex CLI 和模型访问。即使动作决策使用 OpenAI 兼容服务，当前初始重建仍走原项目的 Codex 重建流程。可选的 `table_height_m` 填基座系中的实测桌高，`fixed_obstacles` 可描述已测固定支架。
- 通过现场部署栈执行规划及安全控制。若还要启用原项目的 CodeWorld 网格碰撞检查，安装 `python -m pip install -e '.[collision]'`，将 `real_robot/data/configs/piper_collision.json` 的 `collision` 段复制到同目录的现场配置。碰撞模型从两臂标定读取基座，使用该片段时需核对夹爪行程与手指缩放。执行适配器拿到 `check_path` 回调时，应在运动开始前提交控制器实际将执行的轨迹，格式为 `[{"joints_rad":[...], "width_m":...}, ...]`。返回 `status == "CLEAR"` 才交给驱动，其他结果作为 `blocked` 返回。模板的默认动作路径由现场控制器负责，额外的网格检查需要控制器能够导出规划轨迹。

```bash
python -m codeworld.cli setup-runtime "$PWD/runtime"
```

### 腕部标定、运动能力与动作反馈

可以在工作站配置里设置 `camera_calibration_file`，指向相对该配置文件的 JSON。顶层键是发布给策略的相机名，每个条目的字段与 `cameras` 相同，按字段覆盖。右腕用实测的 `K`、`image_size`、`T_flange_camera`，并设置 `mount="wrist"`、`arm="right"`。在 `driver.camera_map` 中将实际设备名映射到 `r_wrist`。确认 capture.json 的 views 同时列出 top、r_wrist，run 的任务卡会列出同一组标定视图。若传入原始畸变图像，额外提供 OpenCV 针孔模型的 `raw_K` 和 `raw_distortion`，校正后的内参填 `K`。现场已有 `rectify_image` 时由现场完成校正。

可实现 `site.read_state_at(robot, exposure_ns)` 从时间缓冲区返回以臂名为键的 ArmState，时间戳、单位与现有 read_state 一致。未配置时使用同一观测包的电机采样，并核对和曝光的时间差。腕相机外参由该次采样的世界法兰位姿乘以手眼外参得到。双视角几何拟合需要同一物理标志点在两个已标定视图中可见。

普通 `move_to` 延用原有规划器。启用现场 FK、规划器和 CodeWorld 路径检查后，`move_to(..., path="line")` 和 `retreat(delta_xyz, arm=...)` 检查候选关节路径各采样点的 TCP 是否沿所请求线段前进；`home(arm=...)` 要求回到该臂回合开始时的实测关节角，维持夹爪目标。两种路径均先检查整段规划。可选现场钩子 `plan_line(robot, arm, T_base_flange, measured, T_flange_tcp)` 和 `plan_home(robot, arm, initial_joints_rad, measured)` 返回与 plan_move 相同的 N×6 绝对关节路点。若仅有 plan_move，则以其路点作为候选并验证上述约束。检查未通过时返回 blocked，零下发。路径采样的关节插值应与现场控制器执行语义一致。

查看每轮工具反馈中的 `physical_action`，核对实际 TCP、开度、碰撞对、测量步号和 pending_update。普通提前结束进入恢复审查；设备停止或反馈丢失时结束运动，并保留最后一次确认的回执。使用 `python -m real_robot demo` 可离线验证标定观测、控制接口和世界更新。
