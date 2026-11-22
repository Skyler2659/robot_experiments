# PiPER 单臂接入示例

这一步把同步 RGB 和单臂反馈写成 CodeWorld 已有的观测包，并把基座系下的抓取中心目标换算法兰目标。离线部分使用真机包的 Python 环境。示例按一台标准版 PiPER、平行夹爪和水平安装的基座编写。

## 实验室已有采集和部署程序时

优先复用现场已经成功运行的采集与动作调用。下面这段可以直接发给负责设备的同学。

> 学长好，我在接 CodeWorld。它需要读当前图像和机械臂状态，算出下一步动作，执行后再看一帧。方便发我你们平时跑通的最小部署脚本和启动命令吗？我主要想看取观测、发一个动作、判断动作完成的那几行。如果仓库暂时不方便发，一个调用示例或一次输入输出也可以。再麻烦给我一小段现成录制，能对应一次动作前后的相机图像、机械臂状态、动作和时间最好。相机内外参、末端工具配置、环境版本如果已有，告诉我在哪个文件就行。原格式发我就好，我来做转换。

先用原样例回答三个问题，再写我们这边的适配。

| 要确认的事 | 对接时的处理 |
|---|---|
| 部署程序怎样调用策略？它能否在动作之间等待模型计算？ | 找到现有示例中产生动作的位置。如果是固定频率连续取动作的控制循环，沿用现场的保持／等待和超时机制，把 CodeWorld 推理放到循环之外。 |
| 一次观测包含哪些图、状态和时间？标定以哪个坐标系为参照？ | 列出相机名、RGB/BGR、内参对应的分辨率、外参方向、关节或末端状态以及时间字段。将现场数据在我们这边转成观测包。腕相机额外需要曝光时的法兰姿态。 |
| 一次动作具体接受什么，什么时候算执行完？ | 从一条可运行调用确认绝对／增量、关节／笛卡尔、基座／工具坐标、姿态排列、位置和角度单位、夹爪语义，及是否异步。接口收 TCP 就按其约定发 TCP；收法兰才使用工具偏移；只收关节量就确认现有 IK 和轨迹入口。动作结束后读取新状态和图像。 |

下面的一条命令在仓库根目录回放前后两次观测，读回并校验 CodeWorld 观测包，同时预览一个法兰目标。它打印本次输出清单的位置。拿到现场录制后，先将原格式映射为我们内部的清单，再替换命令中的输入路径。`--flange-preview` 可省略。

```bash
.venv/bin/python -m real_robot.piper replay real_robot/data/examples/replay/before.json real_robot/data/examples/replay/after.json --flange-preview real_robot/data/examples/replay/tcp_target.json
```

第一轮现场联调可选装箱的一次抓放。用现有部署程序取一帧，核对 CodeWorld 给出的目标，把它交给现场的动作接口，等执行完成后再取一帧。通过记录核对目标、实际末端状态、物品新位置，以及下一步决策是否用到了这次更新。现场接口映射、PiPER 本体模型和在线 Agent 接入仍需要结合实际配置完成。

## 在电脑上回放一次装箱

在 Embodied2Code 仓库根目录执行以下命令。已激活项目虚拟环境时也可以使用 `python`。

```bash
.venv/bin/python -m real_robot.piper capture real_robot/data/examples/replay/before.json runs/piper-offline
.venv/bin/python -m real_robot.piper capture real_robot/data/examples/replay/after.json runs/piper-offline
.venv/bin/python -m real_robot.piper preview real_robot/data/examples/replay/tcp_target.json
```

前两条分别打印本次写入的 `capture.json` 路径。打开同目录的 `rgb.png`（固定相机）和 `rgb_1.png`（腕相机），可以看到物品从盒外移到盒内。每次采集单独存放，可用 `codeworld.observations.load(root, capture)` 读回。这里的图像、机器人状态和安装尺寸都是合成示例，两组图表现的是预先给定的前后状态。

第三条只做目标坐标换算。示例中夹爪朝下，目标抓取中心的高度为 0.12 m，沿工具轴从法兰到两指中心的示例偏移是 0.10 m，因此法兰目标高度为 0.22 m。这个数值适合用来检查变换方向。真机需要使用实测工具偏移。

## 以后怎样换成现场数据

| 数据 | 现场从哪里取 | 对应示例字段 |
|---|---|---|
| 图像、内参和尺寸 | 保存去畸变后的 RGB，并使用对应输出分辨率和裁切方式的内参 | 每个视角的 `rgb`、`K`、`image_size` |
| 固定相机外参 | 标定相机光学系到基座系的刚体变换 | `T_base_camera` |
| 腕相机外参 | 手眼标定给出光学系到法兰系的变换，在每次曝光时取得法兰在基座系下的位姿 | `T_flange_camera`、`T_base_flange_at_exposure` |
| 机器人状态 | SDK 反馈给出六个关节角、夹爪宽度和法兰位姿 | `robot_state.q_rad`、`gripper_width_m`、`T_base_flange` |
| 时间 | 采集端将相机与机械臂时钟对齐，按同一时基匹配各路数据，运动中的腕相机应按曝光时刻插值 | `clock`、每个视角的 `exposure_ns`、状态的 `timestamp_ns` |

`T_A_B` 把 B 系中的点变换到 A 系，平移单位是米，光学轴使用 OpenCV 约定（右、下、前）。腕相机使用 `T_base_flange_at_exposure @ T_flange_camera`。录制端负责同步，导入器使用清单中已对齐的位姿。替换图片时将路径写成相对清单的位置，并将 `source` 设成 `recorded`。只有固定相机时，`views` 保留固定视角一项即可。

## PiPER 反馈从哪儿接

接上实机后，先在控制机上查明固件和 SDK 协议版本。下面的读取函数按松灵 Python SDK **V2 接口**编写，接受已经连接的接口对象，读取关节、法兰和夹爪三类反馈。实际固件若使用其他协议，需要核对对应版本的字段与单位。[松灵 SDK](https://github.com/agilexrobotics/piper_sdk) · [官方 TCP 计算示例](https://github.com/agilexrobotics/piper_sdk/blob/master/piper_sdk/demo/V2/piper_read_tcp_pose.py)

```python
from real_robot.piper import read_v2_feedback

# arm 由控制机的 SDK 连接代码创建，三种反馈开始发布后调用。
state = read_v2_feedback(arm)
```

返回值包含六个弧度制关节角、米制夹爪开度、法兰在基座系下的位姿 `T_base_flange`，并保留三个独立的 SDK 消息时间戳。建议第一组真实数据在机械臂静止时采集。采集端要检查消息是否持续更新，将时间映射到相机使用的时钟后再匹配曝光。SDK 示例中的工具偏移和本目录的 0.10 m 都应换成现场值。

## 和现有实验代码怎样衔接

1. **建世界。** 初始观测取 `import_capture(manifest, root)` 的返回值。把 `root`、返回值里的相对路径 `packet["capture"]`、按该帧关节状态生成的 PiPER 本体 GLB 和重建配置传给 `codeworld.reconstruction.reconstruct`。在配置中显式选择实际 `view_ids`。当前多视角重建提示提及双腕，接入实验前需改为实际相机列表。导入阶段可以独立于重建运行。
2. **接运动。** Agent 在世界系计算指尖目标，在线后端调用本示例的 `flange_target` 完成 TCP 换算，再交给 PiPER 轨迹规划和执行接口。实际执行前需接入匹配实机的关节限位、碰撞检查和停止控制。动作返回时记录实际末端位姿、夹爪开度和执行状态。
3. **接动作后更新。** 重新采一组真实观测，交给现有的 `world.read_context()` 和 `world.commit_step(...)` 流程。真机入口 `real_robot.scene.prepare` 从现场配置读取机械臂模型、桌高和固定障碍物。完整任务执行与评分留到现场验证。

本目录的验证覆盖图像回放、外参组合、SDK V2 字段与单位换算，以及目标法兰位姿换算。相机采集、CAN 通信和带载运动需在相应硬件上验证。
