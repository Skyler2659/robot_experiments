# Embodied2Code 真机实验

在连接 PiPER 双臂和三路相机的 Linux 工作站上运行。一次 `run` 会采集顶视和左右腕图像，重建 CodeWorld，然后自动启动策略控制机械臂。

## 首次配置工作站

激活实验室已能运行双臂 PiPER 的 LeRobot 环境（Python 3.10–3.12），在仓库根目录执行：

```bash
python -m pip install -e .
cp real_robot/data/configs/lab.example.json real_robot/data/configs/lab.local.json
python -m codeworld.cli setup-runtime "$PWD/runtime"
```

如已有重建运行时，可直接在 `lab.local.json` 的 `reconstruction.runtime` 填其绝对路径，跳过最后一条命令。现场需要填入双臂电机映射、基座及工具标定、三路相机内外参，并接好时间戳、运动规划和停止接口。具体字段见[双臂工作站接入说明](docs/lab.md)。

## 接入模型

初始重建会调用工作站上的 Codex CLI。若尚未安装，可运行 `npm install -g @openai/codex`。用以下任一种方式登录，重建程序会从 Codex 的 `auth.json` 读取凭据。

**用 ChatGPT 账号登录**

```bash
codex -c 'cli_auth_credentials_store="file"' login
```

**用 OpenAI API Key 登录**（在 Bash 里输入，密钥不会回显）

```bash
read -rsp 'OpenAI API Key: ' OPENAI_API_KEY
echo
export OPENAI_API_KEY
printenv OPENAI_API_KEY | codex -c 'cli_auth_credentials_store="file"' login --with-api-key
```

## 输入指令并启动策略

首次联调先采一组图像和双臂状态，核对相机视角和姿态：

```bash
CFG=real_robot/data/configs/lab.local.json
python -m real_robot capture --config "$CFG"
```

在同一终端执行下面的命令。把 `--instruction` 引号里的文字换成本轮目标。重建成功后，策略会直接向现场驱动发送动作。

```bash
python -m real_robot run --config "$CFG" --task pack \
  --instruction '把桌上的红色方块放进盒子里' --output runs/pack-001
```

`--task` 可选 `insert`（插管）、`pack`（装箱）或 `tower`（搭塔）。省略 `--instruction` 会使用该任务的预设指令。动作决策默认通过 Codex CLI 使用 `gpt-6-astra`，换模型时加 `--model 模型名`。

**让动作策略直接调用兼容 API** 时，在同一条 `run` 命令末尾追加以下参数。先将服务密钥设为环境变量 `POLICY_API_KEY`，并将地址和模型名换成服务实际提供的值。初始重建继续使用上一步登录的 Codex CLI，重建模型在 `lab.local.json` 的 `reconstruction.model` 设置。

```text
--runtime openai --base-url https://你的服务地址/v1 --api-key-env POLICY_API_KEY --model 模型名
```

## 看进度和结果

运行期间，在工作站的第二个终端进入仓库根目录，执行 `watch -n 2 cat runs/pack-001/reconstruction/reconstruction.json` 查看重建状态。每轮使用新的输出目录，也可省略 `--output` 让程序自动命名。

| 每轮目录中的文件 | 内容 |
|---|---|
| `world/` | 重建的 CodeWorld 资产、观测与状态更新 |
| `videos/top.mp4`、`videos/l_wrist.mp4`、`videos/r_wrist.mp4` | 从采集首帧到回合结束的三路录像 |
| `result.json` | 策略结束报告及录像状态 |

没有机械臂时，可在独立的 Python 3.10–3.12 环境安装本包后运行 `python -m real_robot demo`。它用随包的单臂录制回放装箱流程，无需登录模型。
