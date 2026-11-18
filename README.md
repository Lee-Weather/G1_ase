# G1 行走运动控制：ASE LLC 与 HLC 速度控制

面向**宇树 G1（Unitree G1，29 自由度）**的仿真运动控制项目。基于 ASE 学习可复用的底层技能控制器（LLC），再冻结 LLC、训练高层控制器（HLC），实现目标速度下的起步、行走、调速与停止，并在 Isaac Gym 和 MuJoCo 中验证。

当前任务沿固定世界 +X 方向控制速度，训练指令范围 0～2 m/s。**本仓库已从上一代 F1 机型整体移植到 G1**：资产、任务、配置、训练与回放入口只保留 G1。当前工作范围是仿真训练与回放；真机部署、实体手柄、任意方向转向和稳定跑步尚未验收。

## 当前进展

截至本机移植完成（G1 侧静态与 MuJoCo 验证通过）：

| 项目 | 状态 |
|---|---|
| G1 29-DoF 资产（MJCF/URDF/mesh） | 完成，MuJoCo 3.15 编译通过（nq=36, nv=35, nu=29, nbody=31） |
| LLC / HLC 任务、配置、训练与回放代码 | 完成，观测维度与上一代一致（actor 93 / critic 448 / AMP 890） |
| 静态一致性自检 | 通过：`python ase/tests/test_g1_consistency.py` |
| MuJoCo 冒烟（资产、姿态、增益、动作映射） | 通过：`czy/g1_wbt/scripts/check_g1_mujoco.py` |
| G1 参考动作数据集 | 完成：由 UMR 的 G1 重定向结果转出 49 条 / 13,828 帧 / 4.6 分钟 |
| G1 的 LLC 训练 | 已跑通：201.5 上 512 env × 10 iter 冒烟通过（2.3 s/iter，检查点正常写出） |
| G1 的 HLC 训练与回放 | **尚未运行**，还在等一次正式 LLC 训练产出可冻结的权重 |

参考动作已就绪（来源 ACCAD → UMR 重定向到 G1 → 本仓库转换），格式与逐条校验见
[ase/data/motions/g1/README.md](ase/data/motions/g1/README.md) 与同目录的 `_validation_g1.json`。

## 控制架构

```mermaid
flowchart LR
    C[目标方向与速度] --> H[HLC：10Hz]
    S[机器人状态] --> H
    H --> Z[32维单位latent z]
    Z --> L[冻结的ASE LLC：50Hz]
    S --> L
    L --> A[29维关节动作]
    A --> P[关节目标映射与PD控制]
    P --> R[G1仿真]
    R --> S
```

| 模块 | 接口与训练方式 |
|---|---|
| ASE LLC | actor 为 93 维状态＋32 维 z，输出 29 维关节动作；critic 为 448 维；AMP 判别器/encoder 使用 10 帧×89 维历史，共 890 维 |
| HLC | actor 为 93 维状态＋2 维局部目标方向＋1 维目标速度，共 96 维；critic 为 451 维；输出归一化的 32 维 z |
| 控制频率 | LLC 50Hz（`sim.dt` 0.01 × `controlFrequencyInv` 2）；每 5 个 LLC 步更新一次 z，HLC 10Hz |
| 冻结依赖 | HLC 加载 LLC actor、判别器及其归一化统计，保持冻结；HLC 使用自己的 actor/critic 和优化器 |

维度为什么和上一代相同：G1 的 29 个转动关节各自驱动 1 个刚体，`pelvis` 之后正好 29 个连杆（`dof_offsets = range(30)`），critic 用到 30 个刚体即 1+15×30−3 = 448，6 个 key body 让 AMP 每步为 13+29+29+3×6 = 89。改动 keyBodies 数量会同时改变 AMP 维度并让旧权重失效。

当前 HLC actor 没有显式的根部实际线速度或速度误差输入；真实线速度用于 critic 和奖励。actor 仍通过角速度、重力投影、关节状态及上一动作获得反馈。

动作采用 `joint_limits` 分方向映射：零动作对应默认站姿，正负方向分别覆盖关节限位并保留 10% 行程 PD 余量。旧 checkpoint 按其保存的控制配置回放。KP/KD、armature 等由环境配置和模型元数据追溯。

## G1 机器人参数

| 项目 | 取值与来源 |
|---|---|
| 自由度 | 29：左右腿各 6、腰 3、左右臂各 7（`g1/mjcf/g1_29dof.xml` 文档顺序） |
| 质量与身高 | 33.34 kg（MuJoCo 子树质量）；默认姿态骨盆高 0.7762 m |
| 关节限位/力矩 | URDF `g1_29dof.urdf`：髋 pitch/yaw 88、髋 roll 139、膝 139、踝 50、腰 yaw 88、腰 roll/pitch 50、肩/肘/腕 roll 25、腕 pitch/yaw 5 N·m |
| PD 增益 | 髋 100/2、膝 150/4、踝 40/2、腰 200/5、臂 40/10（G1 官方部署 velocity 策略同值，50Hz 控制） |
| 默认姿态 | 髋 −0.240、膝 0.440、踝 −0.200，肩 pitch 0.300、肩 roll ±0.250、肘 0.970、腕 roll ±0.150；其余为 0 |
| 默认姿态的选取依据 | 在“脚掌保持水平（髋+膝+踝=0）”约束下搜索，使全身质心落在踝关节轴正上方（实测偏移 0.0000 m），同时保留中等屈膝的力矩余量 |
| armature | 0.01（与上一代一致的估计值；官方模型的逐关节值 0.0036～0.0251 留待后续按需启用） |
| 终止阈值 | 骨盆高 0.40 m、倾角 1.0 rad、非允许着地体接触高度 0.25 m、接触力 1.0 N |
| 初始高度 | 0.78 m（默认姿态 0.7762 m 加少量下落余量） |

初始高度、终止高度与默认姿态之间是相互约束的：改默认姿态后必须重新用 `g1_forward_kinematics.py` 算姿态高度，并用 `check_g1_mujoco.py` 确认接触时刻与阈值仍然成立。

## 目录与配置

以下路径相对本训练仓库根目录。

| 路径 | 内容 |
|---|---|
| [ase/train_g1_cloud.py](ase/train_g1_cloud.py) | G1 ASE/AMP/HRL 统一训练入口、依赖准备、恢复训练与日志 |
| [ase/env/tasks/humanoid_g1.py](ase/env/tasks/humanoid_g1.py) | G1 模型、状态、动作映射与终止条件 |
| [ase/env/tasks/humanoid_g1_hlc.py](ase/env/tasks/humanoid_g1_hlc.py) | HLC 速度指令与站走停任务奖励 |
| [ase/utils/g1_hlc.py](ase/utils/g1_hlc.py) | 冻结 LLC、HLC 网络和高低层控制间隔 |
| [ase/utils/g1_control.py](ase/utils/g1_control.py) | 训练与两个引擎共用的动作映射 |
| [ase/utils/g1_motion_lib.py](ase/utils/g1_motion_lib.py) | G1 tracking NPZ 动作加载与按名重排 |
| [ase/data/cfg/humanoid_g1.yaml](ase/data/cfg/humanoid_g1.yaml) | LLC 环境、默认姿态、PD 及物理参数 |
| [ase/data/cfg/humanoid_g1_hlc.yaml](ase/data/cfg/humanoid_g1_hlc.yaml) | HLC 基础配置（0.6～1.6 m/s 历史任务） |
| [ase/data/cfg/humanoid_g1_hlc_0_2.yaml](ase/data/cfg/humanoid_g1_hlc_0_2.yaml) | 当前 0～2 m/s、20% 精确零速的 HLC 环境 |
| [ase/data/cfg/train/rlg/ase_g1.yaml](ase/data/cfg/train/rlg/ase_g1.yaml) | ASE LLC 训练参数 |
| [ase/data/cfg/train/rlg/hrl_g1.yaml](ase/data/cfg/train/rlg/hrl_g1.yaml) | HLC PPO 训练参数 |
| [ase/replay_g1.py](ase/replay_g1.py) | LLC 双引擎策略回放 |
| [ase/replay_g1_hlc.py](ase/replay_g1_hlc.py) | HLC 双引擎调速/站走停回放及 NumPy 权重导出 |
| [ase/evaluate_g1_coverage.py](ase/evaluate_g1_coverage.py) | 随机 z、参考编码 z 及参考技能覆盖诊断 |
| [ase/evaluate_g1_transitions.py](ase/evaluate_g1_transitions.py) | 连续走→站立 z 切换诊断 |
| [ase/tests/test_g1_consistency.py](ase/tests/test_g1_consistency.py) | 资产/配置/维度静态一致性自检（不需要 Isaac Gym） |
| [ase/data/motions/g1/README.md](ase/data/motions/g1/README.md) | 参考动作来源、权重分配、NPZ 契约与重建方式 |

资产目录 `ase/data/assets/g1/`：`mjcf/g1_29dof.xml`（机器人，35 个 STL，29 个力矩电机）、`urdf/g1_29dof.urdf`（关节限位与连杆名来源）、`meshes/g1/`。

模型来源与生成方式见 `czy/g1_wbt/scripts/build_g1_mjcf.py`：取宇树官方 MuJoCo 描述，去掉 floor/skybox/sensor 等场景内容，把 mesh 路径改写成相对 MJCF 目录（`../meshes/g1/*.STL`，Isaac Gym 与 MuJoCo 回放都依赖这个约定），并按 URDF 力矩上限补 29 个 `motor`。上一代 F1 的资产与动作数据已移出训练仓库，归档在 `czy/archive_f1/`。

实验记录与模型产物在父工作区的 `czy/` 中维护，不随训练 Git 仓库分发。G1 目前还没有实验记录目录。

## 环境准备

训练与 Gym 回放使用 Linux、NVIDIA GPU 和 Isaac Gym Preview 4。201.5 已验证环境为 Python 3.8.20、PyTorch 1.12.1+cu113；云端训练也使用过自带 PyTorch 2.4.1 的镜像。复用匹配 Isaac Gym 的现有环境，保留镜像自带的 PyTorch/CUDA。

训练入口的 `--install-deps` 按 `PACKAGES` 安装配套依赖，包括 NumPy 1.23.5、rl-games 1.1.4、gym 0.23.1、ray 1.13.0 等；依赖目录通过 `G1_DEPS_DIR` 设置，默认位于临时目录的 `g1_ase_deps`。根目录 `requirements.txt` 保留了上游旧版本列表，当前训练以入口的依赖配置为准。

201.5 复用已有依赖的示例（在项目根目录执行）：

```bash
conda activate F1
export G1_DEPS_DIR=/home/robot/czy/g1_ase/.deps
export PYTHONPATH="$G1_DEPS_DIR:ase:${PYTHONPATH:-}"
```

MuJoCo 回放可独立于 Isaac Gym 运行。本机已验证 MuJoCo 3.15.0、NumPy 1.26.4（`C:/Users/cuizeyu/AppData/Local/Programs/Python/Python311`）；录制还需要回放脚本使用的 Pillow、imageio-ffmpeg 等依赖和可用渲染环境。本机 PyTorch 不可用时使用下述 NumPy 推理路径。

## 参考动作与 LLC 训练

参考动作共 49 条 / 13,828 帧 / 50 Hz（4.6 分钟），来源是 ACCAD 经 UMR 重定向到 G1 的结果，
再由 `czy/g1_wbt/scripts/umr_g1_to_tracking.py` 重采样并做正向运动学转成 tracking NPZ。
权重分配、数据特性与重建方式见 [ase/data/motions/g1/README.md](ase/data/motions/g1/README.md)。

数据集里 19 条转向片段各自切出一段直行片段（`*_straight_*`，权重 0.6），因为 HLC 只沿 +X
下发速度指令、纯转向步态用不上；站姿 `A1`/`A2` 从真实片段的静止窗口截取，用于覆盖零速指令。

下列命令从训练仓库根目录执行，首次准备依赖时加入 `--install-deps`。seed 仅为示例，可按实验选择数量和值，并记录实际使用值。

```bash
# G1 ASE LLC，从零训练。
python ase/train_g1_cloud.py --algorithm ase --motion-file ase/data/motions/g1/dataset_g1.yaml --num_envs 4096 --max_iterations 3000 --seed 42 --json-logs
```

LLC 本身不接收目标速度，z 代表技能输入。固定 z 表示在一段回放中保持同一个 latent；不同 case 可取不同 z。随机 z 可能产生站立、前进或其他行为，不能用单个 z 评价完整动作集。

## HLC 速度控制训练

当前 HLC 从默认站姿、零初速开始。每 5～8 秒更新目标速度：20% 为精确 0，其余 80% 在 [0,2) m/s 均匀采样；训练回合上限为 1500 个 LLC 控制步，即 30 秒。

任务奖励为：

```text
exp(-4*(vx-target_speed)^2 - 2*vy^2 - 2*(1-facing_x)
    - I(target_speed=0)*(2*foot_speed_sq + 0.05*leg_velocity_sq))
```

`foot_speed_sq` 为双脚线速度平方范数的均值，`leg_velocity_sq` 为腿部关节速度平方均值。HLC 优化奖励为 0.9×任务奖励＋0.1×冻结判别器奖励；LLC 不参与优化。

```bash
# 冻结 G1 LLC，从零训练 HLC。
python ase/train_g1_cloud.py --algorithm hrl --cfg-env ase/data/cfg/humanoid_g1_hlc_0_2.yaml --motion-file ase/data/motions/g1/dataset_g1.yaml --llc-checkpoint ../czy/data/<LLC实验>/model_3000.pt --num_envs 4096 --max_iterations 3000 --seed 20261118 --json-logs
```

这些命令用于复现配置，实际运行会创建新的训练过程。`humanoid_g1_hlc.yaml` 保留 0.6～1.6 m/s 历史任务；当前 0～2 m/s 实验须显式选择 `humanoid_g1_hlc_0_2.yaml`。HLC checkpoint 记录所依赖 LLC 的 SHA256、轮次和网络维度，回放或恢复必须使用对应 LLC。

续训时增加 `--resume-checkpoint HLC.pt`，保留匹配的 LLC、环境和参考清单。它恢复 HLC 模型、归一化、Adam 和轮次等训练状态；`--max_iterations` 表示总目标轮数。仿真环境重新初始化，不表示恢复原容器的连续物理状态。

云端入口可通过 `gm-run`/`flux` 调用本脚本，并传入 `--llc-checkpoint-url`/`--llc-sha256` 以及对应的 resume 参数下载和核验依赖；下载链接只用于运行时，不写入项目文档或源码。

## 回放与验收

Gym 默认在 201.5 录制，MuJoCo 默认在本机录制。HLC 的配对协议为：

```text
目标速度(m/s)：0 → 0.5 → 1 → 1.5 → 2 → 1 → 0 → 0.5 → 0
每段7.5秒，共67.5秒；默认站姿零初速，中间不重置，失稳则提前结束。
```

使用示例（每次使用新的输出目录，避免覆盖既有回放）：

```bash
python ase/replay_g1_hlc.py --engine gym --checkpoint ../czy/data/<HLC实验>/model_3000.pt --llc-checkpoint ../czy/data/<LLC实验>/model_3000.pt --output review/hlc_g1_gym --protocol stop-start --hold-seconds 7.5 --seed 20261118

python ase/replay_g1_hlc.py --engine mujoco --checkpoint ../czy/data/<HLC实验>/model_3000.pt --llc-checkpoint ../czy/data/<LLC实验>/model_3000.pt --output review/hlc_g1_mujoco --protocol stop-start --hold-seconds 7.5 --seed 20261118
```

Gym 输出 `isaac_hlc.mp4` 和 `isaac_hlc.csv`；MuJoCo 输出 `mujoco_hlc.mp4` 和 `mujoco_hlc.csv`。CSV 按 50Hz 记录指令、世界/机身速度、姿态、z、脚/腿 RMS、任务奖励及终止原因。`--no-video` 仅导出 CSV；`--speeds 1,0.5,0` 可指定额外减速序列。

`HLC_REPLAY_SEGMENTS` 按指令段统计，稳态速度误差排除每段首 2 秒。零速验收同时要求：

- 水平速度 < 0.05 m/s、脚 RMS < 0.1 m/s、腿部关节速度 RMS < 0.3 rad/s，且无跌倒。
- 3 秒内达到连续安静 0.5 秒；日志的 `settle_s` 是该连续窗口的完成时间。
- 停止段末 5 秒路径 ≤ 0.1 m；提前终止的短段不能当成完整末 5 秒验收。

低前向速度、初始静站或 Gym 无跌倒，都不能单独代替走→停、再次起步和 MuJoCo 迁移验收。

### MuJoCo 回放的两点说明

- PD 阻尼放在模型的关节阻尼里隐式积分，而不是作为显式 `-kd*qvel` 力矩：腕关节惯量小，显式阻尼在 5 ms 步长下会发散；Isaac Gym 的 PD 也是隐式的。
- 回放模型会在机器人 MJCF 上临时补地面、灯光和自碰撞排除对，不改动仓库里的资产文件。

### 本机无可用 PyTorch 时

在可用 PyTorch 的主机导出 HLC 及其匹配 LLC 的确定性 actor，再把 NPZ 与原 checkpoint 一起复制到回放主机：

```bash
PYTHONPATH=ase python -c "from replay_g1_hlc import export_numpy_replay; export_numpy_replay('HLC.pt', 'LLC.pt', 'hlc_actor.npz', 'llc_actor.npz')"

python ase/replay_g1_hlc.py --engine mujoco --checkpoint HLC.pt --llc-checkpoint LLC.pt --policy-npz hlc_actor.npz --llc-policy-npz llc_actor.npz --output review/hlc_g1_numpy --protocol stop-start --hold-seconds 7.5 --seed 20261118
```

NumPy 路径保留 HLC tanh、单位 z、LLC 归一化及动作裁剪，并校验两个源 checkpoint 哈希和 LLC 绑定关系。Windows 本机可将 `OPENBLAS_NUM_THREADS`、`OMP_NUM_THREADS`、`MKL_NUM_THREADS` 设为 1，避免小批量推理的线程开销。

### LLC 回放与技能诊断

```bash
# 默认站姿、不同 case 使用不同固定 z。
python ase/replay_g1.py --engine gym --checkpoint LLC.pt --output review/llc_fixed --cases 8 --seed 20261118 --init-mode default --latent-mode fixed

# 随机 z 与每段参考编码 z 的双起步探针。
python ase/evaluate_g1_coverage.py --checkpoint LLC.pt --dataset ase/data/motions/g1/dataset_g1.yaml --output review/llc_coverage --random-cases 64 --seed 20261118 --latent-seed 1000

# 已知走路/站立 z 的连续切换；seed 对需使用对应 LLC 的诊断结果。
python ase/evaluate_g1_transitions.py --checkpoint LLC.pt --dataset ase/data/motions/g1/dataset_g1.yaml --output review/llc_transitions --walk-seeds 1002 1026 --stand-seeds 1000 1003 --blend-seconds 0 0.5 --walk-seconds 3 --duration 10 --seed 20261118 --prefix switchstop_
```

诊断脚本导出视频、逐步 CSV 和汇总。`--dataset` 应与被测 LLC 的数据版本对应；参考编码 z 探针用于辅助检索，失败不能独立证明技能不存在。覆盖统计使用距离阈值等启发式规则，不能直接等同完整技能验收。LLC 回放的 `--motion-file` 用于起步状态，不在闭环中持续输入参考姿态。

## 移植期验证

两个可在无 Isaac Gym 环境下重复执行的自检：

```bash
# 资产/配置/维度静态一致性（需要 PyYAML）。
python ase/tests/test_g1_consistency.py

# 资产编译、接触高度、姿态物理、增益敏感性与动作映射（需要 mujoco 3.15）。
"C:/Users/cuizeyu/AppData/Local/Programs/Python/Python311/python.exe" czy/g1_wbt/scripts/check_g1_mujoco.py
```

实测结论（本机 MuJoCo 3.15）：

- 默认姿态骨盆高 0.7762 m，仿真接触时刻 0.7705 m，二者一致；`initialHeight` 0.78 m 留 4 mm 余量。
- 默认姿态质心与踝关节轴的水平偏移 0.0000 m；质心在踝上方 0.649 m。
- 该姿态在**部署 PD 增益**下是边缘平衡：踝关节刚度合计 80 N·m/rad，远低于单摆刚度 m·g·h ≈ 212 N·m/rad，零动作会以约 2.3～3.1 /s 的速率发散并在 1.6 秒触发 `base_tilt`。实测发散率 2.26 /s 与理论 3.07 /s 同量级，说明接触几何、质心与作动链表现符合模型，不是资产错误。
- 把腿关节增益提高到踝 250、膝 800、髋 400（仅测试用）后，同一姿态可稳定站立 10 秒、倾角 < 2.3°，膝部动作推动的跟踪误差 ≤ 0.17 rad，说明姿态、接触与动作映射本身没有问题。

这意味着**站姿稳定必须由策略学出来**，不能依赖被动的 PD 支撑；训练早期的默认姿态回合会较快倒下，这与上一代机型的设置一致。
另外两个只有真正跑 Isaac Gym 才会暴露的资产坑，现已修好并在自检里卡住：

- Isaac Gym 不解析 `<default class=...>` 的继承，关节属性只写在 default 里时它会当成无限位；生成脚本现在把属性全部摊平、并显式写 `type="hinge" limited="true"`（缺这两个属性时 `get_asset_dof_properties` 返回 ±FLT_MAX）。
- 即便写了 `range`，任务侧也从 URDF 覆盖 `props["lower"]/["upper"]`，保证 PhysX 关节限位与动作映射都拿到真实限位。

## 已知限制与后续工作

- **数据集的已知特性**：没有真正的站立捕获（`A1`/`A2` 是从真实片段截出的静止窗口）；ACCAD Male1 的
  5 条在源数据里整体悬空 2~3 cm（已按最低脚着地对齐）；跑步只有 2 条、没有 LAFAN 与冲刺；
  参考站姿是直腿（膝 0.1~0.27 rad），比默认姿态（膝 0.44）更直。详见
  [ase/data/motions/g1/README.md](ase/data/motions/g1/README.md)。
- `terminationContactHeight` 0.25 m、`armature` 0.01 是按几何与公开配置估的，未做敏感性实验。
- 跨引擎执行器/接触响应差异（尤其踝部小质量关节）尚未在 G1 上量化。
- 真机部署、手柄控制、任意方向转向与跑步不在当前验收范围。

## 上游来源

本项目基于 [ASE（Adversarial Skill Embeddings）](https://github.com/nv-tlabs/ASE) 适配宇树 G1，保留 ASE/AMP 与 HRL 实现。机器人模型来自宇树官方 MuJoCo/URDF 描述。论文来源见 [ASE 项目页](https://xbpeng.github.io/projects/ASE/index.html)，仓库许可证见 [LICENSE.txt](LICENSE.txt)。

仓库保留的原始 Reallusion 示例动作沿用上游“仅限非商业用途”的说明。