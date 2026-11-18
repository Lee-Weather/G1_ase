# G1 参考动作数据

本目录存放 ASE LLC 训练用的 **G1 tracking NPZ** 参考动作，以及采样清单 `dataset_g1.yaml`。
数据已经就绪：**49 条 / 13,828 帧 / 50 Hz / 4.6 分钟 / 24 MB**，逐条校验记录在
`_validation_g1.json`。

## 来源链路

```text
ACCAD 动作捕捉（UMR 的 sample_data/ACCAD）
  → UMR 点云对应重定向到 Unitree G1 29-DoF        E:/UMR/umr_data/G1（30 条 @120 Hz / 27,149 帧）
  → 本仓库转换：50 Hz 重采样 + MuJoCo 正向运动学     czy/g1_wbt/scripts/umr_g1_to_tracking.py
  → ase/data/motions/g1/*.npz + dataset_g1.yaml
```

UMR 侧的模型与质量报告：`E:/UMR/czy/g1/`（G1 模型 + 对应关系）、`E:/UMR/umr_data/G1/README.md`
（逐条帧数、限位与穿地核查）。转换脚本用**本仓库自己的** `g1/mjcf/g1_29dof.xml` 做运动学，
因此 `body_*` 字段与本仓库资产严格一致；已核对 ASE 模型与 UMR 模型在相同 qpos 下刚体位姿完全一致。

## 清单与权重

`dataset_g1.yaml` 按类别分配采样权重（合计 25.0）：

| 类别 | 条数 | 帧数 | 权重合计 | 占比 |
|---|---|---|---|---|
| 直行切片 `*_straight_*` | 17 | 2,286 | 10.2 | 40.8% |
| 转向行走 | 19 | 7,685 | 7.8 | 31.2% |
| 起停过渡 | 5 | 1,843 | 2.4 | 9.6% |
| 跑（C3、走→跑） | 2 | 356 | 1.7 | 6.8% |
| 直线走/倒退走 | 2 | 958 | 1.5 | 6.0% |
| 跳跃步 | 2 | 471 | 0.8 | 3.2% |
| 站姿 `A1`/`A2` | 2 | 229 | 0.6 | 2.4% |

权重参照上一代机型的分配（站姿约 2.5%、过渡 0.4~0.6、直线走/跑 1.0），并按本任务的
实际需要调整：HLC 只沿世界 +X 下发速度指令，**转向步态本身用不上**，所以把 19 条转向
片段里真正直行的那一段单独切出来（`*_straight_*`，净转角 0.02~0.33 rad、均速 0.5~1.1 m/s），
权重 0.6，同时把整条转向片段降到 0.35~0.45。

## NPZ 契约（本目录文件均已满足）

`utils/g1_motion_lib.py` 按**名字**重排关节与刚体，所以文件内的顺序只要自洽即可，但键名、
单位与坐标系必须符合下表（Isaac Lab whole-body-tracking 的 tracking npz 约定）：

| 键 | 形状 | 含义 |
|---|---|---|
| `fps` | `(1,)` | 帧率，本项目固定 50 |
| `joint_pos` | `(T, 29)` | 关节角，弧度，MJCF 文档顺序 |
| `joint_vel` | `(T, 29)` | 关节角速度，rad/s |
| `body_pos_w` | `(T, 30, 3)` | 世界系刚体位置，米；第 0 个是 `pelvis` |
| `body_quat_w` | `(T, 30, 4)` | 世界系刚体朝向，**wxyz** 四元数 |
| `body_lin_vel_w` | `(T, 30, 3)` | 世界系线速度，m/s |
| `body_ang_vel_w` | `(T, 30, 3)` | 世界系角速度，rad/s |
| `joint_names` | `(29,)` | 关节名（`<U` 字符串数组，勿用 object dtype） |
| `body_names` | `(30,)` | 刚体名，须覆盖 keyBodies 与 contactBodies |

加载器要求 `body_names` 至少包含 `pelvis`、`torso_link`、左右 `ankle_roll_link`、
左右 `wrist_roll_link`（keyBodies）以及左右 `ankle_pitch_link`（contactBodies）。

## 重建

```bash
PY311="C:/Users/cuizeyu/AppData/Local/Programs/Python/Python311/python.exe"

# 转换全部 UMR 片段 + 截取站姿切片 + 重写清单（幂等，会覆盖同目录同名文件）
$PY311 czy/g1_wbt/scripts/umr_g1_to_tracking.py --write-dataset 1
```

关键处理：

- **重采样**：120 Hz → 50 Hz，位置与关节线性插值、根朝向 slerp；速度用中心差分、
  角速度用相邻帧四元数相对旋转求解。
- **XY 归零**：每条片段把首帧骨盆 XY 平移到原点；偏航保持不变（AMP 观测对偏航不变，
  参考初始化也不要求朝向）。
- **脚底对齐**：按本仓库模型的脚掌碰撞胶囊几何逐帧算最低点，整段抬到“最低脚正好着地”。
  ACCAD 里 Male1 那 5 条在源数据中整体悬空 2~3 cm，这一步把它们的根高拉回统一基准。

## 已知特性（训练前应该有预期）

- **没有真正的站立捕获**：ACCAD 这套数据没有专门的静站片段，而 HLC 的零速指令需要
  “站住”这个技能，因此 `A1_stand` 从 `Male2_B14` 开头截 2.4 s（根水平速度 0.033 m/s），
  `A2_sway` 从 `Female1_B6` 收尾截 2.2 s（0.115 m/s，带轻微重心晃动），各 0.3 权重。
- **参考站姿是直腿**：重定向出的站姿膝关节约 0.1~0.27 rad，比 `humanoid_g1.yaml` 的
  默认姿态（膝 0.44）更直，根高约 0.79 m。判别器会因此推动策略在站立时把膝盖伸直，
  这属于预期行为；如果要让默认姿态与参考一致，需要重新搜默认姿态并同步 `initialHeight`。
- **Male1 的 5 条整体偏悬空**：这是源数据（人体在 UMR 里没有落地）的问题，UMR 侧也记录了
  同样现象；已按最低脚着地对齐，但多数帧仍离地 2~3 cm。
- **跑只有 2 条**（`C3_Run` 1.7 s、`C5_walk_to_run` 5.4 s），没有 LAFAN、没有冲刺，
  高速能力主要靠这两条外推。
- **骨盆朝向有噪声**：逐帧偏航角速度均值 0.5~0.8 rad/s，瞬时“直行”判据不可用，
  因此直行切片用 2.5 s 窗口内的净转角判定。AMP 观测会去掉偏航并旋转到根坐标系，
  噪声只体现在根角速度维度上。

## 校验

```bash
# 逐条检查键、形状、帧率、关节限位、四元数归一化、名字覆盖（不需要 torch）
python ase/tests/test_g1_consistency.py
```

转换脚本自身也会把逐条统计写入 `_validation_g1.json`（根高范围、脚底最低点、离地比例、
限位越界数、关节速度 p99、根水平速度 p95、z 对齐量）。

## 扩充数据

UMR 侧新增重定向结果（`E:/UMR/umr_data/G1/` 下按来源分子目录）后，直接重跑上面的转换命令
即可，新片段会按文件名规则自动套权重。命名约定：`<来源>_<动作>_g1_29.npz`；
`_straight_` 由脚本生成，`A1_`/`A2_` 是站姿切片。若新增片段来自新的人体来源，
记得在 `_validation_g1.json` 里核对悬空与限位情况。