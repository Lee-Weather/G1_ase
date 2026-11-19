# G1 参考动作数据

本目录存放 ASE LLC 训练用的 **G1 tracking NPZ** 参考动作，以及采样清单 `dataset_g1.yaml`。
数据已经就绪：**45 条 / 14,738 帧 / 50 Hz / 4.9 分钟 / 25.5 MB**，逐条校验记录在
`_validation_g1.json`（`--report quiet` 生成）。

## 来源链路

```text
ACCAD 动作捕捉（CMU ACCAD）
  → UMR 点云对应重定向到 Unitree G1 29-DoF        45 条 @120 Hz，分两处存放：
      组内共享目录（只读）E:/UMR/umr_data/G1        原始 30 条
      本项目目录（可写）  czy/data/umr_g1_stop_data  新补 15 条
  → 本仓库转换：50 Hz 重采样 + MuJoCo 正向运动学     czy/g1_wbt/scripts/umr_g1_to_tracking.py
  → ase/data/motions/g1/*.npz + dataset_g1.yaml
```

45 条重定向结果分两批：

| 批次 | 条数 | 内容 |
|---|---|---|
| 原始批 | 30 | Female1/Male1/Male2 的走、转向、起停、跑、跳跃 |
| exp_f1_7.1 补充批 | 15 | 真实静止站立 `A1`、站立摆动 `A2`、跑/冲刺（`C2 C4 C6 C7`、`M2_C1 M2_C3 M2_C4 M2_C5`、`S007 S008 S009`）、`B2`/`B4` 起停 |

> `E:/UMR/umr_data/G1` 是组内共享的数据目录，**只读不改**；新补的片段一律写进本项目
> `czy/data/umr_g1_stop_data/`（同样按 `<类别>/` 组织）。转换脚本的两个默认输入目录都在
> `DEFAULT_INPUT_DIRS` 里，并会按"类别 + 片段名"统一排序，所以片段放在哪个目录不影响清单顺序。

补充批用的是**同一套** UMR 配置（`E:/UMR/czy/g1/umr_g1_config.json` + `correspondence_slots_final.npz`），
与原始批完全一致；批量脚本 `E:/UMR/umr_g1/batch_g1_stop_data.py`（15 条全部 ok，单条 20~58 s）。
挑选依据：上一代机型项目里的 `czy/data/exp_f1_7.1/README.md`（那轮已验证过的成功配方）。

## 清单与权重

权重照搬 exp_f1_7.1 的配方（那边 36 条 / 10,199 帧 / 203 秒：站姿 5% + 四条停步 25% +
新增其他运动 10% + 原有其他运动 60%）。我们缺 LAFAN 的 3 条，缺出来的份额按归一化自然摊掉：

| 分组 | 条数 | 权重和 | 归一化占比 |
|---|---|---|---|
| 核心走/跑（沿用 F1 逐条概率） | 16 | 0.462 | 53.1% |
| 停步组 `B2`/`B2t2`/`C2`/`B6` | 4 | 0.250 | 28.7% |
| 其余运动（跳跃、Male1 转向、记录外的转向片段） | 23 | 0.100 | 11.5% |
| 静态站立 `A1`（真实捕获） | 1 | 0.050 | 5.7% |
| 站立摆动 `A2`（F1 里也在核心组） | 1 | 0.009 | 1.0% |

- `A1` 5% 与停步组 25% 是这套配方里最关键的两位——上一代"能站住、能停"就是靠它；
  本项目第一版 LLC（按类别粗分、站姿仅 2.4%、无停步组）出现拖步与停不稳，据此重排。
- **不再生成 `_straight_` 直行切片**：转向片段本身已在清单里，同段动作出两份会把它的实际采样率翻倍；
  exp_f1_7.1 也没有这类拷贝。
- **站姿切片也不再从别的片段截**：`A1`/`A2` 现在是 ACCAD `Female1General` 的真实静止捕获
  （`A1_-_stand` 150 帧 / 3.0 s / 根速度 0.02 m/s；`A2_-_sway` 603 帧 / 12.1 s / 0.14 m/s）。

## 文件格式契约

`ase/utils/g1_motion_lib.py` 按**名字**取关节与刚体，所以文件内部顺序只要自洽，但键名、单位、坐标系必须符合下表：

| 键 | 形状 | 含义 |
|---|---|---|
| `fps` | `(1,)` | 固定 50 |
| `joint_pos` / `joint_vel` | `(T, 29)` | 关节角 rad、角速度 rad/s，MJCF 文档顺序 |
| `body_pos_w` | `(T, 30, 3)` | 世界系刚体位置，米；第 0 个是 `pelvis` |
| `body_quat_w` | `(T, 30, 4)` | 世界系朝向，**wxyz** |
| `body_lin_vel_w` / `body_ang_vel_w` | `(T, 30, 3)` | 世界系线速度 m/s、角速度 rad/s |
| `joint_names` / `body_names` | `(29,)` / `(30,)` | `<U` 字符串数组（object dtype 会让 `allow_pickle=False` 加载失败）；刚体名须覆盖 keyBodies 与 contactBodies |

## 加工细节

| 步骤 | 做法 | 依据 |
|---|---|---|
| 重采样 | 120 → 50 Hz：位置/关节线性插值、根四元数 slerp；速度中心差分、角速度用相对旋转 | LLC 控制 50 Hz（`sim.dt=0.01`、`controlFrequencyInv=2`） |
| XY 归零 | 首帧骨盆 XY 移到原点 | 参考初始化以自身原点为基准 |
| 脚底对齐 | 逐帧取脚掌碰撞胶囊最低点，整段平移到"最低脚正好着地" | ACCAD Male1 那 5 条整体悬空 2~3 cm |

## 已知特性

- **真实站立已补上**：不再依赖从别的片段截出来的静站窗口；近零速区域第一次有了参考覆盖。
- **高速素材已补上**：`S009_Sprint1`（110 帧）、`S008_Run1`（90 帧）、`M2_C3_run`（136 帧）、
  `C3_Run`（85 帧）、`S007_QkWalk1`（128 帧，根速度 p95 2.2 m/s）、两个 `walk_to_run`。
  第一版只有 2 条跑（合计 6.5 s），这是"持续速度上不去"的直接原因之一。
- **参考站姿是直腿**：重定向出的站姿膝关节约 0.10~0.27 rad，比 `humanoid_g1.yaml` 的默认姿态
  （膝 0.44、根高 0.7762 m）明显更直，判别器会同时看到这两种先验。
- **Male1 的 5 条仍整体偏悬空**：源数据里人没有落地（UMR 侧同样记录），已按最低脚对齐，
  但多数帧脚底仍在 2~3 cm 高度，根高 0.80~0.89 m（Female1 是 0.73~0.82 m），双腿更直。
- **骨盆朝向有噪声**：逐帧偏航角速度均值 0.5~0.8 rad/s；AMP 观测去偏航并旋转到根坐标系，
  噪声只落在根角速度维度，但足以给判别器一条捷径。
- **没有 LAFAN**：exp_f1_7.1 里 LAFAN1 的 walk/run 占 12.9% 权重，这里完全没有；
  要补得先把裁剪 BVH（上一代项目里的 `czy/f1_wbt/lafan_src`）转成 SMPL-X 再走 UMR。

## 重建与校验

```bash
PY311="C:/Users/cuizeyu/AppData/Local/Programs/Python/Python311/python.exe"

# 重建全部 npz + 清单 + 校验报告（幂等，会覆盖同目录同名文件）
$PY311 czy/g1_wbt/scripts/umr_g1_to_tracking.py --write-dataset 1

# 只校验不写清单
$PY311 czy/g1_wbt/scripts/umr_g1_to_tracking.py --report quiet --write-dataset 0

# 静态契约与限位校验（不需要 torch）
$PY311 ase/tests/test_g1_consistency.py
```

新增片段只需把 UMR 的 npz 放到 `E:/UMR/umr_data/G1/<类别>/` 下重跑脚本；权重在脚本的
`WEIGHTS` 字典里按片段名指定，未列出的按"其余运动"平分 `OTHER_WEIGHT_TOTAL`。
命名约定：`<来源>_<动作>_g1_29.npz`。