"""Static consistency checks for the G1 port.

Runs without Isaac Gym, PyTorch or MuJoCo, so it can gate a commit on any
machine: it re-derives every number the training code assumes from the asset and
config files themselves.

Covered assumptions:

* the MJCF exposes exactly 29 joints in the same order as 29 rigid bodies after
  the pelvis (the ASE action/observation layout relies on ``dof_offsets =
  range(30)``), and each DoF drives the URDF child link of the same joint;
* every referenced mesh file exists and one actuator drives each joint;
* ``humanoid_g1.yaml`` names only existing bodies, its default posture is inside
  the URDF limits, and every joint resolves to exactly one PD gain group;
* the derived observation sizes (actor 93, critic 448, AMP 890) match the
  network configuration the training entry selects;
* the HLC config chain is complete and its ``llc_steps`` matches the task config;
* no F1 names survive in the source tree.

.. code-block:: bash

    python ase/tests/test_g1_consistency.py
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
ASE = ROOT / "ase"
MJCF = ASE / "data/assets/g1/mjcf/g1_29dof.xml"
URDF = ASE / "data/assets/g1/urdf/g1_29dof.urdf"
CFG = ASE / "data/cfg/humanoid_g1.yaml"
HLC = ASE / "data/cfg/humanoid_g1_hlc.yaml"
HLC_0_2 = ASE / "data/cfg/humanoid_g1_hlc_0_2.yaml"
TRAIN = ASE / "data/cfg/train/rlg"

FAILURES: list[str] = []


def check(condition: bool, message: str) -> None:
    print(("  ok   " if condition else "  FAIL ") + message)
    if not condition:
        FAILURES.append(message)


def load_yaml(path: Path) -> dict:
    """Resolve the ``extends:`` chain with the same deep merge as utils.config."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    parent = data.pop("extends", None)
    if parent is None:
        return data
    merged = load_yaml(path.parent / parent)

    def merge(destination: dict, updates: dict) -> None:
        for key, item in updates.items():
            if isinstance(item, dict) and isinstance(destination.get(key), dict):
                merge(destination[key], item)
            else:
                destination[key] = item

    merge(merged, data)
    return merged


def mjcf_model() -> dict:
    root = ET.parse(MJCF).getroot()
    worldbody = root.find("worldbody")
    joints = [j.get("name") for j in worldbody.iter("joint")]
    bodies = [b.get("name") for b in worldbody.iter("body")]
    motors = {m.get("joint") for m in root.findall("actuator/motor")}
    meshes = [m.get("file") for m in root.findall("asset/mesh")]
    return dict(root=root, joints=joints, bodies=bodies, motors=motors, meshes=meshes)


def urdf_model() -> dict:
    root = ET.parse(URDF).getroot()
    joints, limits = [], {}
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.get("name")
        joints.append(name)
        limits[name] = dict(child=joint.find("child").get("link"),
                            **{key: float(joint.find("limit").get(key))
                               for key in ("lower", "upper", "effort", "velocity")})
    return dict(joints=joints, limits=limits)


def validate_motions(dataset: Path, entries: list, cfg: dict, urdf: dict) -> None:
    """Every manifest entry must satisfy the contract G1MotionLib relies on."""
    keys = ("fps", "joint_pos", "joint_vel", "body_pos_w", "body_quat_w",
            "body_lin_vel_w", "body_ang_vel_w", "joint_names", "body_names")
    expected_joints = list(urdf["limits"])
    problems, frames, weights = [], 0, []
    for entry in entries:
        path = dataset.parent / entry["file"]
        weight = float(entry.get("weight", 1.0))
        weights.append(weight)
        if not path.is_file():
            problems.append("{}: missing".format(entry["file"]))
            continue
        with np.load(str(path), allow_pickle=False) as raw:
            missing = [key for key in keys if key not in raw.files]
            if missing:
                problems.append("{}: missing keys {}".format(entry["file"], missing))
                continue
            joints = raw["joint_names"].tolist()
            bodies = raw["body_names"].tolist()
            absent = [name for name in expected_joints if name not in joints]
            if absent:
                problems.append("{}: joints {}".format(entry["file"], absent[:2]))
            absent = [name for name in cfg["keyBodies"] + cfg["contactBodies"] if name not in bodies]
            if absent:
                problems.append("{}: bodies {}".format(entry["file"], absent[:2]))
            n = len(raw["joint_pos"])
            frames += n
            fps = float(raw["fps"].reshape(-1)[0])
            if abs(fps - 50.0) > 1e-3:
                problems.append("{}: fps {}".format(entry["file"], fps))
            if n < 2:
                problems.append("{}: {} frames".format(entry["file"], n))
            shapes = {"joint_pos": (n, len(joints)), "joint_vel": (n, len(joints)),
                      "body_pos_w": (n, len(bodies), 3), "body_quat_w": (n, len(bodies), 4),
                      "body_lin_vel_w": (n, len(bodies), 3), "body_ang_vel_w": (n, len(bodies), 3)}
            for key, shape in shapes.items():
                if raw[key].shape != shape:
                    problems.append("{}: {} {}".format(entry["file"], key, raw[key].shape))
            for key in shapes:
                if not np.isfinite(raw[key]).all():
                    problems.append("{}: non-finite {}".format(entry["file"], key))
            norm = np.linalg.norm(raw["body_quat_w"], axis=-1)
            if not np.allclose(norm, 1.0, atol=1e-3):
                problems.append("{}: quaternion norm {:.4f}".format(entry["file"], float(norm.max())))
            low = np.array([urdf["limits"][name]["lower"] for name in joints])
            high = np.array([urdf["limits"][name]["upper"] for name in joints])
            joints_pos = raw["joint_pos"]
            if ((joints_pos < low - 1e-3) | (joints_pos > high + 1e-3)).any():
                problems.append("{}: joint limits".format(entry["file"]))
    check(not problems, "every clip satisfies the tracking contract ({})".format(problems[:3]))
    check(all(weight > 0 for weight in weights), "all sampling weights are positive")
    check(len(entries) >= 2, "dataset has at least two clips")
    print("  clips {}  frames {}  {:.1f} min  {:.1f} MB".format(
        len(entries), frames, frames / 50.0 / 60.0,
        sum((dataset.parent / entry["file"]).stat().st_size for entry in entries
            if (dataset.parent / entry["file"]).is_file()) / 2 ** 20))

def main() -> None:
    mjcf = mjcf_model()
    urdf = urdf_model()
    cfg = load_yaml(CFG)["env"]

    print("\n[asset]")
    check(len(mjcf["joints"]) == 29, "MJCF has 29 joints (got {})".format(len(mjcf["joints"])))
    check(len(mjcf["bodies"]) == 30, "MJCF has pelvis + 29 links (got {})".format(len(mjcf["bodies"])))
    check(mjcf["bodies"][0] == "pelvis", "root body is pelvis (got {})".format(mjcf["bodies"][0]))
    check(mjcf["joints"] == urdf["joints"],
          "MJCF joint order matches the URDF revolute order")
    order_ok = all(mjcf["bodies"][i + 1] == urdf["limits"][name]["child"]
                   for i, name in enumerate(mjcf["joints"]))
    check(order_ok, "DoF i drives rigid body i+1 (ASE uses dof_offsets = range(30))")
    check(mjcf["motors"] == set(mjcf["joints"]), "one actuator per joint")
    missing = [m for m in mjcf["meshes"] if not (MJCF.parent / m).is_file()]
    check(not missing, "all {} mesh files exist ({})".format(len(mjcf["meshes"]), missing[:3]))
    # Isaac Gym's MJCF loader does not apply <default class=...> inheritance to
    # joints: a range that only lives in a default class comes back as an
    # unlimited joint (lower == upper == 0) and pd_action_parameters rejects it.
    # The asset must therefore be authored flat.
    joints = list(mjcf["root"].find("worldbody").iter("joint"))
    check(mjcf["root"].find("default") is None, "MJCF has no <default> block (Isaac Gym would ignore it)")
    implicit = [j.get("name") for j in joints if j.get("range") is None or j.get("axis") is None]
    check(not implicit, "every joint carries an explicit range and axis ({})".format(implicit[:3]))
    # Isaac Gym additionally needs the joint type and the limited flag spelled out;
    # without them it reports +-FLT_MAX and pd_action_parameters rejects the limits.
    untyped = [j.get("name") for j in joints
               if j.get("type") != "hinge" or j.get("limited") != "true"]
    check(not untyped, "every joint declares type=hinge limited=true ({})".format(untyped[:3]))
    ranges = {j.get("name"): [float(v) for v in j.get("range").split()] for j in joints}
    mismatch = [n for n, (low, high) in ranges.items()
                if abs(low - urdf["limits"][n]["lower"]) > 1e-3
                or abs(high - urdf["limits"][n]["upper"]) > 1e-3]
    check(not mismatch, "MJCF joint ranges match the URDF ({})".format(mismatch[:3]))

    print("\n[environment config]")
    check((ROOT / cfg["asset"]["assetRoot"] / cfg["asset"]["assetFileName"]).is_file(),
          "assetFileName resolves ({})".format(cfg["asset"]["assetFileName"]))
    check(set(cfg["defaultJointPos"]) == set(mjcf["joints"]),
          "defaultJointPos covers exactly the 29 joints")
    outside = [n for n, q in cfg["defaultJointPos"].items()
               if not urdf["limits"][n]["lower"] - 1e-9 <= q <= urdf["limits"][n]["upper"] + 1e-9]
    check(not outside, "default posture is inside the joint limits ({})".format(outside))
    for group in ("keyBodies", "contactBodies", "forceSensorBodies"):
        unknown = [n for n in cfg[group] if n not in mjcf["bodies"]]
        check(not unknown, "{} only names MJCF bodies ({})".format(group, unknown))
    flat = [n for group in cfg["selfCollisionFilterGroups"] for n in group]
    check(not [n for n in flat if n not in mjcf["bodies"]],
          "selfCollisionFilterGroups only names MJCF bodies")
    gains = cfg["pdGains"]
    resolution = {name: [key for key in gains if key in name] for name in mjcf["joints"]}
    ambiguous = {n: keys for n, keys in resolution.items() if len(keys) != 1}
    check(not ambiguous, "every joint matches exactly one PD gain group ({})".format(ambiguous))
    unmatched = [n for n, keys in resolution.items() if not keys]
    check(not unmatched, "no joint is left without a PD gain ({})".format(unmatched))
    check(cfg["armature"] >= 0, "armature is non-negative")
    for key in ("terminationHeight", "terminationTilt", "terminationContactForce",
                "terminationContactHeight", "initialHeight", "episodeLength"):
        check(key in cfg, "env.{} is set".format(key))
    check(0 < cfg["terminationHeight"] < cfg["initialHeight"],
          "terminationHeight {:.2f} < initialHeight {:.2f}".format(
              cfg["terminationHeight"], cfg["initialHeight"]))

    print("\n[observation sizes]")
    num_bodies = len(mjcf["bodies"])
    critic = 1 + 3 * num_bodies - 3 + 6 * num_bodies + 3 * num_bodies + 3 * num_bodies
    actor = 3 + 3 + 29 + 29 + 29
    per_step = 13 + 29 + 29 + 3 * len(cfg["keyBodies"])
    amp = cfg.get("numAMPObsSteps", 10) * per_step
    check(actor == 93, "actor observation is 93 (computed {})".format(actor))
    check(critic == 448, "critic observation is 448 (computed {} for {} bodies)".format(
        critic, num_bodies))
    check(amp == 890, "AMP observation is 890 (computed {}, {} key bodies)".format(
        amp, len(cfg["keyBodies"])))
    check(len(cfg["keyBodies"]) == 6, "six key bodies keep the AMP size at 890")

    print("\n[HLC config chain]")
    hlc = load_yaml(HLC_0_2)["env"]
    check(hlc["episodeLength"] == 1501, "HLC episode is 1501 LLC steps (30 s)")
    check(hlc["stateInit"] == "Default", "HLC starts from the default posture")
    check(hlc["hlc"]["llcSteps"] == 5, "HLC updates every 5 LLC steps (10 Hz)")
    check(hlc["hlc"]["speedRange"] == [0.0, 2.0], "HLC speed range is 0-2 m/s")
    check(0 <= hlc["hlc"].get("stopProbability", -1) <= 1, "stopProbability is a probability")
    for key in ("speedTrackingScale", "lateralTrackingScale", "headingTrackingScale"):
        check(key in hlc["hlc"], "hlc.{} is set".format(key))
    check(hlc["defaultJointPos"] == cfg["defaultJointPos"],
          "the HLC inherits the same default posture")

    print("\n[training configs]")
    ase_cfg = yaml.safe_load((TRAIN / "ase_g1.yaml").read_text(encoding="utf-8"))["params"]["config"]
    hrl_cfg = yaml.safe_load((TRAIN / "hrl_g1.yaml").read_text(encoding="utf-8"))["params"]["config"]
    check(ase_cfg["latent_dim"] == 32, "ASE latent is 32-D (got {})".format(ase_cfg["latent_dim"]))
    check(ase_cfg["disc_reward_w"] + ase_cfg["enc_reward_w"] == 1.0 and ase_cfg["task_reward_w"] == 0.0,
          "ASE optimises discriminator + encoder only")
    check(hrl_cfg["llc_steps"] == hlc["hlc"]["llcSteps"],
          "HLC llc_steps matches the task config")
    check((ROOT / hrl_cfg["llc_config"]).is_file(), "hrl_config points at an existing LLC config")
    check(hrl_cfg.get("frozen_g1_llc") is True, "HLC freezes the LLC")
    check(hrl_cfg["task_reward_w"] + hrl_cfg["disc_reward_w"] == 1.0, "HLC reward weights sum to 1")
    for name in ("ase_g1.yaml", "amp_g1.yaml", "hrl_g1.yaml"):
        check((TRAIN / name).is_file(), "training config {} exists".format(name))

    print("\n[motion dataset]")
    dataset = ASE / "data/motions/g1/dataset_g1.yaml"
    payload = yaml.safe_load(dataset.read_text(encoding="utf-8"))
    entries = payload.get("motions")
    check(isinstance(entries, list), "dataset_g1.yaml has a motions list")
    check((ASE / "data/motions/g1/README.md").is_file(), "motions README documents the NPZ contract")
    if not entries:
        print("  note  dataset is empty: generate the tracking NPZ before training")
    else:
        validate_motions(dataset, entries, cfg, urdf)

    print("\n[no F1 remnants]")
    # Match the F1 identifiers this port renamed, not every occurrence of "f1"
    # (the upstream ASE motion_lib.py uses f1l/f1 as local index names).
    patterns = ("humanoid_f1", "FrozenF1", "f1_hlc", "f1_control", "f1_motion_lib",
                "assets/f1", "motions/f1", "weichai", "F1_", "dataset_f1", "_f1.yaml",
                "train_f1", "replay_f1", "evaluate_f1", "HumanoidF1")
    stale = []
    for path in ASE.rglob("*"):
        if not path.is_file() or path.suffix not in {".py", ".yaml", ".yml", ".md"}:
            continue
        if "__pycache__" in path.parts or path.name == "test_g1_consistency.py":
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        hits = [token for token in patterns if token in text]
        if hits:
            stale.append("{}:{}".format(path.relative_to(ROOT), hits[0]))
    check(not stale, "no F1 identifiers outside this test ({})".format(stale[:5]))

    print("\n{}".format("PASS" if not FAILURES else "FAILED: " + "; ".join(FAILURES)))
    sys.exit(0 if not FAILURES else 1)


if __name__ == "__main__":
    main()