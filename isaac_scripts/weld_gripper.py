#!/usr/bin/env python3
"""把 Lebai 夹爪 + 两个转接环**焊进 UR5 articulation**（合并成一个刚体系统），
使夹爪跟手臂一起动、手指物理开合。

前置（在 Isaac GUI 里先做，目视校）：
  1. 把 cameraURflangeSupport 摆到 UR5 法兰面（沿 tool0 轴，薄面贴法兰）。
  2. 把夹爪转接环 tn__j0l6Rg0k… 叠到 cameraURflangeSupport 上（+0.01145m）。
  3. 把夹爪 gripper_base 摆到转接环上（+0.012m）。
  4. 把相机机身 orbbec_gemini_2 摆到 cameraURflangeSupport 上的眼在手位置。
  全部目视对齐后**存盘**，再跑本脚本。

本脚本做（确定性焊）：
  - 备份 scene.usd → scene.usd.orig
  - 从 /World/gripper_abs_stl/Geometry 移除 ArticulationRootAPI（不再是独立 articulation）
  - 删掉 cameraURflangeSupport 下两个坏的 FixedJoint（毫米单位混入）
  - 给转接环/相机加 RigidBodyAPI（若缺，否则 joint 不当刚体）
  - 用 pose-preserving PhysicsFixedJoint 焊 4 对（按当前相对位冻结）：
      wrist_3_link ↔ cameraURflangeSupport
      cameraURflangeSupport ↔ tn__j0l6Rg0k…（夹爪转接环）
      tn__j0l6Rg0k… ↔ gripper_base_link
      cameraURflangeSupport ↔ orbbec_gemini_2（相机分支）
  - 存盘

跑法（先 --dry-run 看 joint 偏移，确认再实跑）：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh …/isaac_scripts/weld_gripper.py --dry-run
  ./python.sh …/isaac_scripts/weld_gripper.py            # 实跑（备份+改+存盘）

焊后在 Isaac 里 Play，拖手臂应见夹爪跟随；失败（PhysX 不合并）则 GUI 手动建 Fixed Joint。
"""
import os
import argparse
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf


# ---- 焊接对（body0 → body1）。prim 路径按 ~/Desktop/scene.usd 默认；不同则改这里或加 --robot_prim ----
FLANGE_LINK = "/World/ur5/wrist_3_link"          # UR5 articulation 里的 link（焊点锚）
CAM_ADAPTER = "/World/rgbd_camera/cameraURflangeSupport"
GRIPPER_ADAP = "/World/LMG_90_adapter/tn__j0l6Rg0k0f35W04y7p1"
GRIPPER_BASE = "/World/gripper_abs_stl/Geometry/gripper_base_link"
CAMERA_BODY = "/World/rgbd_camera/orbbec_gemini_2"
GRIPPER_ART_ROOT = "/World/gripper_abs_stl/Geometry"   # 要去 ArticulationRootAPI 的 prim
BAD_JOINTS = [
    "/World/rgbd_camera/cameraURflangeSupport/FixedJoint",
    "/World/rgbd_camera/cameraURflangeSupport/FixedJoint0",
]

WELD_PAIRS = [
    (FLANGE_LINK, CAM_ADAPTER, "weld_flange_to_camAdapter"),
    (CAM_ADAPTER, GRIPPER_ADAP, "weld_camAdapter_to_gripperAdapter"),
    (GRIPPER_ADAP, GRIPPER_BASE, "weld_gripperAdapter_to_gripperBase"),
    (CAM_ADAPTER, CAMERA_BODY, "weld_camAdapter_to_camera"),
]


def world_matrix(stage, path):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        return None, None
    xf = UsdGeom.Xformable(p)
    return xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default()), p


def ensure_rigid_body(p):
    if not p.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(p)
        return True
    return False


def make_fixed_joint(stage, b0_path, b1_path, joint_name, dry):
    """Create a PhysicsFixedJoint that welds b1 to b0 at their CURRENT relative pose.
    Constraint: T_world_b0 * F0 == T_world_b1 * F1. With F0=identity →
    F1 = T_world_b1⁻¹ * T_world_b0 (b0's pose in b1's local frame)."""
    T0, p0 = world_matrix(stage, b0_path)
    T1, p1 = world_matrix(stage, b1_path)
    if T0 is None or T1 is None:
        print(f"  [SKIP] {joint_name}: missing body ({b0_path} or {b1_path})")
        return
    F1 = T1.GetInverse() * T0          # Gf.Matrix4d: b0 pose in b1 frame
    lp1 = F1.ExtractTranslation()
    rq = F1.ExtractRotationQuat()
    real, imag = rq.GetReal(), rq.GetImaginary()
    print(f"  [{joint_name}]  {b0_path}  ↔  {b1_path}")
    print(f"      localPos1=({lp1[0]:.5f},{lp1[1]:.5f},{lp1[2]:.5f})  "
          f"localRot1=(w={real:.5f}, xyz=({imag[0]:.5f},{imag[1]:.5f},{imag[2]:.5f}))")
    if dry:
        return
    # ensure both are rigid bodies
    added = []
    if ensure_rigid_body(p0):
        added.append(b0_path)
    if ensure_rigid_body(p1):
        added.append(b1_path)
    if added:
        print(f"      + RigidBodyAPI applied to: {added}")
    joint_path = f"/World/joints/{joint_name}"
    # clear any pre-existing
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([b0_path])
    joint.CreateBody1Rel().SetTargets([b1_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))  # identity (w,xyz)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(float(lp1[0]), float(lp1[1]), float(lp1[2])))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(float(real),
                                             Gf.Vec3f(float(imag[0]), float(imag[1]), float(imag[2]))))
    joint.CreateJointEnabledAttr().Set(True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=os.path.expanduser("~/Desktop/scene.usd"))
    ap.add_argument("--dry-run", action="store_true", help="只打印将建的 joint 偏移，不改不存")
    args, _ = ap.parse_known_args()

    print(f"SCENE: {args.scene}   dry_run={args.dry_run}")
    stage = Usd.Stage.Open(args.scene)
    assert stage, f"cannot open {args.scene}"

    print("\n=== 1) 移除夹爪 ArticulationRootAPI（并入 UR5 articulation 的前置）===")
    root_prim = stage.GetPrimAtPath(GRIPPER_ART_ROOT)
    if root_prim.IsValid() and root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"  {GRIPPER_ART_ROOT} 有 ArticulationRootAPI → {'(dry-run 不移除)' if args.dry_run else '移除中…'}")
        if not args.dry_run:
            root_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    else:
        print(f"  {GRIPPER_ART_ROOT} 无 ArticulationRootAPI（已移除或不在）")

    print("\n=== 2) 删除坏的 cameraURflangeSupport FixedJoints（毫米单位）===")
    for jp in BAD_JOINTS:
        jp_prim = stage.GetPrimAtPath(jp)
        if jp_prim.IsValid():
            print(f"  删 {jp}  {'(dry-run)' if args.dry_run else '…'}")
            if not args.dry_run:
                stage.RemovePrim(jp)
        else:
            print(f"  {jp} 不存在（跳过）")

    print("\n=== 3) 建 pose-preserving FixedJoint（焊 4 对）===")
    for b0, b1, name in WELD_PAIRS:
        make_fixed_joint(stage, b0, b1, name, args.dry_run)

    if args.dry_run:
        print("\n[dry-run] 未改未存。确认偏移合理后去掉 --dry-run 实跑。")
    else:
        # 备份 + 存盘
        bak = args.scene + ".orig"
        if not os.path.exists(bak):
            os.system(f'cp "{args.scene}" "{bak}"')
            print(f"\n备份 → {bak}")
        stage.Save()
        print(f"\n已存盘 → {args.scene}")
        print("下一步：Isaac 里打开场景 → Play → 拖手臂验证夹爪跟随；RViz 发 gripper 命令验手指开合。")

    app.close()


if __name__ == "__main__":
    main()
