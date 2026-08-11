#!/usr/bin/env python3
"""-y 侧不同 z 的 IK（avoid_collisions=True，带 box），找 finger 能到物体身 + elbow 限位 + 无碰撞 的 z。
物体中心 z=-0.351，物体身范围 z[-0.461,-0.241]。要 finger 在物体身（z≈-0.351）。
"""
import rclpy
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion


def main():
    rclpy.init()
    node = rclpy.create_node("ik_z")
    app = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    app.wait_for_service(timeout_sec=10)
    obj = CollisionObject()
    obj.id = "target_object"; obj.header.frame_id = "base_link"; obj.operation = CollisionObject.ADD
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.06, 0.06, 0.22])]
    obj.primitive_poses = [Pose(position=Point(x=0.45, y=0.05, z=-0.351), orientation=Quaternion(w=1.0))]
    scene = PlanningScene(world=type(obj.world)())
    scene.world.collision_objects = [obj]
    f = app.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    print("RESULT box:", f.result().success if f.done() and f.result() else "FAIL")

    ik = node.create_client(GetPositionIK, "/compute_ik")
    ik.wait_for_service(timeout_sec=10)
    for z in [-0.351, -0.32, -0.30, -0.28, -0.25, -0.20]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position = Point(x=0.45, y=-0.181, z=z)
        req.ik_request.pose_stamped.pose.orientation = Quaternion(x=-0.707, y=0, z=0, w=0.707)
        r = ik.call_async(req)
        rclpy.spin_until_future_complete(node, r, timeout_sec=8)
        if r.done() and r.result():
            ec = r.result().error_code.val
            jt = list(r.result().solution.joint_state.position[:6]) if r.result().solution.joint_state.position else []
            elbow = jt[2] if len(jt) > 2 else "?"
            in_obj = "物体身内" if -0.461 <= z <= -0.241 else "物体外"
            ok = "✓" if ec == 1 else "✗"
            print(f"RESULT z={z} ({in_obj}) err={ec} {ok} elbow={round(elbow,2) if isinstance(elbow,float) else elbow}")
        else:
            print(f"RESULT z={z} timeout")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
