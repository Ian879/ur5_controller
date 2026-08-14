import numpy as np
from ur5_motion.grasp_geometry import tilted_reach, quat_from_reach, grasp_poses, GRIPPER_TIP_OFFSET, APPROACH_MARGIN

def test_tilted_reach_horizontal_is_unit():
    a = tilted_reach(azim=0.0, tilt=0.0)
    assert abs(np.linalg.norm(a) - 1.0) < 1e-9
    assert abs(a[2]) < 1e-9                      # tilt=0 → pure horizontal

def test_tilted_reach_points_downward_when_tilted():
    a = tilted_reach(azim=0.0, tilt=np.pi/4)     # 45° down
    assert a[2] < -0.7                           # downward component
    assert abs(np.linalg.norm(a) - 1.0) < 1e-9

def test_quat_from_reach_keeps_fingers_horizontal_for_tilt():
    import math
    for tilt in [0.0, math.pi/6, math.pi/4]:
        a = tilted_reach(azim=math.pi/2, tilt=tilt)
        q = quat_from_reach(a)
        # tool0+X (fingers) in world = R[:,0] of the quat; its z-component must be ~0 (horizontal)
        x, y, z, w = q
        tx = (1 - 2*(y*y + z*z), 2*(x*y + z*w), 2*(x*z - y*w))
        assert abs(tx[2]) < 1e-6, f"fingers not horizontal at tilt {tilt}"

def test_grasp_poses_tcp_lands_on_object_center():
    obj = np.array([0.5, 0.1, -0.5])
    a = tilted_reach(azim=0.0, tilt=np.pi/4)
    q = quat_from_reach(a)
    grasp, pre = grasp_poses(obj, a, q)
    tcp = np.array(grasp) + GRIPPER_TIP_OFFSET * a
    assert np.allclose(tcp, obj, atol=1e-6)
    assert np.allclose(np.array(pre), np.array(grasp) - APPROACH_MARGIN * a, atol=1e-6)
