# SP1 Phase 0 verdict

**STABLE** — 官方 isaac_moveit (Franka) 在本机 ROS2 Humble + Isaac Sim 6.0.1 + 双 RTX5000 下验证通过：
RViz Plan/Execute 顺畅、Franka 在 Isaac 里按规划运动、无间歇性失败/超时。

用户实测结论："动得很顺"。

⇒ 走 TopicBasedSystem 路线（T5/T6/T7），T9 passthrough 降级不需要。
