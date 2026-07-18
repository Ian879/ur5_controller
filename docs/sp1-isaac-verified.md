# SP1 verified (Isaac Sim end-to-end)

demo_plan_execute on backend:=isaac: 4 moves all [OK], exit 0.
Isaac UR5 moved to targets (home/approach/cartesian/home); /isaac_joint_states reflected motion.
Chain: MoveIt -> joint_trajectory_controller -> TopicBasedSystem -> /isaac_joint_commands -> Isaac articulation.
Bridge publishes /isaac_joint_states(6 UR joints,~138Hz)/clock/camera RGBD.
