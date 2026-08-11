from glob import glob

from setuptools import setup

package_name = "ur5_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.demos"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shengzhi",
    maintainer_email="todo@todo.todo",
    description="UR5 MoveIt2 Python 规划客户端 + 示例",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_plan_execute = ur5_motion.demos.demo_plan_execute:main",
            "demo_collision = ur5_motion.demos.demo_collision:main",
            "demo_grasp = ur5_motion.demos.demo_grasp:main",
            "go_home = ur5_motion.demos.go_home:main",
            "demo_table_scene = ur5_motion.demos.demo_table_scene:main",
            "sync_scene = ur5_motion.demos.sync_scene:main",
            "world_sync = ur5_motion.world_sync_node:main",
            "isaac_gripper_action = ur5_motion.isaac_gripper_action:main",
            "sam_segmentation = ur5_motion.sam_segmentation:main",
        ],
    },
)
