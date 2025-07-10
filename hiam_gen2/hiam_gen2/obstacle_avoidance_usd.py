#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
" This example moves the robot through a sequence of poses and dumps an animated usd."
# CuRobo
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.types.base import TensorDeviceType
from curobo.geom.types import WorldConfig, Cuboid, Mesh, Capsule, Cylinder, Sphere

import torch

def update_world_objects():
    cuboid_list = [
        Cuboid(name="obs_1", pose=[0.35, 0, 0.15, 1, 0, 0, 0], dims=[0.1, 0.1, 0.3]),
        # Cuboid(name="obs_2", pose=[0.5, 0, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5]),
        # Cuboid(name="obs_3", pose=[0.3, 0.3, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5]),
        # Cuboid(name="obs_4", pose=[0, 0.5, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5])
    ]
    sphere_list = []
    cylinder_list = [
        # Cylinder(
        #     name="cylinder_1",
        #     radius=0.01875,
        #     height=0.1,
        #     pose=[0.4625, 0.1125, 0.05, 1, 0, 0, 0]
        #     ),
        # Cylinder(
        #     name="cylinder_2",
        #     radius=0.01875,
        #     height=0.1,
        #     pose=[0.4625, 0.1875, 0.05, 1, 0, 0, 0]
        #     ),
    ]
    mesh_list = []
    
    world_model = WorldConfig(
        cuboid=cuboid_list,
        cylinder=cylinder_list,
        sphere=sphere_list,
        mesh=mesh_list,
    ).get_collision_check_world()
    
    return world_model

def pose_sequence_piper():
    # load ur5e motion gen:

    # world_file = "collision_table.yml"
    world_file = WorldConfig.from_dict(
            {
                'cuboid': {
                    'table': {
                        'pose': [0, 0, -0.05, 1, 0, 0, 0],  # x, y, z, qw, qx, qy, qz
                        'dims': [4.0, 4.0, 0.1],
                    }
                },
                # 'cuboid': {
                #     'table': {
                #         'pose': [0.35, 0, 0.15, 1, 0, 0, 0],  # x, y, z, qw, qx, qy, qz
                #         'dims': [0.1, 0.1, 0.3],
                #     }
                # },
            }
        )
    robot_file = "piper_with_gripper.yml"
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_file,
        world_file,
        interpolation_dt=(1 / 30),
    )

    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup(parallel_finetune=True)

    # world_model = update_world_objects()
    # motion_gen.update_world(world_model)

    # retract_cfg = motion_gen.get_retract_config()
    # start_state = JointState.from_position(retract_cfg.view(1, -1))
    # start_pose = Pose(
    #         position = torch.tensor([[0.2, 0.0, 0.2]], dtype=torch.float32, device='cuda:0'),
    #         quaternion = torch.tensor([[0.7071, 0.0, 0.7071, 0.0]], dtype=torch.float32, device='cuda:0')
    #    )
    tensor_args = TensorDeviceType()  # or motion_gen.tensor_args if inside a class

    start_state = JointState.from_position(
        tensor_args.to_device(torch.tensor([0.0, 1.308, -0.177, 0.0, -0.104, 0.0], dtype=torch.float32)).unsqueeze(0),
        joint_names=[
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ],
    )

    home_pose = [0.5, 0.0, 0.4, 0.7071, 0.0, 0.7071, 0.0]
    pose_1 = [0.2, 0.0, 0.2, 0.7071, 0.0, 0.7071, 0.0]

    pose_list = [home_pose, pose_1, home_pose]
    trajectory = start_state
    motion_time = 0
    for i, pose in enumerate(pose_list):
        goal_pose = Pose.from_list(pose, q_xyzw=False)
        start_state = trajectory[-1].unsqueeze(0).clone()
        start_state.velocity[:] = 0.0
        start_state.acceleration[:] = 0.0
        result = motion_gen.plan_single(
            start_state.clone(),
            goal_pose,
            plan_config=MotionGenPlanConfig(parallel_finetune=True, max_attempts=1),
        )
        if result.success.item():
            plan = result.get_interpolated_plan()
            trajectory = trajectory.stack(plan.clone())
            motion_time += result.motion_time
        else:
            print(i, "fail", result.status)
    print("Motion Time (s):", motion_time)
    # CuRobo
    from curobo.util.usd_helper import UsdHelper

    trajectory.position = trajectory.position.contiguous()

    UsdHelper.write_trajectory_animation(
        robot_file,
        motion_gen.world_model,
        start_state,
        trajectory,
        save_path="piper_sequence.usd",
        base_frame="/grid_world_1",
        flatten_usd=True,
        visualize_robot_spheres=True,
        dt=1.0 / 30.0,
    )


if __name__ == "__main__":
    pose_sequence_piper()
