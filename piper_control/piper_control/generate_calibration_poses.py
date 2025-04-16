import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

# Base pose
base_pos = np.array([0.06129656, 0.00032804, 0.33907518])
base_quat = np.array([-0.00117427, 0.72152044, 0.00096871, 0.69239146])
base_rot = R.from_quat(base_quat)

# Perturbations
delta_xyz = [
    [ 0.00,  0.00,  0.00],  # base (center)
    [ 0.05,  0.05,  0.05],  # top-left
    [-0.05,  0.05,  0.05],  # top-right
    [ 0.05, -0.05,  0.05],  # bottom-left
    [-0.05, -0.05,  0.05],  # bottom-right
    [ 0.07,  0.00,  0.00],  # side-left
    [-0.07, 0.00,  0.00],   # side-right
    [ 0.00,  0.07,  0.00],  # top
    [ 0.00, -0.07,  0.00],  # bottom
]

delta_rpy_deg = [
    [ 0.0,   0.0,   0.0],   # base
    [15.0,  15.0,   0.0],
    [-15.0, 15.0,   0.0],
    [15.0, -15.0,   0.0],
    [-15.0,-15.0,   0.0],
    [20.0,   0.0,   0.0],
    [-20.0,  0.0,   0.0],
    [0.0,   20.0,   0.0],
    [0.0,  -20.0,   0.0],
]

# poses = []

# Generate poses
# for i in range(9):
#     delta_pos = np.array(delta_xyz[i])
#     delta_rot = R.from_euler('xyz', np.radians(delta_rpy_deg[i]))
#     new_pos = base_pos + delta_pos
#     new_rot = delta_rot * base_rot
#     new_quat = new_rot.as_quat()  # [x, y, z, w]
    
#     print(f"Pose {i+1}:")
#     print(f"  Position : x={new_pos[0]:.6f}, y={new_pos[1]:.6f}, z={new_pos[2]:.6f}")
#     print(f"  Quaternion: qx={new_quat[0]:.6f}, qy={new_quat[1]:.6f}, qz={new_quat[2]:.6f}, qw={new_quat[3]:.6f}")
#     print()

    # pose = {
    #     f'pose_{i+1}': {
    #         'position': {
    #             'x': float(new_pos[0]),
    #             'y': float(new_pos[1]),
    #             'z': float(new_pos[2])
    #         },
    #         'orientation': {
    #             'x': float(new_quat[0]),
    #             'y': float(new_quat[1]),
    #             'z': float(new_quat[2]),
    #             'w': float(new_quat[3])
    #         }
    #     }
    # }
    # poses.append(pose)

    # # Save to YAML
    # with open('handeye_calibration_poses.yaml', 'w') as f:
    #     yaml.dump({'poses': poses}, f, sort_keys=False)
pose_strings = []

# Generate cuPose-formatted strings
for i in range(9):
    delta_pos = np.array(delta_xyz[i])
    delta_rot = R.from_euler('xyz', np.radians(delta_rpy_deg[i]))
    new_pos = base_pos + delta_pos
    new_rot = delta_rot * base_rot
    new_quat = new_rot.as_quat()  # [x, y, z, w]

    # Format: wxyz for cuRobo
    cu_quat = [new_quat[3], new_quat[0], new_quat[1], new_quat[2]]

    pose_str = (
        f"goal_pose_{i+1}:\n"
        f"  position = torch.tensor([[{new_pos[0]:.6f}, {new_pos[1]:.6f}, {new_pos[2]:.6f}]], dtype=torch.float32, device='cuda:0'),\n"
        f"  quaternion = torch.tensor([[{cu_quat[0]:.6f}, {cu_quat[1]:.6f}, {cu_quat[2]:.6f}, {cu_quat[3]:.6f}]], dtype=torch.float32, device='cuda:0')"
    )
    pose_strings.append(pose_str)

# Save to a YAML-like text file
yaml_path = "cuRobo_goal_poses_formatted.yaml"
with open(yaml_path, 'w') as f:
    f.write("\n\n".join(pose_strings))
