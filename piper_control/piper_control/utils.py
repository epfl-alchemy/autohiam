import torch
import numpy as np
import quaternion

def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    R = np.array([
        [1 - 2 * y**2 - 2 * z**2,     2 * x * y - 2 * z * w,     2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w,       1 - 2 * x**2 - 2 * z**2,   2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w,       2 * y * z + 2 * x * w,     1 - 2 * x**2 - 2 * y**2]
    ])
    return R

def calculate_new_point(quaternion, point, distance):
    R = quaternion_to_rotation_matrix(quaternion)
    normal_vector = R[:, 2]
    new_point = point + distance * normal_vector
    return new_point

def plot_traj(trajectory, dt, file_name):
    # Third Party
    import matplotlib.pyplot as plt

    _, axs = plt.subplots(4, 1)
    q = trajectory.position.cpu().numpy()
    qd = trajectory.velocity.cpu().numpy()
    qdd = trajectory.acceleration.cpu().numpy()
    qddd = trajectory.jerk.cpu().numpy()
    timesteps = [i * dt for i in range(q.shape[0])]
    for i in range(q.shape[-1]):
        axs[0].plot(timesteps, q[:, i], label=str(i))
        axs[1].plot(timesteps, qd[:, i], label=str(i))
        axs[2].plot(timesteps, qdd[:, i], label=str(i))
        axs[3].plot(timesteps, qddd[:, i], label=str(i))

    plt.legend()
    plt.savefig(file_name)
    plt.close()
    plt.show()
    
def plot_traj_improved(trajectory, dt, file_name):
    # Third Party
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    
    q = trajectory.position.cpu().numpy()
    qd = trajectory.velocity.cpu().numpy()
    qdd = trajectory.acceleration.cpu().numpy()
    qddd = trajectory.jerk.cpu().numpy()
    timesteps = [i * dt for i in range(q.shape[0])]

    labels = ['Position graph', 'Velocity graph', 'Acceleration graph', 'Jerk graph']
    data = [q, qd, qdd, qddd]

    for i, ax in enumerate(axs):
        for j in range(q.shape[-1]):
            ax.plot(timesteps, data[i][:, j], label=f'Joint A {j+1}')
        ax.set_title(labels[i])
        ax.set_ylabel(labels[i])
        ax.legend(loc='upper right')
    
    axs[-1].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(file_name, dpi=300)
    plt.close()

    
def plot_iters_traj(trajectory, d_id=1, dof=7, seed=0):
    # Third Party
    import matplotlib.pyplot as plt

    _, axs = plt.subplots(len(trajectory), 1)
    if len(trajectory) == 1:
        axs = [axs]
    for k in range(len(trajectory)):
        q = trajectory[k]

        for i in range(len(q)):
            axs[k].plot(
                q[i][seed, :-1, d_id].cpu(),
                "r+-",
                label=str(i),
                alpha=0.1 + min(0.9, float(i) / (len(q))),
            )
    plt.legend()
    plt.show()


def plot_iters_traj_3d(trajectory, d_id=1, dof=7, seed=0):
    # Third Party
    import matplotlib.pyplot as plt

    ax = plt.axes(projection="3d")
    c = 0
    h = trajectory[0][0].shape[1] - 1
    x = [x for x in range(h)]

    for k in range(len(trajectory)):
        q = trajectory[k]

        for i in range(len(q)):
            # ax.plot3D(x,[c for _ in range(h)],  q[i][seed, :, d_id].cpu())#, 'r')
            ax.scatter3D(
                x, [c for _ in range(h)], q[i][seed, :h, d_id].cpu(), c=q[i][seed, :, d_id].cpu()
            )
            # @plt.show()
            c += 1
    plt.legend()
    plt.show()