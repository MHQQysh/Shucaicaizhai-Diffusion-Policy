import sys

sys.path.append('./diffusion_policy')
sys.path.append('./imitation_learning_dp')

import hydra
from omegaconf import OmegaConf
import pathlib
import numpy as np
import zarr


def write_zarr(filename, images, states, actions, episode_ends):
    root = zarr.open(store=filename, mode='w')
    data_group = root.create_group('data')
    data_group.create_dataset('img', shape=images.shape, dtype=images.dtype,
                              chunks=(episode_ends[0], images.shape[1], images.shape[2], images.shape[3]))
    data_group.create_dataset('state', shape=states.shape, dtype=states.dtype,
                              chunks=(episode_ends[0], states.shape[1]))
    data_group.create_dataset('action', shape=actions.shape, dtype=actions.dtype,
                              chunks=(episode_ends[0], actions.shape[1]))
    data_group['img'][:] = images
    data_group['state'][:] = states
    data_group['action'][:] = actions

    meta_group = root.create_group('meta')
    meta_group.create_dataset('episode_ends', shape=(len(episode_ends),), dtype=np.int64, chunks=(len(episode_ends),))
    meta_group['episode_ends'][:] = episode_ends


@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        'imitation_learning_dp', 'config'))
)
def main(cfg: OmegaConf):
    OmegaConf.resolve(cfg)

    env = hydra.utils.instantiate(cfg.task.env)

    num = 50

    images = np.array([])
    states = np.array([])
    actions = np.array([])
    episode_ends = []

    for i in range(num):
        env.reset()
        data = env.run()
        if i == 0:
            images = data['imgs']
            states = data['states']
            actions = data['actions']
        else:
            images = np.vstack((images, data['imgs']))
            states = np.vstack((states, data['states']))
            actions = np.vstack((actions, data['actions']))
        episode_ends.append(images.shape[0])

    filename = './data/ur5_grab/ur5_grab.zarr'
    write_zarr(filename, images, states, actions, episode_ends)


if __name__ == '__main__':
    main()
