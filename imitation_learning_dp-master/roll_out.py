import sys

sys.path.append('./diffusion_policy')
sys.path.append('./imitation_learning_dp')

import time
import hydra
from omegaconf import OmegaConf
import pathlib
import numpy as np
import torch


@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        'imitation_learning_dp', 'config'))
)
def main(cfg: OmegaConf):
    OmegaConf.resolve(cfg)

    policy = hydra.utils.instantiate(cfg.policy)

    env = hydra.utils.instantiate(cfg.task.env)
    n_obs_steps = cfg.n_obs_steps
    n_action_steps = cfg.n_action_steps

    obs = env.reset()
    image = np.zeros((1, n_obs_steps, *obs['image'].shape))
    agent_pos = np.zeros((1, n_obs_steps, *obs['agent_pos'].shape))
    for i in range(n_obs_steps):
        image[0, i, ...] = obs['image']
        agent_pos[0, i, ...] = obs['agent_pos']
    observation = {}
    observation['image'] = image
    observation['agent_pos'] = agent_pos

    device = 'cuda:0'
    device = torch.device(device)
    policy.load_state_dict(torch.load("./outputs/model15.pth", map_location=device))
    policy.to(device)
    policy.eval()

    done = False
    step_num = 0

    while not done:
        step_start = time.time()

        if step_num % n_action_steps == 0:
            observation['image'][0, 0:-1, ...] = observation['image'][0, 1:, ...]
            observation['agent_pos'][0, 0:-1, ...] = observation['agent_pos'][0, 1:, ...]

            observation['image'][0, -1, ...] = obs['image']
            observation['agent_pos'][0, -1, ...] = obs['agent_pos']

            actions = policy.predict_action(observation)['action'].cpu().detach().numpy()

        action = actions[0, step_num % n_action_steps, :]
        obs, reward, done, info = env.step(action)

        env.mj_viewer.sync()
        step_num += 1
        time_until_next_step = env.mj_model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    env.mj_viewer.close()


if __name__ == "__main__":
    main()
