# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

# ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task=Isaac-Velocity-Flat-T1-v0 --checkpoint /home/dolores/IsaacLab/logs/rsl_rl/t1_flat/2025-05-15_20-25-42/model_2999.pt --num_envs 10

gym.register(
    id="Isaac-Velocity-Flat-T1-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:T1FlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:T1FlatPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Velocity-Rough-T1-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:T1RoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:T1RoughPPORunnerCfg",
    },
)