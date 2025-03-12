



def get_dummy_dreamer_cfg():
    import rosnav_rl
    import rosnav_rl.model.dreamerv3.cfg as dreamerv3_cfg
    import rl_utils.cfg as arena_cfg
    
    return arena_cfg.TrainingCfg(
        arena_cfg=arena_cfg.ArenaBaseCfg(
            general=arena_cfg.GeneralCfg(
                n_envs=1,
                max_num_moves_per_eps=50,
                debug_mode=True,
            )
        ),
        agent_cfg=rosnav_rl.AgentCfg(
            name=None,
            framework=dreamerv3_cfg.DreamerV3Cfg(),
            reward=rosnav_rl.RewardCfg(
                reward_function_dict={
                    "goal_reached": {"reward": 15},
                    "factored_safe_distance": {"factor": -0.2},
                    "collision": {"reward": -15},
                    "approach_goal": {
                        "pos_factor": 0.3,
                        "neg_factor": 0.4,
                        "_goal_update_threshold": 0.25,
                        "_follow_subgoal": True,
                        "_on_safe_dist_violation": True,
                    },
                }
            ),
        ),
        resume=False,
    )
    
def get_dummy_sb3_cfg():
    import rl_utils.cfg as arena_cfg
    import rosnav_rl
    import rosnav_rl.model.stable_baselines3.cfg as sb3_cfg
    
    sb3 = sb3_cfg.StableBaselinesCfg(
        algorithm=sb3_cfg.PPO_Cfg(
            architecture_name="AGENT_1", parameters=sb3_cfg.PPO_Algorithm_Cfg()
        ),
        architecture_name="AGENT_1",
    )
    
    return arena_cfg.TrainingCfg(
        arena_cfg=arena_cfg.ArenaSB3Cfg(),
        agent_cfg=rosnav_rl.AgentCfg(
            framework=sb3,
            reward=rosnav_rl.RewardCfg(
                reward_function_dict={
                    "goal_reached": {"reward": 15},
                    "factored_safe_distance": {"factor": -0.2},
                    "collision": {"reward": -15},
                    "approach_goal": {
                        "pos_factor": 0.3,
                        "neg_factor": 0.4,
                        "_goal_update_threshold": 0.25,
                        "_follow_subgoal": True,
                        "_on_safe_dist_violation": True,
                    },
                }
            ),
        ),
    )