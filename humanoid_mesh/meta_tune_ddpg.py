import ray
from ray import tune
import ray.rllib.agents.ddpg as ddpg
import seagul.envs
import yaml


# settings = yaml.load(open('pend_td3.yaml'))
# config = settings['pendulum-ddpg']['config']


config = ddpg.DEFAULT_CONFIG.copy()
# config["num_workers"] = 0
# config["num_envs_per_worker"] = 6
# config["num_gpus"] = 0
# config["eager"] = False
# config["actor_hiddens"] = [64, 64]
# config["critic_hiddens"] = [64,64]
# config["n_step"] : 1
# config["model"] :
# config['observation_filter'] = 'MeanStdFilter'
# #config["train_batch_size"] = tune.sample_from(lambda spec: spec.config.sgd_minibatch_size*32)
# config["seed"] = tune.grid_search([0,1,2,3,4,5,6,7])
# env_name = "Walker2d-v3"
# env_name =  "Walker2DBulletEnv-v0"
# env_name =   "HumanoidBulletEnv-v0"
env_name = "Humanoid-v2"
# env_name = "sym_pendulum-v0"#
# env_name = "dt_pendulum-v0"

config["env"] = env_name
config["learning_starts"] = 10000
config["pure_exploration_steps"] = 10000
# config["evaluation_interval"] = 5
# config["evaluation_num_episode"] = 10
config["seed"] = tune.grid_search([0, 1, 2, 3])
config["num_workers"] = 15
config["num_gpus"] = 1

# import pprint
# pprint.pprint(config)

# from seagul.rllib.sym_ppo_policy import PPOTFSymPolicy
# from seagul.rllib.symrew_ppo_policy import PPOTFSymRewPolicy
# PPOSymTrainer = ppo.PPOTrainer.with_updates(name="SymPPO", default_policy = PPOTFSymPolicy)
# PPOSymRewTrainer = ppo.PPOTrainer.with_updates(name="SymRewPPO", default_policy = PPOTFSymRewPolicy)

analysis = tune.run(
    #    PPOSymRewTrainer,
    # PPOSymTrainer,
    ddpg.DDPGTrainer,
    config=config,
    stop={"timesteps_total": 5e7, "episode_reward_mean": 6000},
    local_dir="./data/bench_model/",
    checkpoint_at_end=True,
)
