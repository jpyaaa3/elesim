"""Direct reverse-state curriculum with free PPO actions and lift smoothing."""
from workbench.research.experiments.hug_home_rl.environment import HomeEnv
from workbench.research.experiments.hug_reverse_rl.bank import ResetBank
from .task import LinearHugEnv
from .linear import validate_resets, contract


class LinearResetBank(ResetBank):
    def __init__(self,env,plan,path,directory):
        validate_resets(path['command'][plan['pool_ticks'],0])
        # A teacher replay is not a learned episode. It runs grasp -> Home;
        # charge the Home -> reset lower bound only after restoring a start.
        env.linear_enforced=False
        try:
            super().__init__(env,plan,path,directory)
        finally:
            env.linear_enforced=True
            env.reset_linear_history()
        self.report['linear_motion_contract']=contract()
        from .contracts import save
        save(self.directory/'validation.json',self.report)

    def restore(self,bins,ids):
        super().restore(bins,ids)
        self.env.reset_linear_history(ids)


class ReverseHomeEnv(HomeEnv):
    reset_bank_class = LinearResetBank
    core_env_class = LinearHugEnv

    def metadata(self):
        result=super().metadata()
        result.pop('forward_reference_sha256',None)
        result.update(reverse_reference_sha256=self.plan['reference_path_sha256'],
                      scripted_episode_actions=False)
        return result
