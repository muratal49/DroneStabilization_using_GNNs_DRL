import numpy as np
from gym_pybullet_drones.envs.BaseRLAviary import BaseRLAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType

class CurriculumHoverAviary_july31(BaseRLAviary):
    """Custom HoverAviary with manual KF/KM fix and curriculum training support."""

    def __init__(self,
                 drone_model: DroneModel = DroneModel.CF2X,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics = Physics.PYB,
                 pyb_freq: int = 240,
                 ctrl_freq: int = 30,
                 gui=False,
                 record=False,
                 obs: ObservationType = ObservationType.KIN,
                 act: ActionType = ActionType.RPM,
                 wind_speed_range=(0, 0),
                 wind_angle_range=(0, 0),
                 payload_range=(0, 0),
                 tumbling_range=(0, 0)):
        
        self.wind_speed_range = wind_speed_range
        self.wind_angle_range = wind_angle_range
        self.payload_range = payload_range
        self.tumbling_range = tumbling_range
        self.TARGET_POS = np.array([0, 0, 30])  # ✅ stabilize anywhere above 30m
        self.EPISODE_LEN_SEC = 10

        super().__init__(drone_model=drone_model,
                         num_drones=1,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         pyb_freq=pyb_freq,
                         ctrl_freq=ctrl_freq,
                         gui=gui,
                         record=record,
                         obs=obs,
                         act=act)

        # ✅ Fix KF and KM manually
        self.KF = 3.16e-10
        self.KM = 7.94e-12
        print(f"✅ Manually set KF={self.KF}, KM={self.KM}")

    ################################################################################
    def _computeReward(self):
        state = self._getDroneStateVector(0)
        z = state[2]
        # ✅ Reward for being high (>30m) and stable
        reward = -abs(30 - z) + 5.0 * max(0, z - 30)
        return reward

    def _computeTerminated(self):
        return False  # We let it run until timeout

    def _computeTruncated(self):
        state = self._getDroneStateVector(0)
        if abs(state[0]) > 3 or abs(state[1]) > 3 or state[2] > 350:
            return True
        if self.step_counter / self.PYB_FREQ > self.EPISODE_LEN_SEC:
            return True
        return False

    def _computeInfo(self):
        return {"info": 42}
