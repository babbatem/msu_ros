import time
import copy
import pathlib
import numpy as np
import motor_skills.core.mj_control as mjc
from mujoco_py import load_model_from_path, MjSim, MjViewer
#
# class MjJacoRosEnv(object):
#     """docstring for MjJacoEnv."""
#
#     def __init__(self, vis=False):
#         super(MjJacoRosEnv, self).__init__()
#         parent_dir_path = str(pathlib.Path(__file__).parent.absolute())
#         self.fname = parent_dir_path + '/kinova_j2s6s300/j2s6s300.xml'
#         self.model = load_model_from_path(self.fname)
#         self.sim = MjSim(self.model)
#         self.viewer = MjViewer(self.sim)
#         self.vis=vis
#
#     def step(self, action):
#         for i in range(len(action)):
#             self.sim.data.ctrl[i]=action[i]
#
#         self.sim.forward()
#         self.sim.step()
#         self.viewer.render() if self.vis else None
#         return self.sim.data.qpos

def main():
    parent_dir_path = str(pathlib.Path(__file__).parent.absolute())
    fname = parent_dir_path + '/kinova_j2s6s300/j2s6s300.xml'
    model = load_model_from_path(fname)
    sim = MjSim(model)
    viewer = MjViewer(sim)

    t = 0
    while True:

        if t == 1000:
            ndof = len(sim.data.qpos)
            captured_state = copy.deepcopy(sim.data.qpos)
            desired_vel = [0]*len(captured_state)
            kv = np.eye(ndof) * 10

        if t < 1000:
            sim.data.ctrl[:] = sim.data.qfrc_bias[:]

        else:
            sim.data.ctrl[:] = mjc.pd(None,desired_vel,captured_state, sim, ndof=len(captured_state), kv=kv)


        sim.forward()
        sim.step()
        viewer.render()
        t+=1


if __name__ == '__main__':
    main()
