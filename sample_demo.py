# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 10:44:49 2021

@author: yhh18
"""

from topo_env import TOPOENV

env = TOPOENV()
action = [0,0,1,0,0,0,
          0,0,1,0,0,0,
          0,0,0,1,0,0,
          0,0,0,0,1,1,
          0,0,0,0,0,0,
          0,0,0,0,0,0]


stepIdx = 0
while True:

    print("---action: ", action)

    print("Step: ", stepIdx)
    obs, reward, done, _ = env.step(action)
    print("---pkgs, reward, done: ", sum(obs[:4]), reward, done)
    stepIdx += 1
    if done:
        stepIdx = 0
        break
reward = env.step(action)