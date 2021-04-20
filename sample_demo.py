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

reward = env.step(action)