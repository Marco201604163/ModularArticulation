# -*- coding: utf-8 -*-
"""
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
"""
# -*- coding: utf-8 -*-

# Add parent directory to path so the example can run without the library being installed
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import actuatorComs as act

# Open port and Init Coms
act.init()

address = 1
refSpeed = 313.1
refPos = 424.2

act.setSpeedPos(address, refSpeed, refPos)
# act.setSpeed(address, 43.25)
# act.setPos(address, 67.89)

act.getSpeedPos(address)

act.setPIDparameters(1, 17.0, 22.3, 43.1)
# act.getPIDparameters(address)

# Stop Coms
act.stop()
