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
act.init("COM3")

address = 1
refSpeed = 100.0
refPos = 0.0

act.setCascade(address, refPos)
# act.setSpeed(address, 110)
# act.setPos(address, 0)

# act.getSpeedPos(address)
# act.getMotorTemp(address)

# act.calibHomePosSensor(address)

# act.stopControl(address)

# Stop Coms
act.close()
