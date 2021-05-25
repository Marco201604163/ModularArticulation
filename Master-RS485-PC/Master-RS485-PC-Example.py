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
refSpeed = 100.0
refPos = 150.0

# act.setSpeedPos(address, refSpeed, refPos)
# act.setSpeed(address, 110)
act.setPos(address, -10.0)

# act.getSpeedPos(address)

# act.setPIDparameters(address, 17.0, 22.3, 43.1)
# act.getPIDparameters(address)

# act.calibHomePosSensor(address)

# act.stopControl(address)

# Stop Coms
act.close()
