#!/usr/bin/env python
__builtins__.__openravepy_version__ = '0.9'

from openravepy import *
from simplenavigation import SimpleNavigationPlanning
from numpy import *
import os
from scipy import ndimage

import matplotlib.pyplot as plt

#RaveInitialize()
#RaveLoadPlugin('../lib/filter')

env=Environment()
xml = os.path.dirname(__file__)
if xml == "": xml = "."
env.Load(xml+"/myscene.env.xml")

env.SetViewer('qtcoin')

robot = env.GetRobots()[0]
env.UpdatePublishedBodies()

nav = SimpleNavigationPlanning(robot)

Filter = RaveCreateModule(env,'quadtree')
Filter.SendCommand('SetTranslation -2.5 -2.5 0.74')
Filter.SendCommand('SetSize 5 5 6')

while True:
    nav.performNavigationPlanning()
    while not nav.robot.GetController().IsDone():
        Filter.SendCommand('Scan')
        Filter.SendCommand('Render')

raw_input("\nPress Enter to exit ...\n")