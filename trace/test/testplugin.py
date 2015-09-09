#!/usr/bin/env python
__builtins__.__openravepy_version__ = '0.9'

from openravepy import *
from openravepy.examples.simplenavigation import SimpleNavigationPlanning
from numpy import *
import os

#RaveInitialize()
#RaveLoadPlugin('../lib/trace')

env=Environment()

xml = os.path.dirname(__file__)
if xml == "": xml = "."
env.Load(xml+"/myscene.env.xml")

env.SetViewer('qtcoin')

robot = env.GetRobots()[0]
env.UpdatePublishedBodies()

nav = SimpleNavigationPlanning(robot)


while True:
    nav.performNavigationPlanning()
    nav.robot.WaitForController(0)