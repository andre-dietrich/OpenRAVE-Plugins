#!/usr/bin/env python
__builtins__.__openravepy_version__ = '0.9'

from openravepy import *
from numpy import *
import os
from scipy import ndimage

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

#RaveInitialize()
#RaveLoadPlugin('../lib/filter')

env=Environment()
xml = os.path.dirname(__file__)
if xml == "": xml = "."
env.Load(xml+"/myscene.env.xml")
env.SetViewer('qtcoin')

for sensor in env.GetSensors():
    sensor.Configure(Sensor.ConfigureCommand.PowerOn)
    sensor.Configure(Sensor.ConfigureCommand.RenderDataOn)

Filter = RaveCreateModule(env,'sensorcubemap')
Filter.SendCommand('SetTranslation -2.5 -2.5 0.5')
Filter.SendCommand('SetSize 26 0.2 26 0.2 1 0.2')

robot = env.GetRobots()[0]
env.UpdatePublishedBodies()

Filter.SendCommand('Render')
map = Filter.SendCommand('Scan')
Filter.SendCommand('Render')

print map

raw_input("\nPress Enter to exit ...\n")
