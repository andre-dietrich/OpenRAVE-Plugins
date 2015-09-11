#!/usr/bin/env python
__builtins__.__openravepy_version__ = '0.9'

from openravepy import *
from numpy import *
import os
import matplotlib.pyplot as plt
from scipy import ndimage

#RaveInitialize()
#RaveLoadPlugin('../lib/filter')

env=Environment()
xml = os.path.dirname(__file__)
if xml == "": xml = "."
env.Load(xml+"/myscene.env.xml")

for sensor in env.GetSensors():
    sensor.Configure(Sensor.ConfigureCommand.PowerOn)
    sensor.Configure(Sensor.ConfigureCommand.RenderDataOn)

env.SetViewer('qtcoin')

sensor = env.GetSensors()[0]
sensor.Configure(Sensor.ConfigureCommand.PowerOff)
sensor.Configure(Sensor.ConfigureCommand.RenderDataOff)

Filter = RaveCreateModule(env,'sensorgridmap')
Filter.SendCommand('SetTranslation -2.5 -2.5 0.1')
Filter.SendCommand('SetSize 25 25 0.2')
Filter.SendCommand('SetLineWidth 2.0')

robot = env.GetRobots()[0]
env.UpdatePublishedBodies()

plt.ion()
plt.show()

Filter.SendCommand('Render')
map = Filter.SendCommand('Scan')

matrix = numpy.fromstring(map, dtype=int, count=-1, sep=' ').reshape(25,25).astype(numpy.float)

print matrix

plt.imshow(matrix.astype(numpy.float), cmap=plt.cm.hot)
plt.draw()

raw_input("\nPress Enter to exit ...\n")
