#!/usr/bin/env python
__builtins__.__openravepy_version__ = '0.9'

from openravepy import *
from simplenavigation import SimpleNavigationPlanning
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

env.SetViewer('qtcoin')

Filter1 = RaveCreateModule(env,'occupancygridmap')
Filter1.SendCommand('SetTranslation -2.5 -2.5 0.1')
Filter1.SendCommand('SetSize 25 25 0.2')
Filter1.SendCommand('SetLineWidth 2.0')

Filter2 = RaveCreateModule(env,'occupancygridmap')
Filter2.SendCommand('SetTranslation -2.5 -2.5 0.7')
Filter2.SendCommand('SetSize 25 25 0.2')
Filter2.SendCommand('SetLineWidth 2.0')

robot = env.GetRobots()[0]
env.UpdatePublishedBodies()

nav = SimpleNavigationPlanning(robot)

plt.ion()
plt.show()
while True:
    nav.performNavigationPlanning()
    while not nav.robot.GetController().IsDone():
        #while True:
        render = Filter1.SendCommand('Scan')
        Filter1.SendCommand('Render')
        matrix1 = numpy.fromstring(render, dtype=bool, count=-1, sep='').reshape(25,25)
        
        render = Filter2.SendCommand('Scan')
        Filter2.SendCommand('Render')
        matrix2 = numpy.fromstring(render, dtype=bool, count=-1, sep='').reshape(25,25)
        
        plt.clf()
        plt.subplot(121)
        plt.imshow(matrix1.astype(numpy.float), cmap=plt.cm.gray)
        plt.subplot(122)
        plt.imshow(matrix2.astype(numpy.float), cmap=plt.cm.gray)
        plt.draw()
        

        #width, height, resolution = Filter.SendCommand('GetSize').split(" ")
        #print width, height, resolution
        
        