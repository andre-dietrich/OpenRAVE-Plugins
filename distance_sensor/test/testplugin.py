#!/usr/bin/env python
__builtins__.__openravepy_version__ = '0.9'

from openravepy import *
from openravepy.examples.simplenavigation import SimpleNavigationPlanning
from numpy import *
import os

#RaveInitialize()
#RaveLoadPlugin('../lib/distancesensor')

env=Environment()

xml = os.path.dirname(__file__)
if xml == "": xml = "."
env.Load(xml+"/myscene.env.xml")

env.SetViewer('qtcoin')

robot = env.GetRobots()[0]
env.UpdatePublishedBodies()

nav = SimpleNavigationPlanning(robot)


# store the sensors
sensor = []
for i in range(5):
    sensor.append(env.GetSensor("YouBot_dist_"+str(i)))

while True:
    nav.performNavigationPlanning()
    while not nav.robot.GetController().IsDone():
        
        print "============================================================="
        for i in range(5):
            print "Measurement:", i
            print sensor[i].GetSensorData().positions, sensor[i].GetSensorData().ranges, sensor[i].GetSensorData().intensity
            print "colliding bodies " + sensor[i].SendCommand('collidingbodies')

#sensor = env.GetSensor("dist_0")
#while True:
#    print sensor.GetSensorData().positions, sensor.GetSensorData().ranges, sensor.GetSensorData().intensity
#    print "colliding bodies " + sensor.SendCommand('collidingbodies')
#    time.sleep(1)
  


