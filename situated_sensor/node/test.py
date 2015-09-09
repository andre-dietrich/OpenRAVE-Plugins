#!/usr/bin/env python
__builtins__.__openravepy_version__ = '0.9'

from openravepy import *
import time
import os

#RaveInitialize()
#RaveLoadPlugin('../lib/situated_sensor')

env=Environment()

xml = os.path.dirname(__file__)
if xml == "": xml = "."
env.Load(xml+"/test.env.xml")

env.SetViewer('qtcoin')

print env.GetSensors()

#sensor = env.GetSensor("cam")

#sensor.Configure(Sensor.ConfigureCommand.PowerOn)

#while True:
#    print sensor.GetSensorData().positions, sensor.GetSensorData().ranges, sensor.GetSensorData().intensity
#    print sensor.SendCommand('collidingbodies')
#    time.sleep(1)
#DistanceSensor = RaveCreateSensor(env,'DistanceSensor')

#print DistanceSensor.SendCommand('help')

raw_input('press ENTER to continue...')
