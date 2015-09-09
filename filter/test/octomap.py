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

Filter = RaveCreateModule(env,'octomap')
Filter.SendCommand('SetTranslation -2.5 -2.5 0.5')
Filter.SendCommand('SetSize 5 5 2 2')

robot = env.GetRobots()[0]
env.UpdatePublishedBodies()

Filter.SendCommand('Render')
map = Filter.SendCommand('Scan')
Filter.SendCommand('Render')

#matrix = numpy.fromstring(map, dtype=bool, count=-1, sep='').reshape(26, 26)

#plt.ion()
#plt.show()
#plt.clf()
#plt.imshow(matrix.astype(numpy.float), cmap=plt.cm.gray)
#plt.draw()

#fig = plt.figure()
#ax = Axes3D(fig)
#verts = [zip(X, Y,Z)]
#ax.add_collection3d(Poly3DCollection(verts))
#plt.show()

raw_input("\nPress Enter to exit ...\n")