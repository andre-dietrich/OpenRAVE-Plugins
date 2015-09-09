# OpenRAVE - Plugins

This repository hosts some of our plugins, developed for the OpenRAVE robotic
simulator. Currently there are three plugins available:

## 1. distance_sensor

This plugin was developed to use more realistic distance sensors within a
robotic simulation. Different shapes of sensor beams can be defined by using
simple meshes. The youBot random walk example in the video below shows the usage
of this OpenRAVE plugin. The front sensors have a typical infrared shape, while
side and back sensors have a typical utrasonic beam. Using meshes it is easy to
extend this plugin and to imitate various different distance sensors.

<iframe width="425" height="344" frameborder="0"
 src="https://www.youtube.com/embed/Ts7Acf70D8U">
</iframe>

## 2. filter

This plugin was intended to filter different simulation scenarios, and thus to
generate more abstract views. As you can see in the video, two grids were placed
to cut through the scenario and generate online different occupancy grid maps
for different heights. This plugin was intended to be extended and used as a
base for further filters.

<iframe width="425" height="344" frameborder="0"
 src="https://www.youtube.com/embed/DTX2pXk5Q2Q">
</iframe>

### 2.1 quadtrees

![screenshot1](misc/filter_quadtree.png)

### 2.2 sensorcoverage

![screenshot2](misc/filter_sensorcoverage.png)

### 2.3 sensordistance

![screenshot3](misc/filter_sensordistance.png)

### 2.4 occupancycubes

![screenshot4](misc/filter_occupancycubes.png)

### 2.5 and more ...


## 3. trace

Sometimes it is required to keep track of an object and its trajectory during a
simulation. This simple plugin was intended to do so. Just attach this element
like an sensor to any robot, link, or object and visualize historical positions.

<iframe width="425" height="344" frameborder="0"
 src="https://www.youtube.com/embed/f_TOxP75buk">
</iframe>

## 4. situated_sensor

This plugin allows to measure within the virtual world as well as in the real
world, by connecting a sensor to a ROS topic.

![screenshot4](misc/situated_sensor.png)

## Installation

Simply download the project via:

```bash
git clone https://gitlab.com/OvGU-ESS/OpenRAVE-Plugins.git
```
go into every directory, for example

```bash
cd distance_sensor
```

and then run

```bash
cmake .
sudo make install
```
every project contains a test folder with a python script.

## Contact

If you have further comments or remarks, go to our website and write a mail:

http://eos.cs.ovgu.de/crew/dietrich

or visit our blog at:

http://www.aizac.info
