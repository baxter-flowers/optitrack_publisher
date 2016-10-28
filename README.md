# Optitrack publisher

This package allows to publish optitrack markers declared as rigid bodies as TF transforms.
Data is gathered through the embedded VRPN server of Motive/Arena.
Only rigid bodies are requested to the server, thus single points in 2D/3D are ignored.
VRPN server can be enable in View > Data streaming in Motive.

## Dependencies
Install [VRPN for Python](https://github.com/vrpn/vrpn/wiki):
```
git clone https://github.com/vrpn/vrpn.git
cd vrpn/python
nano GNUmakefile
# set HW_OS := pc_linux64
make && sudo make install
```

## Publish
The optitrack publisher script publishes rigid bodies as tf::Transform(). Coordinates are directly
broadcasted without any additional transform, with a frame_id set to "optitrack_frame". Thus two tf
trees are present: one with the root being the robot's root ("base" or "base_link") and another one
with a root being "optitrack_frame".

The two trees are then connected thanks to a calibration procedure storing the robot->optitrack
transform on the parameter server /optitrack/calibration_matrix. Forgetting the calibration will
result in two unconnected trees.

First load the rigid body names to track from a YAML or manually:
```
rosparam set /optitrack/objects "[seat, table, pen, notebook]"
```
Then launch the publisher by specifying the IP of the the VRPN server (the PC running Motive/Arena):
```
roslaunch optitrack_publisher optitrack_publisher.launch ip:=<ip of the VRPN server> port:=<port of the VPRN server> world:=<name of the robot's world frame e.g. base, base_link...>
```

## Calibrate
Running the previous publishing without prior calibration will show an incorrect pose of the `optitrack_frame` within the robot's world frame.
This calibration operates by recording points in both frames (optitrack and robot frames) and matching them by minimizing a cost function.

You must first define a rigid body (e.g. named `/robot/calibrator` in Motive) firmly attached to the robot's end effector.
To calibrate please use the dedicated python notebook in the notebooks/ folder:

```
cd optitrack_publisher/notebooks
ipython notebook
```
