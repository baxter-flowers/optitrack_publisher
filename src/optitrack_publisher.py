#!/usr/bin/env python

try:
    import vrpn
except ImportError:
    raise ImportError("ImportError: vrpn. Please install VRPN bindings for python: https://github.com/vrpn/vrpn")

import sys
import rospy
from tf import TransformBroadcaster
from collections import namedtuple

TrackedObject = namedtuple('TrackedObject', ('position', 'quaternion', 'timestamp'))

class OptiTrackClient():
    def __init__(self, addr, port, obj_names, world, dt, rate=120):
        """
        Class tracking optitrack markers through VRPN and publishing their TF frames as well as the transformation
        from the robot's world frame to the optitrack frame
        :param addr: IP of the VRPN server
        :param port: Port of the VRPN server
        :param obj_names: Name of the tracked rigid bodies
        :param world: Name of the robot's world frame (base_link, map, base, ...)
        :param dt: Delta T in seconds whilst a marker is still considered tracked
        :param rate: Rate in Hertz of the publishing loop
        """
        self.rate = rospy.Rate(rate)
        self.obj_names = obj_names
        self.world = world
        self.dt = rospy.Duration(dt)
        self.tfb = TransformBroadcaster()

        self.trackers = []
        for obj in obj_names:
            t = vrpn.receiver.Tracker('{}@{}:{}'.format(obj, addr, port))
            t.register_change_handler(obj, self.handler, 'position')
            self.trackers.append(t)

        self.tracked_objects = {}

    @property
    def recent_tracked_objects(self):
        """ Only returns the objects that have been tracked less than 20ms ago. """
        f = lambda name: (rospy.Time.now() - self.tracked_objects[name].timestamp)
        return dict([(k, v) for k, v in self.tracked_objects.iteritems() if f(k) < self.dt])

    def handler(self, obj, data):
        self.tracked_objects[obj] = TrackedObject(data['position'],
                                                  data['quaternion'],
                                                  rospy.Time.now())

    def run(self):
        while not rospy.is_shutdown():
            for t in self.trackers:
                t.mainloop()

            # Publish the calibration matrix
            calib_matrix = rospy.get_param("/optitrack/calibration_matrix")
            self.tfb.sendTransform(calib_matrix[0], calib_matrix[1], rospy.Time.now(), "optitrack_frame", self.world)

            # Publish all other markers as frames
            for k, v in self.recent_tracked_objects.iteritems():
                self.tfb.sendTransform(v.position, v.quaternion, v.timestamp, k, "optitrack_frame")

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('optitrack_publisher', anonymous=True)
    try:
        vrpn_ip = rospy.get_param("/optitrack/vrpn_ip")
        vrpn_port = rospy.get_param("/optitrack/vrpn_port")
        world = rospy.get_param("optitrack/world_frame")
        dt = rospy.get_param("optitrack/delta_t")
    except KeyError:
        rospy.logerr("Please use the launch file to start this node so that configuration parameters are loaded")
        raise

    try:
        objects = rospy.get_param("/optitrack/objects")
    except KeyError:
        rospy.logerr("Please load the names of the tracked rigid bodies on the parameter server: /optitrack/objects. "
                     "Use rosparam set /optitrack/objects '[rigid_body1, rigid_body2]' or load a YAML to /optitrack/objects")
        sys.exit(1)

    OptiTrackClient(vrpn_ip, vrpn_port, objects, world, dt).run()
