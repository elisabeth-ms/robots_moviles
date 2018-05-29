#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header
from math import sqrt,pow
import tf

class MeasureDistance:
    def __init__(self):
        self.first = True
        self.distance = 0
        r = rospy.Rate(10)

        self.listener = tf.TransformListener()
        r.sleep()
        r.sleep()

        while not rospy.is_shutdown():
            print "Comenzamos a medir"
            if not self.first:
                try:
                    (trans, rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                    self.current_x = trans[0]
                    self.current_y = trans[1]
                    print self.current_x, self.current_y
                    self.distance = self.distance + sqrt(pow((self.current_x - self.previous_x),2)+pow((self.current_y-self.previous_y),2))
                    print "Distancia recorrida:",self.distance
                    self.previous_x = self.current_x
                    self.previous_y = self.current_y
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            else:
                try:
                    (trans, rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                    self.previous_x = trans[0]
                    self.previous_y = trans[1]
                    print self.previous_x, self.previous_y
                    self.first = False
                    print self.first
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            r.sleep()




if __name__ == '__main__':
    rospy.init_node('Measure_distance_node', anonymous=True)
    try:
        measuredistance = MeasureDistance()
    except rospy.ROSInterruptException:
        pass
