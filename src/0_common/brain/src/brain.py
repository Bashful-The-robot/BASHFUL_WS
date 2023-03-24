#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import  PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
 
#import pose 
# create class that discretizes the workspace 
class Brain:
    def __init__(self):
        self.aruco_sub = rospy.Subscriber('/camera/aruco/markers', ArucoMarkerArray, self.aruco_callback)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.loc_sub = rospy.Subscriber('/state/cov', PoseWithCovarianceStamped, self.loc_callback)
        self.markerArr_sub = rospy.Subscriber('/ObjectList', MarkerArray, self.markerArr_callback)
        self.pose_pub = rospy.Publisher('/goal', MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
        self.mapdef = False
        self.locdef = False
        self.seenCube = False
        self.arucoList = []
        self.ObjectList = []
        self.seenBox = False
        self.rate = rospy.Rate(10)



    def tf_callback(self, msg):
        '''This should check if the map has been defined'''
        if self.mapdef:
            return
        if msg.transforms[0].header.frame_id == 'map':
            self.mapdef = True
            rospy.loginfo('Map defined')


    def loc_callback(self, msg):
        '''This should not publish until the map has been defined'''
        self.locdef = True


    def markerArr_callback(self, msg):
        '''Publishes all object found from detection'''
        if self.seenCube:
            return
        
        
        for marker in msg.markers:
            self.ObjectList.append(marker)
        self.seenCube = True

    def aruco_callback(self,msg):

        if self.seenBox:
            return

        self.marker = msg.markers
        for marker in self.marker:
            if marker.id == 500:
                self.arucoList.append(marker)
                self.seenBox = True
        
        
    def pub_goal(self):
        marker1 = Marker()
        marker1.header.frame_id = self.ObjectList[0].header.frame_id
        marker1.header.stamp = self.ObjectList[0].header.stamp
        marker1.points = self.ObjectList[0].points
        self.marker_array.markers.append(marker1)
        marker2 = Marker()
        marker2.header.frame_id = self.arucoList[0].header.frame_id
        marker2.header.stamp = self.arucoList[0].header.stamp
        marker2.points = self.arucoList[0].pose.pose.position
        self.marker_array.markers.append(marker2)

        self.pose_pub.publish(self.marker_array)
        rospy.loginfo("I have now published pose")
        self.rate.sleep()


def main():
    rospy.init_node('brain')
    brain = Brain()
    while not rospy.is_shutdown():
        #rospy.loginfo(len(brain.arucoList))
        #rospy.loginfo(len(brain.ObjectList))

        if len(brain.arucoList) != 0 and len(brain.ObjectList) != 0:
            rospy.loginfo("I have seen the box and the cube")
            brain.pub_goal()
    rospy.spin()





if __name__ == '__main__':
    main()