#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
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
        self.pose_pub = rospy.Publisher('/goal', Pose, queue_size=10)
        self.mapdef = False
        self.locdef = False
        self.seenCube = False
        self.arucoList = []
        self.ObjectList = []
        self.seenBox = False



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
        pose=Pose()
        pose.position = self.ObjectList[0].points
        pose.orientation = self.ObjectList[0].orientation
        self.pose_pub.publish(pose)
        pose2=Pose()
        pose2.position = self.arucoList[0].pose.pose.position
        pose2.orientation = self.arucoList[0].pose.pose.orientation
        self.pose_pub.publish(pose2)
        rospy.loginfo("I have now published pose")


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