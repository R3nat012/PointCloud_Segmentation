#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import time


class SavePoses(object):
    def __init__(self):
        
        self._pose = Pose()
        self.poses_dict = {}
        self._pose_sub = rospy.Subscriber('/odom', Odometry , self.sub_callback)

    def sub_callback(self, msg):
        
        self._pose = msg.pose.pose
    
    def write_to_file(self, count):
        
        key_str = 'pose' + str(count)
        self.poses_dict[key_str] = self._pose
        rospy.loginfo(f"Written {key_str}")
        
        with open('poses.txt', 'w') as file:
            
            for key, value in self.poses_dict.items():
                if value:
                    file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                    
        rospy.loginfo("Written all Poses to poses.txt file")



if __name__ == "__main__":
    rospy.init_node('spot_recorder', log_level=rospy.INFO)
    rate = rospy.Rate(1) # 10hz

    save_spots_object = SavePoses()
    count = 0
    while not rospy.is_shutdown():
        time.sleep(5)
        save_spots_object.write_to_file(count)
        count = count + 1
        rate.sleep()

    #rospy.spin() # mantain the service open.