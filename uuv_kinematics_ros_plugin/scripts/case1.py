#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# import matplotlib.pyplot as plt

def command(startTime):
    while not rospy.is_shutdown():
        if rospy.get_time()-startTime >= 1:  # initiate after 2 seconds

            # dummy initial command
            pose = Pose()
            pub.publish(pose)
            time.sleep(1)

            print("\n----- Commanding the Pose ROS msg ------")
            pose = Pose()
            
            # ----------- Calculation -------------- #
            surge, sway, heave, roll, pitch, yaw = 0, 0, 0, 0, 0, 0

            rate = 1.0  # rate of surge motion in units/s
            time_step = 0.1  # time step in seconds
            duration = 10  # duration of simulation in seconds

            steps = int(duration / time_step)  # calculate the number of steps

            for _ in range(steps):

                # -------------------------------------- #
                # -------------------------------------- #
                # calculate uuv 6DOF pose

                surge += rate * time_step
                sway += rate/2.0 * time_step
                yaw += 0.05*rate * time_step
                pitch -= 0.05*rate * time_step

                # -------------------------------------- #
                # -------------------------------------- #
                pose.position = Point(surge, sway, heave)
                (q1, q2, q3, q4) = quaternion_from_euler(roll, pitch, yaw)
                pose.orientation.x = q1
                pose.orientation.y = q2
                pose.orientation.z = q3
                pose.orientation.w = q4

                rospy.loginfo(pose)
                pub.publish(pose)
                time.sleep(time_step)

            # publish last pose to stop at it's position
            pub.publish(pose)
            pub.publish(pose)
            pub.publish(pose)
            # Shutdown
            rospy.signal_shutdown('\n\nDONE!')

        # rate.sleep()

if __name__ == '__main__':
    try:
        # start node
        pub = rospy.Publisher('/uuv_kinematics/pose', Pose, queue_size=10)
        rospy.init_node('commander', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        startTime = rospy.get_time()

        # send command
        command(startTime)

    except rospy.ROSInterruptException:
        pass
