#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
import math

#Control Class
class Control:
    Kp = 1.0
    Ki = 0
    Kd = 0

#Class to store pose
class Pose:
    x = 0
    y = 0
    theta = 0

#Object initialization
GPose = Pose()
RPose = Pose() 

vControl = Control()
wControl = Control()

#Function to update robot pose
def update_pose(msg):
    global RPose
    RPose.x = msg.linear.x
    RPose.y = msg.linear.y
    RPose.theta = msg.angular.z
    

def get_distance(x1,y1,x2,y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

  
def main():

    global RPose,GPose,vControl,wControl
    
    rospy.init_node('go_to_goal_node')

    rate = rospy.Rate(10)
 
    control_msg = Twist()

    #Takes the user input
    if len(sys.argv) == 4:
        GPose.x = float(sys.argv[1])
        GPose.y = float(sys.argv[2])
        GPose.theta = float(sys.argv[3])
    else:
        print ("%s [x y theta]"%sys.argv[0])
        sys.exit(1)

    #Loading the parameters
    dist_error_threshold = rospy.get_param("dist_error_threshold")
    distance_threshold = rospy.get_param("distance_threshold")
    angular_threshold = rospy.get_param("angular_threshold")
    ep_angle_error_threshold = rospy.get_param("ep_angle_error_threshold")
    
    #Publisher to publish to the topic "cmd/vel"
    control_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

    #Subscriber to listen to the topic "pozyx_position"
    rospy.Subscriber('/pozyx_position',Twist,update_pose) 

    goal_x = GPose.x
    goal_y = GPose.y
    goal_theta = GPose.theta

    prev_distance_error = get_distance(goal_x,goal_y,0,0)
    prev_theta_error = 0
 
    while not rospy.is_shutdown():
	 
        robot_x = RPose.x
        robot_y = RPose.y
        robot_theta = ((360 - RPose.theta)*math.pi/180)%(2*math.pi)
        

        #Calculating the error in the angle of heading
        theta_error = (math.atan2((goal_y-robot_y),(goal_x-robot_x))+2*math.pi)%(2*math.pi) - robot_theta
        theta_error = (theta_error + math.pi)%(2*math.pi) - math.pi
        print('robot theta',robot_theta*180/math.pi)
        print('theta error',theta_error*180/math.pi)
	
        #The distance error
        distance_error = get_distance(robot_x,robot_y,goal_x,goal_y)
     
	
        #Filtering for any large changes in the error
        if(abs(prev_distance_error-distance_error) > dist_error_threshold):
            distance_error = prev_distance_error       
        
            
        #Aligning the angle of travel and varying the angular velocity
        if(abs(theta_error) >= angular_threshold and distance_error >= distance_threshold):
            control_msg.angular.z = 0.5*abs(theta_error)/theta_error
        else:
            control_msg.angular.z = 0  

        #Varying the linear velocity
        if(distance_error >= distance_threshold):
            control_msg.linear.x = 0.1
            
        else:
            control_msg.linear.x = 0
            #Turning the robot to align in the desired orientation 
            """   
            if(abs(goal_theta-robot_theta) >= ep_angle_error_threshold):
                control_msg.angular.z = 0.75
            else:
                control_msg.angular.z = 0
                return  
	        """
        control_pub.publish(control_msg)
        rate.sleep()
        
        prev_distance_error = distance_error
        prev_theta_error = theta_error
	

if __name__ == "__main__":
    try:
        main()
        print("Program Ended")
    except rospy.ROSInterruptException:
        pass    

