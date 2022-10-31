#importing the nescessary libraries 
import rospy #python library for ROS 
from std_msgs.msg import Int64 #importing Int64
from math import sin, cos, pi #importing the necessary library 
import tf #importing tf transforms for doing various transformations
from geometry_msgs.msg import  Twist #importing teist messgae from geometry messages
from geometry_msgs.msg import PoseWithCovarianceStamped #importig message for publish
import numpy as np #importing thr numericla python library
from localisation.srv import odom_reset,odom_resetResponse #importing the nescessary service request and response messages  

#class to keep track of the ticks which arise from the motor encoders
class ticks:

    #variables storing the previous left and right tick 
    prev_left_tick=0
    prev_right_tick=0

    #variables stroing the current left and right tick
    left_tick=0
    right_tick=0

#class to keep track of the position of pozyx (x,y ,theta)
class posyx:

    #variables storing the positiona nd orienstation related to pozyx
    x=0
    y=0 
    theta=0

#defining global variables to be used any where as needed 
Tick=ticks()
Posyx =posyx()
pose=(0,0,0)

#function to get the jacobian associated with the current state due to previous state 
def get_jacobian_state(pose,control,robo_width):

        #getting the current orientation and control inputs 
        theta = pose[2]
        left_tick, right_tick = control

        #defining the first case if the right control input is not equal to the left controll input 
        if right_tick != left_tick:

            #change in roeintation and radius of curvature of the turn 
            alpha = (right_tick - left_tick) / robo_width #change in orientation
            R = left_tick / alpha #radius of curvature 

            #derivative of x at t+1 and y at t+1 with respect to theta at t
            g1 = (R + (robo_width/ 2)) * (cos(theta + alpha) - cos(theta)) #derivative of x at t+1 with respect to theta at t
            g2 = (R + (robo_width/ 2)) * (sin(theta + alpha) - sin(theta)) #derivative of y at t+1 with respect to theta at t

            #the final jacobian matrix 
            m = np.array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])

        else:

            #the final jacobian matrix (simplified version of earlier case due to the fect r=l)
            m = np.array([[1.0, 0.0, -left_tick * sin(theta)], [0.0, 1.0, left_tick * cos(theta)], [0.0, 0.0, 1.0]])

        return m

#function to get the jacobian of the current state with respect ot the control inputs 
def get_jacobian_control(pose,control,robo_width):

        #getting the current orientation and control inputs 
        theta = pose[2]
        left_tick, right_tick = tuple(control)

        #defining the first case if the right control input is not equal to the left controll input 
        if right_tick != left_tick:

            #estimating the change in orientation
            alpha = (right_tick - left_tick) / (robo_width)

            #varioius terms which are useful for simplification 
            wr = ((robo_width) * right_tick) / ((right_tick - left_tick) ** 2)
            wl = ((robo_width) * left_tick) / ((right_tick - left_tick) ** 2)
            r2l = (right_tick + left_tick) / (2 * (right_tick - left_tick))

            #different terms in the jacobian matrix ( for left control)
            g1_l = wr * (sin(theta + alpha) - sin(theta)) - r2l * cos(theta + alpha) #derivative of x w.r.t left control
            g2_l = wr * (-cos(theta + alpha) + cos(theta)) - r2l * sin(theta + alpha) #derivative of y w.r.t left control
            g3_l = - ((1 / robo_width)) #derivative of theta w.r.t left control

            #different terms in the jacobian matrix ( for right control)
            g1_r = -wl * (sin(theta + alpha) - sin(theta)) + r2l * cos(theta + alpha) #derivative of x w.r.t right control
            g2_r = -wl * (-cos(theta + alpha) + cos(theta)) + r2l * sin(theta + alpha) #derivative of x w.r.t right control
            g3_r = 1 / (robo_width) #derivative of x w.r.t right control

            #the final jacobian matrix 
            m = np.array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        else:

            # This is for the special case l == r.
            g1_l = .5 * (cos(theta) + (left_tick / robo_width) * sin(theta)) #derivative of x w.r.t left control
            g2_l = .5 * (sin(theta) - (left_tick / robo_width) * cos(theta)) #derivative of y w.r.t left control
            g3_l = - 1 / (robo_width) #derivative of orientation  w.r.t left control

            g1_r = .5 * (((-left_tick / robo_width)) * sin(theta) + cos(theta)) #derivative of x w.r.t right control
            g2_r = .5 * (((left_tick /robo_width)) * cos(theta) + sin(theta)) #derivative of y w.r.t right control
            g3_r = 1 / (robo_width) #derivative of theta w.r.t right control

            #the final jacobian matrix (simplified version of earlier case due to the fect r=l)
            m = np.array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        return m

# 
def get_covariance(covariance,current_pose,control,control_motion_factor,control_turn_factor,robot_width):

    #extracting the left and right control input 
    left, right = control

    #exracting the motion and tuen factors
    alpha_1 = control_motion_factor
    alpha_2 = control_turn_factor

    #defining the covariances associated with x the left and right componenet of the control
    sigma_l = (alpha_1 * left) ** 2 + (alpha_2 * (left - right)) ** 2 # covariance propotional to left and to difference in left and right control
    sigma_r = (alpha_1 * right) ** 2 + (alpha_2 * (left - right)) ** 2 # covariance propotional to right and to difference in left and right control

    control_covariance = np.diag([sigma_l, sigma_r]) # froming a diagonal matrix using the control covariance 

    G_t = get_jacobian_state(current_pose, control, robot_width) # getting covariance in teh current sttae due to previous state

    V   = get_jacobian_control(current_pose, control,robot_width) #getting theh covariance due to the control factor 

    covariance = np.dot(G_t, np.dot(covariance, G_t.T)) + np.dot(V, np.dot(control_covariance, V.T)) # updating the covariance as the sum of covariance due to previous stse and the control input 

    return covariance # returning thr updated covariance 

#function to update the curretnn pose bsed on previous pose and control input 
def pose_update(tick_difference,ticks_to_meter,width_robo,scanner_displacement):
	global pose

	#first case robot travels in straight line 
	if tick_difference[0]==tick_difference[1]:

		theta = pose[2] #orientation remains same 
		x = pose[0]+tick_difference[0]*ticks_to_meter*cos(theta) #updating x 
		y = pose[1]+tick_difference[1]*ticks_to_meter*sin(theta) #updating y 
		
		#returning the pose
		return (x,y,theta)


	#second case in case of a curve
	else:

		#getting the previous parameters
		theta = pose[2]
		x = pose[0]-scanner_displacement*sin(theta)
		y = pose[1]-scanner_displacement*cos(theta)
		

        #change in roeintation and radius of curvature of the turn
		alpha = ticks_to_meter*(tick_difference[1]-tick_difference[0])/width_robo# change in orientation
		R = ticks_to_meter*tick_difference[0]/alpha #radius of curvature
		

        #calulating the center of curvature 
		centerx = x-(R+width_robo/2)*sin(theta) # x coordinate 
		centery = y+(R+width_robo/2)*cos(theta) # y coordinate
		theta = (theta + alpha + pi) % (2*pi) - pi # theta the roeinataion is normalised to be in the range from 0 to 2pi 
		
		#updating the x and using newly calualted theta value 
		x = centerx+(R+width_robo/2)*sin(theta)+scanner_displacement*sin(theta) # caluating the new x
		y = centery-(R+width_robo/2)*cos(theta)+scanner_displacement*cos(theta) # cacualting the new y
		
		
		return (x,y,theta) # returning the pose 

#call back function to update left tick    
def callback_left(msg):

    global Tick #acessing the global class tick 

    Tick.left_tick=msg.data #updating the left_tick

def callback_right(msg):

    global Tick #acessing the global class tick 

    Tick.right_tick=msg.data #updating the right tick 

def callback_posyx(msg):

    global Posyx #acessing the global class pozyx

    Posyx.x=msg.linear.x #updating the x position
    Posyx.y=msg.linear.y #updating the y position
    Posyx.theta=msg.angular.z #updating the z pisiton

def set_odometry(req):

    global pose #accessing the global variable pose 

    pose=(req.x,req.y,req.theta) #getting the required variable x , y, theta .

    return odom_resetResponse(True) #setting the response variable in the srv file to be true 

def main():

    #acessing the global variables which need to be used 
    global Tick
    global Posyx
    global pose

    #creating a publisher topic named pose and a node named pose_update 
    odom_pub = rospy.Publisher("odom1",PoseWithCovarianceStamped, queue_size=1)

    #creating tf broadcasters
    odom_broadcaster = tf.TransformBroadcaster() #tf broadcasters for odometry
    lidar_broadcaster = tf.TransformBroadcaster() #tf broadcasters for lidar
    posyx_transform = tf.TransformBroadcaster() #tf broadcasters for pozyx

    #intialising the odom_node
    rospy.init_node('odom_node')

    #creating a service called reset_odometry to seta pose to odometry intially or at any other time
    s = rospy.Service('reset_odometry', odom_reset, set_odometry)


    #initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) # 10hz
  
    #different constants which are used in various calculations
    ticks_to_millimeter = rospy.get_param("ticks_to_millimeter") #in mm per tick
    width_robo = rospy.get_param("robot_width")  #in mm
    scanner_displacement = rospy.get_param("scanner_displacement")   #in mm
    control_motion_factor =  rospy.get_param("control_motion_factor") #motion factor which relates to the error due to stright line motion
    control_turn_factor = rospy.get_param("control_turn_factor") #turn factor which relates to the error due to turning moiton
   
    #begining pose estimation
    print("Starting Pose estimation")

   #intialising the covariance as a large value to see the effect of filtering being prominent 
    covariance = np.diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])

    #different subscribers involved 
    rospy.Subscriber('/left_ticks', Int64, callback_left) #subscriber to subscribe to the left motor_ticks
    rospy.Subscriber('/right_ticks', Int64, callback_right) #subscriber to subscribe to the left motor_ticks
    rospy.Subscriber("/position",Twist,callback_posyx) #subscriber to subscribe to the posyzx position

    #loop for rospy
    while not rospy.is_shutdown():

        current_time = rospy.Time.now() #getting the current time for using it for transform  
        
<<<<<<< HEAD
        tick_difference=[0,0] #intialisng the tick difference array which is the control input
=======
        rospy.Subscriber('/left_ticks', Int64, callback_left)
        rospy.Subscriber('/right_ticks', Int64, callback_right)
        rospy.Subscriber("/pozyx_position",Twist,callback_posyx)
>>>>>>> 9e52596ffdf8fbba7384e509d067af25ae042da4

        tick_difference[0] = Tick.left_tick-Tick.prev_left_tick #assigning the left tick difference 
        tick_difference[1] = Tick.right_tick-Tick.prev_right_tick #assigning the right tick difference 

        Tick.prev_left_tick = Tick.left_tick #updating the left tick variable in the class
        Tick.prev_right_tick = Tick.right_tick #updating the right tick variable in the class
        
        control = np.array((tick_difference[0],tick_difference[1])) * ticks_to_millimeter #transforming the variable into millimeters for control input 

        covariance = get_covariance(covariance,pose,control,control_motion_factor,control_turn_factor,width_robo) #getting the new covariance  asssicated with the state using curretn pose , control covariance etc 

        pose = pose_update(tick_difference,ticks_to_millimeter,width_robo,scanner_displacement)#updating pase uisng current state 

        #printing the ouput for debuging
        #print("Pose")
        #print("   ")
        #print(pose)
        #print("Covariance")
        #print("   ")
        #print(covariance)

        #assiging each pose vaiable into separate X Y and theta
        X=pose[0]
        Y=pose[1]
        theta=pose[2]

        #assiging each pozyx vaiable into separate X Y and theta
        X_posyx=Posyx.x
        Y_posyx=Posyx.y
        

        #getting the quaternion for tf tranform concerning odometry and pozyx
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        pozyx_quat= tf.transformations.quaternion_from_euler(0, 0, Posyx.theta)
        
        #sending each transform for lidar odometry and pozyx
        odom_broadcaster.sendTransform((X, Y, 0),odom_quat,current_time,"/ekf_base_link","/odom")  
        #lidar_broadcaster.sendTransform((X, Y, 0),odom_quat,current_time,"/laser","/odom") 
        #posyx_transform.sendTransform((X_posyx,Y_posyx,0), pozyx_quat,current_time,"/pozyx","/odom")

        #creating a instance of the custom odometry message 
        odom = PoseWithCovarianceStamped()
        odom.PoseWithCovarianceStamped.pose.position.x = X
        odom.PoseWithCovarianceStamped.pose.position.y = Y
        odom.PoseWithCovarianceStamped.pose.position.z = 0
        odom.PoseWithCovarianceStamped.pose.orientation.x = odom_quat[0]
        odom.PoseWithCovarianceStamped.pose.orientation.y = odom_quat[1]
        odom.PoseWithCovarianceStamped.pose.orientation.z = odom_quat[2]
        odom.PoseWithCovarianceStamped.pose.orientation.w = odom_quat[3]
        
        # publish the message
        odom_pub.publish(odom) #publishing the odometry message 
        rate.sleep() #stopiing the loop to maintian rate 
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: #exception for ROSInteruppt
        pass
