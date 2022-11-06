#!/usr/bin/env python

#importing the nescessary libraries 
import rospy #python library for ROS 
import tf #importing tf transforms for doing various transformations
from geometry_msgs.msg import PoseWithCovarianceStamped #importing message for publish
from geometry_msgs.msg import  Twist #importing message for publish
import numpy as np #importing thr numericla python library


#class to keep track of the states and covariances
class ExtendedKalmanFilter():

    #the necessary state variables for ekf
    state=[]
    covariance=[]

    #variables associated withe the standard deviatiion associated with lidar distance and angle
    measurement_distance_stddev = 0.1
    measurement_angle_stddev = 0.1 
    threshold=0 # threshold for identifying the best wall matches 
    scanner_displacement=0 
    result=[]

    #dictionary for storing the map
    map=dict()
    

    @staticmethod
    #fucntion which relates between the state and the reference walls 
    def lidar_measurement_model(state, landmark, scanner_displacement):
        #finding the r, theta of the wall with respect to the lidar
        dx = landmark[0] - (state[0] + scanner_displacement * np.cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * np.sin(state[2]))
        r = np.sqrt(dx * dx + dy * dy)
        alpha = (np.atan2(dy, dx) - state[2] + np.pi) % (2*np.pi) - np.pi

        return np.array([r, alpha])
    
    @staticmethod
    #function to return the covariance detected parameters of the wall 
    def get_covariance_lidar(p1,p2,m_radius,m_angle):

        #here we are doing an end point fit so we calcualte covariance by assuming some fixed variance in those points 

        #first point 
        pho_1 = p1[0]**2+p1[1]**2
        theta_1= np.atan2(p1[1],p1[0])

        #second point 
        pho_2 = p2[0]**2+p2[1]**2
        theta_2= np.atan2(p2[0],p2[1])

        #covariance matrix for each of the two points used for estimaing walls
        covariance_x =np.diag[m_radius,m_radius,m_angle,m_angle]

        #a term in the jacobian matrix 
        c=-((pho_1-pho_2)/2)*np.sin((theta_1-theta_2)/2)

        #jacobian for the process used for estimating the walls from lidar data 
        Fpq = np.array[[0,0,1/2,1/2],[np.cos((theta_1-theta_2)/2),-np.cos((theta_1-theta_2)/2),c,-c]]

        #converting the covarince to the final covariance estimate
        covariance_ar=np.dot(Fpq, np.dot(covariance_x, Fpq.T))

        return covariance_ar

    @staticmethod
    #fucntion to calculate the jacobian of the measurement with respect to the state 
    def dmeasurement_model_dstate(state, landmark, scanner_displacement):

        #the variables involved in cacualting the x,y and theta
        x, y, theta = state
        x_m, y_m = landmark
        d = scanner_displacement

        #updating the jacobian matrix using the standard procedure 
        x_l = x + d * np.cos(theta)
        y_l = y + d * np.sin(theta)

        delta_x = x_m - x_l
        delta_y = y_m - y_l

        q = (delta_x) ** 2 + (delta_y) ** 2

        dr_dx = -delta_x / np.sqrt(q)
        dr_dy = -delta_y / np.sqrt(q)
        dr_dtheta = (d / np.sqrt(q)) * (delta_x * np.sin(theta) - delta_y * np.cos(theta))

        dalpha_dx = delta_y / q
        dalpha_dy = -delta_x / q
        dalpha_dtheta = - (d / q) * (delta_x * np.cos(theta) + delta_y * np.sin(theta)) - 1

        return np.array([[dr_dx, dr_dy, dr_dtheta], [dalpha_dx, dalpha_dy, dalpha_dtheta]])

    def lidar_correction(self,r_values,alpha_values,endpoints_x,endpoints_y):

        #loading the map
        map=self.map

        #converting the map to polar values for ease of comparison
        r_ref_values,alpha_ref_values = map_convert_to_polar(map)

        #extracting the beginning points and end points of the walls
        wall_begin_points=map["wall_begin_points"]
        wall_end_points=map["wall_end_points"]

        #converting the detected wall into to the world coordinate frame
        wall_detected_r_values,wall_detected_alpha_values,endpoints_x,endpoints_y= robot_to_world(r_values,alpha_values,endpoints_x,endpoints_y)

        #defining certain thresholds to be used for finding the best wall matches 
        best_dist = self.threshold
        best_r = None

        #looping acroos the detected landmarks and refernce land marks to detect the best corellations 
        for i in range(len(wall_detected_r_values)):

            #extracting the begining enpoint and ending endpoint
            endpoint_x_begin = endpoints_x[2*i][0]
            endpoint_y_begin = wall_begin_points[2*i][1]

            endpoint_x_end = wall_end_points[2*i+1][0]
            endpoint_y_end = wall_end_points[2*i+1][1]

            #loop to correlate with the reference values
            for j in range(len(r_ref_values)):

                #calcualting the absolute squared norm difference between the refrence and detected walls
                dx, dy = r_ref_values[j] - wall_detected_r_values[i], alpha_ref_values[j] - wall_detected_alpha_values[i]

                dist_2 = dx * dx + dy * dy

                #extracting the begining endpoint and ending endpoint of the detected walls to check if they follow the conditions 
                endpoint_x_begin_wall = wall_begin_points[j][0]
                endpoint_y_begin_wall = wall_begin_points[j][1]

                endpoint_x_end_wall = wall_end_points[j][0]
                endpoint_y_end_wall = wall_end_points[j][1]

                #set of conditions to ensure that the detected walls lie within the refernce walls 
                condition1 = endpoint_x_begin_wall<=endpoint_x_begin and endpoint_x_begin_wall<endpoint_x_end 
                condition2 = endpoint_x_end_wall>endpoint_x_begin and endpoint_x_end_wall>=endpoint_x_end 
    
                condition3 = endpoint_y_begin_wall<=endpoint_y_begin and endpoint_y_begin_wall<endpoint_y_end 
                condition4 = endpoint_y_end_wall>endpoint_y_begin and endpoint_y_end_wall>=endpoint_y_end 

                #detecting the best landmarks and adding them to a list 
                if condition1 and condition2 and condition3 and condition4:
                    if dist_2 < best_dist:
                        best_dist = dist_2
                        best_r,best_alpha = r_ref_values[j],alpha_ref_values[j]
                        endpoint_b = [endpoint_x_begin,endpoint_y_begin]
                        endpoint_e = [endpoint_x_end,endpoint_y_end]
                    
            #appending the best detected wall result after a pass through the reference walls if its present 
            if best_r:
                self.result.append(wall_detected_r_values[i],wall_detected_alpha_values[j],endpoint_b,endpoint_e,best_r,best_alpha)
        
        #using the list created to perform correction on the predicted estimate of the robots pose for each correlation found 
        for ele in self.result:

            landmark=(ele[4]*np.cos(ele[5]),ele[4]*np.sin(ele[5]))

            #covarinace update and kalman gain calculation
            H_t = self.dmeasurement_model_dstate(self.state, landmark,self.scanner_displacement) #jacobian asociated with the measurement model
            Q = self.get_covariance_lidar(ele[2],ele[3],self.measurement_distance_stddev,self.measurement_angle_stddev) #getting the covariance due to the procedure followed for line fitting 
            K_t = np.dot(np.dot(self.covariance, H_t.T), np.linalg.inv(np.dot(H_t, np.dot(self.covariance, H_t.T)) + Q))#extracting the kalman filter

            #correction step and updation of the state in the ekf class
            innovation = np.array(ele[0],ele[1]) - self.lidar_measurement_model(self.state, landmark, self.scanner_displacement)
            innovation[1] = (innovation[1] + np.pi) % (2*np.pi) - np.pi

            #updating the state and covariance
            self.state += np.dot(K_t, innovation)  
            self.covariance = np.dot(np.eye(3) - np.dot(K_t, H_t), self.covariance)

    #function to correct the yaw angle 
    def imu_correction(self,yaw_imu,variance_imu):

        #converting yaw angle to radians
        yaw_imu = yaw_imu*np.pi/180

        #stroing the current yaw and varaince 
        current_yaw = self.state[2]
        current_variance = self.covariance[2][2]

        #appyling kalman filter based correction on the yaw angle mean and variance
        corrected_yaw = current_yaw + (current_variance/(current_variance + variance_imu))*(yaw_imu-current_yaw)
        corrected_variance = current_variance - (current_variance/(current_variance + variance_imu))*(current_variance)

        #updating the state and variance in the ekf class 
        self.state[2]=corrected_yaw
        self.covariance[2][2]= corrected_variance

    #function to apply the kalman filter correction using the values measure from april_tag
    def april_tag_correction(self,measurement_pose,measurement_covariance):

        #cacualting the innovation to update the innovationand innovation covariance here jacobain is identity so the equations simplify a lot 
        innovation = measurement_pose - self.state
        innovation_covariance = self.covariance+ measurement_covariance
            
        #calcualtin gthe Kalman Gain
        Kalman_gain = np.dot(self.covariance, np.linalg.inv( innovation_covariance))

        #updating the state and covariance
        self.state += np.dot(Kalman_gain, innovation)  
        self.covariance = np.dot((np.eye(3) - Kalman_gain),  self.covariance)
        

#function for proper conversion from catesian to polar if the radius is negative
def radius_negative_conversion(r,alpha):

    if r < 0:
        #shifing the alpha by pi
        alpha = alpha + np.pi

        #restricting the alpha tobe between -pi and pi
        if alpha > np.pi:
            alpha = alpha-2*np.pi
        
        #negating r
        r = -r
    return r,alpha

#function to convert the detected landmark positions to the world coordinate frame for comparison
def robot_to_world(r_values,alpha_values,endpoints_x,endpoints_y):
    #acessing the global ekf object 
    global ekf

    #lists to strore rnage , alpha, x and y endpoints
    r=[]
    alpha=[]
    endpoints_x_world=[]
    enpoints_y_world=[]

    #direction 
    dx = np.cos(ekf.state[2])
    dy = np.sin(ekf.state[2])

    #converting the radius and alpha values to world coordinate frame
    for index in len(r_values):

        #getting the x ,y values and converting then into the world frame
        x,y = r_values[index]*np.cos(alpha_values[index]),r_values[index]*np.sin(alpha_values[index])
        x_world,y_world = (x * dx - y * dy + ekf.state[0], x * dy + y * dx + ekf.state[1]) 

        #extracting the rnage and alpha values from the world frame 
        r_world=np.sqrt(x_world**2+y_world**2)
        alpha_world=np.atan2(y_world,x_world)

        #appending to the respective list 
        r.append(r_world)
        alpha.append(alpha_world)
    
    #converting the endpoints to the world coordinate frame
    for index in len(endpoints_x):

        #extracting the x nad y end_points for the walls with respect ot the world coordiante frame 
        x,y = endpoints_x[index],endpoints_y[index]
        x_world,y_world = (x * dx - y * dy + ekf.state[0], x * dy + y * dx + ekf.state[1]) 
        endpoints_x_world.append(x_world)
    

#cartesian to polar conversion given a set of coordiantes 
def cartesian_to_polar(X,Y):

    #fitting a pline over the points and extracting slope and intercept 
    k,n = np.polyfit(X,Y,1)
    alpha = np.arctan(-1/k) 

    #converting the slope and intercept to r and alpha form
    r = n/(np.sin(alpha)-k*np.cos(alpha))

    #checking for negative r cases 
    r,alpha=radius_negative_conversion(r,alpha)

    #returning the necessary values
    return r,alpha

#function to convert the map values from cartesian to polar coordiantes 
def map_convert_to_polar(map):
    wall_begin_points=map["wall_begin_points"]
    wall_end_points=map["wall_end_points"]
    r_values=[]
    alpha_values=[]

    for p1,p2 in wall_begin_points,wall_end_points:

        x=np.array(p1[0],p2[0])
        y=np.array(p1[1],p2[1])

        r,alpha=cartesian_to_polar(x,y)
        r_values.append(r)
        alpha_values.append(alpha)

    return r_values,alpha_values

def pred_update_callback(data):
    #function to perform the prediction update step by using the messge from odom node
    rospy.loginfo('pred_update_callback is called')
    #acessing the global ekf class object 
    global ekf

    #getting updating ekf state as the output predicted by odometry
    ekf.state = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])  

    #defining the new 3x3 zeros matrix and updating it wiht relevant values from the 1x36 covariance array from odometry_node
    pred_covariance = np.zeros((3,3))

    pred_covariance[0,0] = data.pose.covariance[0]
    pred_covariance[1,1] = data.pose.covariance[7]
    pred_covariance[2,2] = data.pose.covariance[35]
    pred_covariance[1,2] = data.pose.covariance[10]
    pred_covariance[2,1] = data.pose.covariance[10]
    pred_covariance[0,1] = data.pose.covariance[2]
    pred_covariance[1,0] = data.pose.covariance[2]
    pred_covariance[2,0] = data.pose.covariance[5]
    pred_covariance[0,2] = data.pose.covariance[5]

    #updating the covarince assicated witht the ekf_object
    ekf.covariance = np.copy(pred_covariance)

def correction_from_lidar_callback(msg):
    #acessin gthe global ekf 
    global ekf

    #applying the ekf correction using lidar
    ekf.lidar_correction(map,msg.radius_values,msg.alpha_values,msg.endpoints_x,msg.endpoints_y)

def correction_from_imu_callback(msg):
    #acessing the global ekf class object 
    global ekf

    #fixing a constant variance for IMU_orientation
    variance_imu = 5*np.pi/180

    #calling the correction function to correct the yaw angle of the robot
    ekf.imu_correction(msg.angular.z,variance_imu)

def correction_from_april_tag_callback(data):

    #acessing the global ekf class object 
    global ekf

    #stroing the measured pose and relevan covariance values from April_tag node 
    measurement_pose = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

    measurement_covariance = np.zeros((3,3))

    measurement_covariance[0,0] = data.pose.covariance[0]
    measurement_covariance[1,1] = data.pose.covariance[7]
    measurement_covariance[2,2] = data.pose.covariance[35]
    measurement_covariance[1,2] = data.pose.covariance[10]
    measurement_covariance[2,1] = data.pose.covariance[10]
    # measurement_covariance[0,1] = data.pose.covariance[0]
    # measurement_covariance[1,0] = data.pose.covariance[0]
    measurement_covariance[2,0] = data.pose.covariance[5]
    measurement_covariance[0,2] = data.pose.covariance[5]

    #applying the correction for state using measured values from april_tag
    ekf.april_tag_correction(measurement_pose,measurement_covariance)

    
#Global object of the ekf class
ekf=ExtendedKalmanFilter()

#updating the map associated with ekf
ekf.map["wall_begin_points"] =rospy.get_param("wall_begin_points") #updating the beginning points of the map
ekf.map["wall_end_points"] = rospy.get_param("wall_end_points")  #updating the ending points of the map

def main():

    #acessing the global ekf 
    global ekf

    #creating a publisher topic named pose_ekf and a node named ekf_node
    ekf_pub = rospy.Publisher("pose_ekf", PoseWithCovarianceStamped, queue_size=1)
    ekf_broadcaster = tf.TransformBroadcaster() #tf broadcasters for ekf
    rospy.init_node('ekf_node')

    #initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) 
  
    #intialising some parameters realted to the ekf 
    ekf.measurement_distance_stddev = rospy.get_param("measurement_distance_stddev")
    ekf.measurement_angle_stddev = rospy.get_param("measurement_angle_stddev")
    ekf.threshold=1.1 #threshold for calcualting the association with found out walls and reference walls


    #deffining array to keep track of sates and covariances 
    states=[]
    covariances=[]

    #getting time for using it in tf transforms
    current_time = rospy.Time.now()
    # last_time = rospy.Time.now()

    #subscribing the required topics
    rospy.Subscriber('/odom', PoseWithCovarianceStamped, pred_update_callback) # getting the odometry prediction from odom_node 
    #commented out as it is not being tested now 
    #rospy.Subscriber('feature_from_lidar', features, correction_from_lidar_callback)
    rospy.Subscriber('/pozyx_position',Twist,correction_from_imu_callback) # getting the orientation asscoiated with the imu of pozyx 
    rospy.Subscriber('/aprtag',PoseWithCovarianceStamped,correction_from_april_tag_callback) # getting the predicted position and covariance from apriltag based detection module

    while not rospy.is_shutdown():

        #extracting the curretn time to give it to the tf transformation 
        current_time = rospy.Time.now()

        #appending the states and covariances to a list 
        states.append(ekf.state)
        covariances.append(ekf.covariance)

        #extracting pose and covariances to separate variable for easenes of sending
        if len(ekf.state) == 0:
            continue
        X=ekf.state[0]
        Y=ekf.state[1]
        theta=ekf.state[2]
        covariance = ekf.covariance


        #getting the quaternion for tf tranform concerning ekf
        ekf_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        #preparing the message for publishing and publishing the message updating the nescessary fields
        ekf_message = PoseWithCovarianceStamped()
        ekf_message.pose.pose.position.x = X
        ekf_message.pose.pose.position.y = Y
        ekf_message.pose.pose.position.z = 0
        ekf_message.pose.pose.orientation.x = ekf_quat[0]
        ekf_message.pose.pose.orientation.y = ekf_quat[1]
        ekf_message.pose.pose.orientation.z = ekf_quat[2]
        ekf_message.pose.pose.orientation.w = ekf_quat[3]

        ekf_message.pose.covariance = [covariance[0,0],covariance[0,1],0,0,0,covariance[0,2],\
                                                            covariance[1,0],covariance[1,1],0,0,0,covariance[1,2],\
                                                            0,0,0,0,0,0,\
                                                            0,0,0,0,0,0,\
                                                            0,0,0,0,0,0,\
                                                            covariance[2,0],covariance[2,1],0,0,0,covariance[2,2]]

        #broadcasting the transform created with odom as the base frame
        ekf_broadcaster.sendTransform((X, Y, 0),ekf_quat,current_time,"/ekf_base_link","/odom") 
        
        # publish the message
        ekf_pub.publish(ekf_message)

        last_time = current_time

        ##stopiing the loop to maintian rate 
        rate.sleep()
    rospy.spin()
        
if __name__ == '__main__':
   
    try:
        main()
    except rospy.ROSInterruptException: #exception for ROSInteruppt
        pass
