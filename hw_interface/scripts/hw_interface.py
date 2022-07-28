import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
import serial
import time

#Open the serial connection to the Arduino - which causes the Arduino to reset
ser = serial.Serial("/dev/ttyUSB0", 9600)

#For communication the message to be sent to the Arduino starts with startmarker '<' and ends with endmarker '>'
#the message content comprises desired left motor speed as integer and desired right motor speed as integer
startMarker = 60 #Unicode code for <
endMarker = 62 #Unicode code for >

#To store information from ROS callback
left_speed = 0
right_speed = 0

#Robot physical parms
wheel_radius = 37.0 #mm
robot_width = 475.0 #mm

#============================
def recvFromArduino():
# Receiving a message from the Arduino involves
# waiting until the startMarker is detected
# saving all subsequent bytes until the end marker is detected
  global startMarker, endMarker, ser

  recieved_data = ""
  recieved_char = "!" # any value that is not an endmarker (>) or startMarker (<)
  
  # wait for the start character
  while  ord(recieved_char) != startMarker: 
    recieved_char = ser.read()
  
  # save data until the end marker is found
  while ord(recieved_char) != endMarker:
    if ord(recieved_char) != startMarker:
      recieved_data = recieved_data + recieved_char.decode('utf-8') 
    recieved_char = ser.read()
  
  return(recieved_data)

#============================
# Wait for a message from the Arduino to give it time to reset
def waitForArduino():

  # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
  # it also ensures that any bytes left over from a previous message are discarded
  
  global startMarker, endMarker, ser
  print("Waiting for Arduino to be ready")
  
  msg = ""
  while msg.find("Arduino is ready") == -1:
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0): 
      pass
    
    msg = recvFromArduino()
 
  print(msg)
  return

#===========================
# Function for gracefull shutdown
def turn_off():
  #Function for safe shutdown 
  
  print('hw_interface node turning off')
  ser.write("<0,0>".encode('utf-8'))
  time.sleep(3)
  ser.close
  return

#===========================
# ROS callback
def convert_vel_cmd(msg):
  #Function to convert linear and angular velocity request to 
  #left and right wheel velocities

  global wheel_radius, robot_width, left_speed, right_speed

  cmd_linear_vel = msg.linear.x*1000.0
  cmd_angular_vel = msg.angular.z

  right_speed = ((2.0*cmd_linear_vel) + (cmd_angular_vel*robot_width))/(2*wheel_radius)
  left_speed = ((2.0*cmd_linear_vel) - (cmd_angular_vel*robot_width))/(2*wheel_radius)
  
  right_speed = right_speed*3
  left_speed = left_speed*3

  return

#============================
def main():
  global wheel_radius, robot_width, ser, left_speed, right_speed

  #Initialising             
  rospy.init_node('hw_interface')
  rospy.on_shutdown(turn_off)
  waitForArduino()
  
  print('hw_interface node running')
  rospy.Subscriber("/cmd_vel", Twist, convert_vel_cmd)
  pub_left_tick = rospy.Publisher("/left_ticks", Int64, queue_size = 1)
  pub_right_tick = rospy.Publisher("/right_ticks", Int64, queue_size = 1)
  
  wheel_radius = rospy.get_param('wheel_radius', 37)
  robot_width = rospy.get_param('robot_width', 475)

  rate = rospy.Rate(2) #2Hz
  while not rospy.is_shutdown():
    
    #Sent commanded velocity to Arduino
    output_string = "<" + str(int(left_speed)) + "," + str(int(right_speed)) + ">"
    print(output_string)
    ser.reset_output_buffer()
    ser.write(output_string.encode('utf-8'))
   
    #Update ticks info from arduino
    ser.reset_input_buffer()
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0): 
      pass

    recieved_data = recvFromArduino()
    print("a::",recieved_data)
    ticks = [int(x) for x in recieved_data.split(',')]
    
    print("ticks",ticks)
    #Publish ticks
    pub_left_tick.publish(ticks[0])
    pub_right_tick.publish(ticks[1])

    rate.sleep()

  return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass