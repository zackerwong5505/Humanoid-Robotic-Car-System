#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray,Bool

from geometry_msgs.msg import Twist
import roslaunch 
from enum import Enum




class CoreNodeController():
    def __init__(self):
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subObject=rospy.Subscriber("/objects", Float32MultiArray, self.callback)
        self.pubSlow = rospy.Publisher('/isslowdown', Bool, queue_size=10)
        self.launchfl=False
        self.launchStop=False
        self.sleepawhile=False
        self.l=False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.is_triggered=False

        self.STOP=12
		self.PARKING=10
		self.CROSS=11
        self.CONS=14
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.current_mode = "fl idle"
        
        rospy.on_shutdown(self.nodeShutdown)

    def launch(self,launch_num,is_start):
        if launch_num==1:
        
            if is_start==True:
                if self.launchfl==False:
                    self.launchfl=True
                    roslaunch.configure_logging(self.uuid)
                    self.launchFollowLine = roslaunch.scriptapi.ROSLaunch()
                    self.launchFollowLine.parent = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/vm/detect_ws/src/line_follower_turtlebot/launch/line.launch"])
                    self.launchFollowLine.start()
                else:
                    pass
            else:
                # self.nodeShutdown()
                if self.launchfl==True:
                    self.launchfl=False
                    self.launchFollowLine.stop()   
                    rospy.loginfo("Shutting down")
                    
                else:
                    pass

    def callback(self,data):    
       
        

        if (len(data.data)>0):

            id=data.data[0]
           
            if id==self.STOP:
                self.is_triggered=True
                self.current_mode="stop"
                
                
                

            elif id==self.CROSS:
                self.is_triggered=True
		self.current_mode="slowdown"
              
		
                
                

               
	    elif id==self.PARKING:
                self.is_triggered=True
                self.current_mode="parking"
                
		

	
        else:
            self.current_mode="fl"
            self.pubSlow.publish(False)
            self.is_triggered=True
           
 

    def stop(self,duration):
       
        vel_msg=Twist()

        t0 = rospy.Time.now().to_sec()
        d=0
        while(d<duration):
            t1 = rospy.Time.now().to_sec()
            vel_msg.linear.x=0
            d=t1-t0
            self.velocity_publisher.publish(vel_msg)
        
        self.velocity_publisher.publish(vel_msg)
       


    def slowdown(self):
        t0 = float(rospy.Time.now().to_sec())
        duration=0
        while(duration<1.0):
            t1 = float(rospy.Time.now().to_sec())
            duration=t1-t0
            self.pubSlow.publish(True)
	


    def parking(self):
	self.stop(1)
	self.rotate(10,90)
	self.stop(0.5)
	self.move(0.05,0.5)
   
    def move(self, speed, distance):
        vel_msg=Twist()
        t0 = float(rospy.Time.now().to_sec())
        current_distance = 0
        vel_msg.linear.x = -speed
        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=float(rospy.Time.now().to_sec())
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)

    def rotate(self,speed, angle):
        vel_msg=Twist()
        PI = 3.1415926535897
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)


        #Forcing our robot to stop
        vel_msg.angular.z = 0

        self.velocity_publisher.publish(vel_msg)
     
    def nodeControl(self):
        if self.current_mode=="fl":            
            self.launch(1,True)
            rospy.loginfo("launching follow line")
            

        elif self.current_mode=="stop":
            rospy.loginfo("launching Stop")
          
            self.stop(1)
            self.current_mode="fl"
           
            self.sleepAwhile()
         


	elif self.current_mode=="slowdown":
            rospy.loginfo("slowdown")
            self.current_mode="fl"
            self.slowdown()
            self.launch(1,True)
            self.sleepAwhile()

	elif self.current_mode=="parking":
            rospy.loginfo("parking")
            self.rotate(10,15)
            self.move(0.03,0.06)
            self.stop(5)

	self.is_triggered=False
            
            
    def sleepAwhile(self):
        
        t0 = rospy.Time.now().to_sec()
        duration=0
        while(duration<2.0):
            t1 = rospy.Time.now().to_sec()
            self.subObject.unregister()
            duration=t1-t0

        self.subObject=rospy.Subscriber("/objects", Float32MultiArray, self.callback)
        self.current_mode="fl"
     

    def listener(self):       
        
        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            
            print("launchfl= "+str(self.launchfl))
            print("istriggered="+str(self.is_triggered))
            print(self.current_mode)
            if self.is_triggered==True:
                self.nodeControl()

            loop_rate.sleep()
        # spin() simply keeps python from exiting until this node is stopped

    def nodeShutdown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    
    
   
    try:
        rospy.init_node('action_controller', anonymous=True)

        
        node=CoreNodeController()
        node.listener()
        # rospy.spin()


         
    except rospy.ROSInterruptException:
        pass

    
