#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

#define SMILE 4
#define STOP 13
#define ARROW_UP 5
#define ARROW_DOWN 6

int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{	
   if (object->data.size() > 0)
   {
      id = object->data[0];

      switch (id)
      {
      case STOP:
         
	 
         set_vel.linear.x = 0;
	 for(int n=300; n>0; n--) {
		set_vel.linear.x = 0;
		set_vel.angular.z=3.142;
      		action_pub.publish(set_vel);
      		
	}
	set_vel.linear.x = 0;
	ros::Duration(1).sleep();
	 
         break;
      case ARROW_UP:
         set_vel.linear.x = 1;
         set_vel.angular.z = 0;
         break;
      case ARROW_DOWN:
         set_vel.linear.x = -1;
         set_vel.angular.z = 0;
         break;
      default: // other object
         set_vel.linear.x = 0;
         set_vel.angular.z = 0;
      }
      action_pub.publish(set_vel);
   }
   else
   {
      // No object detected
      set_vel.linear.x = 0.1;
      set_vel.angular.z = 0.0;
      action_pub.publish(set_vel);
   }
}

int main(int argc, char **argv)
{
   
   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(50);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

   
   action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   set_vel.linear.x = 0;
   set_vel.linear.y = 0;
   set_vel.linear.z = 0;
   set_vel.angular.x = 0;
   set_vel.angular.y = 0;
   set_vel.angular.z = 0;
   
   while (ros::ok())

   {  action_pub.publish(set_vel);
      
      ros::spinOnce();
      loop_rate.sleep();
   }
}
