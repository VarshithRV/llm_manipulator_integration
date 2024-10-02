/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

// setpoint contains incoming message
int setpoint = 0;

void messageCb( const std_msgs::Bool& msg){
  setpoint = (int)msg.data;
}

ros::Subscriber<std_msgs::Bool> sub("gripper", messageCb );



std_msgs::Bool received_msg;
ros::Publisher chatter("gripper_state", &received_msg);

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
}

void loop()
{
  received_msg.data = setpoint;

  // change pin outputs using setpoint here.
  
  chatter.publish( &received_msg );
  if(setpoint)
  {
    digitalWrite(8,HIGH);
//    digitalWrite(9,HIGH);
//    digitalWrite(10,HIGH); 
  }
  else
  {
    digitalWrite(8,LOW);
//    digitalWrite(9,LOW);
//    digitalWrite(10,LOW);
  }
  nh.spinOnce();
  delay(50);
}
