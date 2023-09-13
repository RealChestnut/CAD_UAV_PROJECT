#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
ros::NodeHandle nh;
Servo servo1;
Servo servo2;

int btn = 7;

void servo_cb(const std_msgs::UInt16 &cmd_msg)
{
  servo1.write(cmd_msg.data); // set servo angle, should be from 0-180
  servo2.write(cmd_msg.data);
  // digitalWrite(13, HIGH-digitalRead(13)); //toggle led
}

 
ros::Subscriber<std_msgs::UInt16> sub("cmd_msg", servo_cb);
std_msgs::UInt16 isdock;
ros::Publisher message("message", &isdock);

void setup()
{
  // pinMode(13, OUTPUT);
  pinMode(btn, INPUT);
  digitalWrite(btn, HIGH);    //추가한 코드
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(message);
  servo1.attach(10); // attach it to pin 9
  servo2.attach(6);
}

void loop()
{
  if (digitalRead(btn) == LOW)
  {
  isdock.data = 0;
  message.publish(&isdock);
  }
  else
  {
  isdock.data = 1;
  message.publish(&isdock);
  }
nh.spinOnce();
delay(1);
//Serial.println();
}
