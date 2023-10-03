#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/UInt16.h>
ros::NodeHandle nh;

int btn = 7;


std_msgs::UInt16 isdock;
ros::Publisher message("switch_onoff", &isdock);

void setup()
{
  // pinMode(13, OUTPUT);
  pinMode(btn, INPUT);
  digitalWrite(btn, HIGH);
  nh.initNode();
  nh.advertise(message);
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
