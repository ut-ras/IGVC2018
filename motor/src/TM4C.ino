/**
This node runs on the MCU. To edit it you need to follow the tutorials at
http://wiki.ros.org/rosserial_tivac/Tutorials/Energia%20Setup.
*/

#include <ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

Servo left_motor;
Servo right_motor;

void color_cb(const std_msgs::ColorRGBA& msg){
    analogWrite(RED_LED, msg.r * msg.a);
    analogWrite(GREEN_LED, msg.g * msg.a);
    analogWrite(BLUE_LED, msg.b * msg.a);
}

//TODO: add watchdog
void left_cb(const std_msgs::Float32& msg){
    left_motor.write(90 + msg.data * 90);
}

void right_cb(const std_msgs::Float32& msg){
    right_motor.write(90 + msg.data * 90);
}

ros::NodeHandle nh;
ros::Subscriber < std_msgs::ColorRGBA > color_sub("led", &color_cb);
ros::Subscriber < std_msgs::Float32 > left_sub("left", &left_cb);
ros::Subscriber < std_msgs::Float32 > right_sub("right", &right_cb);

void setup(){
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    analogWrite(RED_LED, 255);
    analogWrite(GREEN_LED, 255);
    analogWrite(BLUE_LED, 255);

    left_motor.attach(PD_2);
    right_motor.attach(PD_3);

    nh.initNode();
    nh.subscribe(color_sub);
    nh.subscribe(left_sub);
    nh.subscribe(right_sub);
}

void loop(){
    nh.spinOnce();
    delay(10);
}
