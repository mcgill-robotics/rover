#include <ros.h>
#include <std_msgs/Float32.h>

#define trigPin 10
#define echoPin 9

ros::NodeHandle nh;

std_msgs::Float32 distance_msg;
ros::Publisher distRead("distReader", &distance_msg);

int duration;
float distance;

void setup() {
  nh.initNode();
  nh.advertise(distRead);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(57600);
}

void loop() {
  digitalWrite(trigPin, LOW); //reset trigpin signal
  delay(500);
  digitalWrite(trigPin, HIGH);
  delay(1000);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); // duration is the pulse to echoPin
  distance = (duration/2)/29.1; // converts duration to distance in cm
  Serial.println(distance);

  distance_msg.data = distance;
  distRead.publish( &distance_msg );
  nh.spinOnce();
  delay(10);

}
