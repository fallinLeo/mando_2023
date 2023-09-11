#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String message;

void messageCallback(const std_msgs::String& msg) {
  // 받은 메시지를 시리얼 모니터로 출력
  Serial.println(msg.data);
}
ros::Subscriber<std_msgs::String> sub("chatter", messageCallback); // 수정된 부분

void setup() {
  nh.initNode();

  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
