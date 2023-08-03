#include <MsTimer2.h>

//========================= ROS 관련 ==============================================
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4
#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7

float linear_x;
float linear_y;
float linear_z;
float angular_x;
float angular_y;
float angular_z;

ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel;
//std_msgs::Float32 data_msg;

int velocity = -999;
int steer_angle = 0;
void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  velocity = (int)msg.linear.x;
  steer_angle = (int)msg.angular.z;

  if(velocity>=255) velocity = 255;
  if(velocity<=-255) velocity = -255;
}


ros::Subscriber<geometry_msgs::Twist> cmd_sub("teleop_cmd_vel", cmd_vel_callback);
ros::Publisher cmd_pub("cmd_vel2", &cmd_vel);
//ros::Publisher encoder_pub1("encoder1", &encoder_data1);
//ros::Publisher encoder_pub2("encoder1", &encoder_data2);


//motor val
// Front Motor Drive Pin 설정



//

void setup() {
  // 타이머 함수
  MsTimer2::set(10, control_callback);
  MsTimer2::start();
  Serial.begin(57600);
  pinMode(MOTOR1_PWM,OUTPUT);
  pinMode(MOTOR1_ENA,OUTPUT);
  pinMode(MOTOR1_ENB,OUTPUT);
  
  pinMode(MOTOR1_PWM,OUTPUT);
  pinMode(MOTOR1_ENA,OUTPUT);
  pinMode(MOTOR1_ENB,OUTPUT);
  // ROS
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(cmd_pub);
}

void control_callback(void)
{
  //data_exam = 5.0;
  //data_msg.data = data_exam;
  // ROS
  //data_pub.publish(&data_msg);
  nh.spinOnce();
}


void loop()
{
  int f_speed = 0, r_speed = 0;
  f_speed = r_speed = velocity;
  front_motor_control(f_speed);
  rear_motor_control(r_speed);
  cmd_vel.linear.x = velocity;
  cmd_pub.publish(&cmd_vel);
  nh.spinOnce();
  delay(10);
}




/*void steer_motor_control(float motor_pwm)
{
  if( (steer_y_m >= -255  ) || (steer_y_m <= 255  ) ) // 최대 최소값이상 움직이지 않도록 보호 알고리즘
  {
     digitalWrite(MOTOR3_ENA, LOW);
     digitalWrite(MOTOR3_ENB, LOW);
     analogWrite(MOTOR3_PWM, 0);    // 정지
     return;    
  }

  if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}*/
void front_motor_control(float motor1_pwm)
{
  if(motor1_pwm>0)//forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);
   }
}
void rear_motor_control(float motor2_pwm)
{

  if (motor2_pwm > 0) // forward
  {
    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, motor2_pwm);
   }

  else if (motor2_pwm < 0) // backward
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, -motor2_pwm);
  }
  else
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }
}
