#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "FrontMotorPIDController.h"
#include "MotorPIDController.h"
#include "Arduino.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <MsTimer2.h>


ros::NodeHandle nh;


int last_time_action_done_millis = 0;
//==================================================================================

// 1m 당 pulse 수
#define m_2_pulse    349      
#define pulse_2_m  1./349.
#define vel_2_pulse   m_2_pulse/50. // 20Hz 제어 주기에서 속도와 Δpulse 변환 값

// Front motor pin number
#define MOTOR1_PWM 8  //6
#define MOTOR1_ENA 9  //7

// Rear motor pin number
#define MOTOR2_PWM 6  //8
#define MOTOR2_ENA 7  //9

// Encoder pin
#define encoderPinA 2
#define encoderPinB 3

// Steering motor pin
#define MOTOR3_PWM 4  //4
#define MOTOR3_ENA 5  //5

#define STEERPOT A2

int encoderPos = 0;

//==================================================================================


static float Kp_front = 1.5;
static float Kd_front = 0;
static float Ki_front = 0.1;

FrontMotorPIDController front_PID_controller(MOTOR1_PWM, MOTOR1_ENA,
                                          MOTOR2_PWM, MOTOR2_ENA);

//==================================================================================

//float steer_r = map(steer_r_deg,0,980,-20,20) << steer_r_deg를 subscribe해서 변환해서 사용함 추후에 추가해야함

// float steer_r = 600.0; // target pot_value 
float neural_angle = 0.0; // degree
float min_angle = -20.0; // degree
float max_angle = 20.0; // degree
float ad_min = 90.0+100.0; // analog data
float ad_max = 890.0-100.0; // analog data

//PID 상수 설정
static float Kp = 0.4;
static float Ki = 0.21;
static float Kd = 1.0;

static float alpha = 0.3;


MotorPIDController steer_PID_controller(MOTOR3_PWM, MOTOR3_ENA);

//========================= ROS 관련 ==============================================
geometry_msgs::Twist cmd_vel;
ros::Publisher cmd_pub("cmd_vel2", &cmd_vel);


//std_msgs::Float32 data_y_r;
std_msgs::Float32 drive_y_m;
std_msgs::Float32 steer_y_m;

ros::Publisher drive_y_m_pub("/drive_data_time_plot",&drive_y_m);
ros::Publisher steer_y_m_pub("/steer_data_time_plot",&steer_y_m);
//ros::Publisher error_pub("/error",&error_msg);
//ros::Publisher r_pub("/r",&r_msg);

//}

float target_velocity = 0;
float steer_r = 500;
//float input_velocity=0;
//float input_steer=0;

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  
  target_velocity = (int)msg.linear.x;
  if(target_velocity >= 15){
    target_velocity = 15;
  }
  else if(target_velocity <= -15){
    target_velocity = -15;
  }
  
  steer_r = (int)msg.angular.z;
  if(steer_r >= ad_max){
    steer_r = ad_max;
  }
  else if(steer_r <= ad_min){
    steer_r = ad_min;
  }
  
  control_callback();
}

//ros::Subscriber<std_msgs::Float32> y_r_sub("/data_y_r", y_r_Callback); // 수정된 부분
ros::Subscriber<geometry_msgs::Twist> cmd_sub("teleop_cmd_vel", &cmd_vel_callback);

//==================================================================================

void setup() {
  // put your setup code here, to run once:
  // For checking motor control
  //mySerial.begin(9600);
  // Set gain for front motor control
  front_PID_controller.set_gain(Kp_front, Kd_front, Ki_front);

  // steer PID 게인 및 저주파필터 alpha 설정
  steer_PID_controller.set_alpha(alpha); 
  steer_PID_controller.set_gain(Kp, Ki, Kd);
  steer_PID_controller.set_maxNmin(ad_max, ad_min);

  // Initialize encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);

//  MsTimer2::set(50, getEncoderputDrive);
//  MsTimer2::start();

  
  // ROS
  nh.initNode();
//  nh.subscribe(y_r_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(cmd_pub);
  nh.advertise(drive_y_m_pub);
//
  nh.advertise(steer_y_m_pub);
//  nh.advertise(error_pub);
//  nh.advertise(r_pub);
}


void doEncoderA(){ // 빨녹일 때
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;
}

void doEncoderB(){ // 보파일 때
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;
}

void getEncoderputDrive()
{
  // 20Hz, 50msec마다 증가해야 하는 encoder값 // 속도를 50msec마다 가야하는 거리(pulse)값으로 환산
  float pulse_r = target_velocity * vel_2_pulse; 
  float pulse_y = (float)encoderPos;
  
  int front_motor_pwm;
  
//구동모터 컨트롤
  front_PID_controller.control(pulse_y, pulse_r);

//  pulse_y = front_PID_controller.y_m;
  front_motor_pwm = front_PID_controller.motor_pwm;

  //clearEncoderCount(1);
  encoderPos = 0;
  
  }

//이게 사실상 메인 함수 
void control_callback()
{
  float pot_y_m = float(analogRead(STEERPOT));
  int motor_pwm;
  float error;
  float r;
  //steer_y_m = map(steer_y_m, ad_min, ad_max, min_angle + neural_angle, max_angle + neural_angle); // y_m 범위를 설정 , map함수는 정수형만 가
  
  float steer_motor_pwm;

//조향모터 컨트롤
  steer_PID_controller.control(pot_y_m, steer_r); 
  pot_y_m = steer_PID_controller.y_m;
  steer_motor_pwm = steer_PID_controller.motor_pwm;

  error = steer_PID_controller.error;
  r = steer_r;

// ROS
  drive_y_m.data = front_PID_controller.y_m;
  steer_y_m.data = steer_PID_controller.y_m;
  
  cmd_vel.linear.x = target_velocity;
  cmd_vel.angular.z = steer_r;
  
}

void loop(){
  if ( millis() > last_time_action_done_millis + 50 )
  {
     last_time_action_done_millis = millis();
    
//     drive_y_m_pub.publish(&drive_y_m);
//     steer_y_m_pub.publish(&steer_y_m);
//     cmd_pub.publish(&cmd_vel);

    getEncoderputDrive();
     nh.spinOnce();
  

  }


}

//====================================================================
