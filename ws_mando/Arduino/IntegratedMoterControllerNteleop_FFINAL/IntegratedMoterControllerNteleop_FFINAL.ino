 #include <MsTimer2.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "FrontMotorPIDController.h"
#include "MotorPIDController.h"
#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>


ros::NodeHandle nh; // 버퍼 크기 조정 가능

int ad_min = 30+ 50.0; // analog data
int ad_max = 1040.0- 50.0; // analog data
int netural = 540.0; //analog data

//========================= ROS 관련 ==============================================
geometry_msgs::Twist cmd_vel;

ros::Publisher cmd_pub("cmd_vel2", &cmd_vel);


//std_msgs::Float32 data_y_r;
std_msgs::Float32 drive_y_m;
std_msgs::Int32 drive_pwm;

std_msgs::Float32 steer_y_m;
//std_msgs::Int32 data_pwm;
//std_msgs::Float32 error_msg;
//std_msgs::Float32 r_msg;

ros::Publisher drive_y_m_pub("/drive_data_time_plot",&drive_y_m);
ros::Publisher drive_pwm_pub("/front_pwm",&drive_pwm);

ros::Publisher steer_y_m_pub("/steer_data_time_plot",&steer_y_m);
//ros::Publisher steer_pwm_pub("/check_pwm",&data_pwm);
//ros::Publisher error_pub("/error",&error_msg);
//ros::Publisher r_pub("/r",&r_msg);

//void y_r_Callback(const std_msgs::Float32& msg) {
//  target_velocity = msg.data;
//}

const int frontLED = A3; // "frontLED"로 핀 이름 변경
const int rearLED = A4; // "rearLED"로 핀 이름 변경


float target_velocity = 0;
int steer_r = 700;
int brake=0;


void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  
//teleop_keyboard에서 값 받아오고 최대최소값 제한하는부분
  brake= (int)msg.linear.y; //if press b(E-STop) -> brake = 1
  if(brake==0)
  {
    if (msg.linear.x >= 0.5)
    {
      target_velocity = (float)msg.linear.x+ 0.25; //0.25더한 이유는 줄어들지 않는 S-S error때문에 입력값을 크게줘서 S-S error를 줄
      }
    else
    {
      target_velocity = (float)msg.linear.x;
      }
    if(target_velocity >= 5){
      target_velocity = 5;
    }
    else if(target_velocity <= -5){
      target_velocity = -5;
    }
  
    steer_r = (int)msg.angular.z;
    if(steer_r >= ad_max){
      steer_r = ad_max;
    }
    else if(steer_r <= ad_min){
      steer_r = ad_min;
    }
  }
  
  control_callback();
}

//ros::Subscriber<std_msgs::Float32> y_r_sub("/data_y_r", y_r_Callback); // 수정된 부분
ros::Subscriber<geometry_msgs::Twist> cmd_sub("teleop_cmd_vel", &cmd_vel_callback);


//==================================================================================

// 1m 당 pulse 수
#define m_2_pulse    348.9      
#define pulse_2_m  1./348.9.
#define vel_2_pulse   m_2_pulse/100. // 10Hz 제어 주기에서 속도와 Δpulse 변환 값
#define pulsePer100millis 34.89

// Front motor pin number
#define MOTOR1_PWM 8  //6
#define MOTOR1_ENA 9  //7

// Rear motor pin number
#define MOTOR2_PWM 6  //8
#define MOTOR2_ENA 7  //9

// Encoder pin
#define encoderPinA 3
#define encoderPinB 2

// Steering motor pin
#define MOTOR3_PWM 4  //4
#define MOTOR3_ENA 5  //5

#define STEERPOT A2

int encoderPos = 0;

//==================================================================================


static float Kp_front = 2.5;
static float Ki_front = 1.7;
static float Kd_front = 0.1;  //0.5


FrontMotorPIDController front_PID_controller(MOTOR1_PWM, MOTOR1_ENA,
                                          MOTOR2_PWM, MOTOR2_ENA);

//==================================================================================

// float steer_r = 600.0; // target pot_value 


//PID 상수 설정
static float Kp = 0.4;  //0.6
static float Ki = 0.25; //0.25
static float Kd = 0.2;  //0.6

static float alpha = 0.3;

bool do_once = true;


MotorPIDController steer_PID_controller(MOTOR3_PWM, MOTOR3_ENA);

//==================================================================================

void setup() {
  pinMode(frontLED, OUTPUT); // "frontLED" 핀을 출력으로 설정
  pinMode(rearLED, OUTPUT); // "rearLED" 핀을 출력으로 설정

  
  // put your setup code here, to run once:
  // For checking motor control
  //mySerial.begin(57600);
  // Set gain for front motor control
  front_PID_controller.set_gain(Kp_front, Ki_front, Kd_front);

  // steer PID 게인 및 저주파필터 alpha 설정
  steer_PID_controller.set_alpha(alpha); 
  steer_PID_controller.set_gain(Kp, Ki, Kd);
  steer_PID_controller.set_maxNmin(ad_max, ad_min);

  // Initialize encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);

  MsTimer2::set(100,clear_encoderPos);
  MsTimer2::start();

  
  // ROS
  nh.initNode();
//  nh.subscribe(y_r_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(cmd_pub);
  nh.advertise(drive_y_m_pub);
  nh.advertise(drive_pwm_pub);
//
  nh.advertise(steer_y_m_pub);
//  nh.advertise(steer_pwm_pub);
//  nh.advertise(error_pub);
//  nh.advertise(r_pub);


}


void doEncoderA(){ // 빨녹일 때
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;
}

void doEncoderB(){ // 보파일 때
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;
}

int front_motor_pwm=0;
int prev_motor_pwm = 0;

void clear_encoderPos()
{
  // 10Hz, 100msec마다 증가해야 하는 encoder값 // 속도를 100msec마다 가야하는 거리(pulse)값으로 환산

  float encoderALPHA = 0.1;
  
  float pulse_r = target_velocity * pulsePer100millis; 
//  float filtered_encoder = (1-encoderALPHA)*filtered_encoder + encoderALPHA * (int)encoderPos;
//  float pulse_y = filtered_encoder;
  float pulse_y = (int)encoderPos;
  //구동모터 컨트롤
  front_PID_controller.control(pulse_y, pulse_r);

  if (target_velocity <= 0.5)
    Ki_front = 0.1;
  else if (target_velocity <= 0.6)
    Ki_front = 0.2;
  else if (target_velocity <= 0.7)
    Ki_front = 0.3;
  else if (target_velocity <= 0.8)
    Ki_front = 0.4;
  else if (target_velocity <= 0.9)
    Ki_front = 0.5;
  else if (target_velocity <= 1.0)
    Ki_front = 0.6;
  else if (target_velocity <= 1.1)
    Ki_front = 0.7;
  else if (target_velocity <= 1.2)
    Ki_front = 0.8;
  else if (target_velocity <= 1.3)
    Ki_front = 0.9;
  else if (target_velocity <= 1.4)
    Ki_front = 1.0;
  else if (target_velocity <= 1.6)
    Ki_front = 1.1;
  else if (target_velocity <= 1.7)
    Ki_front = 1.2;
  else if (target_velocity <= 1.8)
    Ki_front = 1.3;
  else if (target_velocity <= 2.0)
    Ki_front = 1.4;
  else if (target_velocity <= 2.1)
    Ki_front = 1.5;
  else if (target_velocity <= 2.3)
    Ki_front = 1.6;
  else if (target_velocity <= 2.4)
    Ki_front = 1.7;
  else if (target_velocity <= 2.6)
    Ki_front = 1.8;
  else if (target_velocity <= 2.8)
    Ki_front = 1.9;
  else if (target_velocity <= 2.9)
    Ki_front = 2.0;
  else if (target_velocity <= 3.1)
    Ki_front = 2.1;
  else if (target_velocity <= 3.2)
    Ki_front = 2.2;
  else if (target_velocity <= 3.3)
    Ki_front = 2.3;
  else if (target_velocity <= 3.4)
    Ki_front = 2.4;
  else
    Ki_front = 2.5;

  Ki_front = Ki_front-0.1;



  

  
  front_PID_controller.set_gain(Kp_front, Ki_front, Kd_front);

  front_motor_pwm = front_PID_controller.motor_pwm;
  encoderPos = 0;
}


//이게 사실상 메인 함수 
void control_callback()
{
  int pot_y_m = int(analogRead(STEERPOT));
  int motor_pwm;
  int error;
  int r;
  int start_time;
  int steer_motor_pwm;

  if(brake!=0)
  {
    target_velocity=0;
    steer_r = netural;
  }

//조향모터 컨트롤
  steer_PID_controller.control(pot_y_m, steer_r); 
  pot_y_m = steer_PID_controller.y_m;
  steer_motor_pwm = steer_PID_controller.motor_pwm;

  error = steer_PID_controller.error;
  r = steer_r;
//
//  if (abs(prev_motor_pwm-front_motor_pwm)>150)  //갑작스런 큰입력 방지
//  {
//      front_motor_pwm = front_motor_pwm+prev_motor_pwm;
//  }
//      prev_motor_pwm = front_motor_pwm;

// ROS
  drive_y_m.data = front_PID_controller.y_m;
  drive_pwm.data = front_motor_pwm;
  steer_y_m.data = pot_y_m;
//  data_pwm.data = steer_motor_pwm;
//  error_msg.data = error;
//  r_msg.data = steer_r;
  
  drive_y_m_pub.publish(&drive_y_m);
  drive_pwm_pub.publish(&drive_pwm);
//
  steer_y_m_pub.publish(&steer_y_m);
//  steer_pwm_pub.publish(&data_pwm);
//  error_pub.publish(&error_msg);
//  r_pub.publish(&r_msg);
  cmd_vel.linear.x = target_velocity;
  cmd_vel.angular.z = steer_r;
  cmd_vel.linear.y = brake;
  cmd_pub.publish(&cmd_vel);

}

void loop(){
  // PWM 값을 "frontLED" 핀으로 출력
  analogWrite(frontLED, 140);
  analogWrite(rearLED, 140);
  nh.spinOnce();
  delay(10);
}

//====================================================================
