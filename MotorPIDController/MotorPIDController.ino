#include "MotorPIDController.h"
#include <MsTimer2.h>
#include "Arduino.h"

//========================= ROS 관련 ==============================================
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
std_msgs::Float32 data_y_m;
std_msgs::Int32 data_pwm;
std_msgs::Float32 error_msg;
std_msgs::Float32 r_msg;




ros::Publisher y_m_pub("/data_time_plot",&data_y_m);
ros::Publisher pwm_pub("/check_pwm",&data_pwm);
ros::Publisher error_pub("/error",&error_msg);
ros::Publisher r_pub("/r",&r_msg);
//geometry_msgs::Twist cmd_vel;

//모터드라이버 사용하면서, 5,6번만 사용함.
//5번이 PWM, 6번이 ENA
#define MOTOR3_PWM 8  //8 9 10
#define MOTOR3_ENA 9
#define MOTOR3_ENB 7

//==================================================================================

float steer_r = 300.0; // pot_value
float neural_angle = 0.0; // degree
float min_angle = -20.0; // degree
float max_angle = 20.0; // degree
float ad_min = 90.0; // analog data
float ad_max = 940.0; // analog data

//PID 상수 설정
static float Kp = 0.4;
static float Ki = 0.21;
static float Kd = 1.0;

static float alpha = 0.3;

bool do_once = true;


MotorPIDController steer_PID_controller(MOTOR3_PWM, MOTOR3_ENA, MOTOR3_ENB);

//==================================================================================
void setup() {
  // steer PID 게인 및 저주파필터 alpha 설정
  steer_PID_controller.set_alpha(alpha); 
  steer_PID_controller.set_gain(Kp, Ki, Kd);
  steer_PID_controller.set_maxNmin(ad_max, ad_min);
  //steer_PID_controller.change_direction();

  // ROS
  nh.initNode();
  nh.advertise(y_m_pub);
  nh.advertise(pwm_pub);
  nh.advertise(error_pub);
  nh.advertise(r_pub);
  
  // 타이머 함수
  MsTimer2::set(50, control_callback);
  
  MsTimer2::start();
}

// 이게 사실상 메인 함수
void control_callback(void)
{
  float steer_y_m = float(analogRead(A15));
  int motor_pwm;
  float error;
  float r;
  int start_time;
  //steer_y_m = map(steer_y_m, ad_min, ad_max, min_angle + neural_angle, max_angle + neural_angle); // y_m 범위를 설정 , map함수는 정수형만 가

  Serial.print("control_callback");
  Serial.println(error);
  if(do_once == true){
    start_time = millis();
    do_once=false;
    Serial.print("DO_ONCE");
    Serial.print(start_time);
    }
  if (millis()-start_time > 1400){
  steer_PID_controller.control(steer_y_m, steer_r); 
    Serial.print("CONTROLLER");
    
  }
  steer_y_m = steer_PID_controller.y_m;
  motor_pwm = steer_PID_controller.motor_pwm;

  error = steer_PID_controller.error;
  r = steer_r;

  // ROS
  data_y_m.data = steer_y_m;
  data_pwm.data = motor_pwm;
  error_msg.data = error;
  r_msg.data = r;
  
  y_m_pub.publish(&data_y_m);
  pwm_pub.publish(&data_pwm);
  error_pub.publish(&error_msg);
  r_pub.publish(&r_msg);

  nh.spinOnce();
}

void loop() {
  
}

/*
  Last edited by Ji Taeho on July 30, 2023
*/
