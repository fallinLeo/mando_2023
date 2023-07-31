#include <MotorPIDController.h>
#include <MsTimer2.h>


//========================= ROS 관련 ==============================================
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
 
ros::NodeHandle nh;
std_msgs::Float32 data; // 변수 형태, 변수 형태 

ros::Publisher data_pub("/data_time_plot",&data); // 토픽, 데이터

#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

//==================================================================================
int steer_r = 15; // degree
float neural_angle = 0.0; // degree
float min_angle = -20.0; // degree
float max_angle = 20.0; // degree
int ad_min = 40; // analog data
int ad_max = 900; // analog data

//PID 상수 설정
static float Kp = 1.0;
static float Ki = 0.0;
static float Kd = 0.0;

static float alpha = 0.3;

MotorPIDController steer_PID_controller(MOTOR3_PWM, MOTOR3_ENA, MOTOR3_ENB);

//==================================================================================
void setup() {
  // serial 보더레이트
  Serial.begin(9600);

  // 타이머 함수
  MsTimer2::set(10, control_callback);
  MsTimer2::start();

  // steer PID 게인 및 저주파필터 alpha 설정
  steer_PID_controller.set_alpha(alpha); 
  steer_PID_controller.set_gain(Kp, Ki, Kd);
  steer_PID_controller.set_maxNmin(min_angle, max_angle);
  steer_PID_controller.change_direction();

  // ROS
  nh.initNode();
  //nh.subscribe
  nh.advertise(data_pub);
}

// 이게 사실상 메인 함수
void control_callback(void)
{
  steer_y_m = map(analogRead(A15), ad_min, ad_max, min_angle + neural_angle, max_angle + neural_angle); // y_m 범위를 설정
  steer_PID_controller.control(steer_y_m, steer_r);
  steer_y_m = steer_PID_controller.y_m;
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.print("steer_r = "); Serial.print(steer_r);
  // Serial.print("steer_y_m = "); Serial.print(steer_y_m);
  // Serial.print("steer_y_m_lpf  = "); Serial.println(steer_y_m_lpf);

  // ROS
  data.data = steer_y_m; // 아두이노 상에서 float을 Float32 메세지 형식인 ~~.data에 대입하는
  data_pub.publish(&data);
  delay(10);
  nh.spinOnce();
}

/*
  Last edited by Ji Taeho on July 30, 2023
*/
