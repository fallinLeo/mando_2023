#include <MotorPIDController.h>
#include <MsTimer2.h>


//========================= ROS 관련 ==============================================
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
 
ros::NodeHandle nh;
std_msgs::Float32 data; // 변수 형태, 변수 형태 

ros::Publisher data_pub("/data_time_plot",&data); // 토픽, 데이터
// Front Motor Drive
#define MOTOR1_PWM 2
#define MOTOR1_IN1 3
#define MOTOR1_IN2 4
//Rear Motor Drive
#define MOTOR2_PWM 5
#define MOTOR2_IN1 6
#define MOTOR2_IN2 7

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

//=================bluetooth============================
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2,3);
void bluetoothcheck(char);
void steer_motor_control(int);
void front_motor_control(int);
void rear_motor_control(int);
int motor_steer=0;
int motor_front=0;
int motor_rear=0;


//======================================================

//==================================================================================
void setup() {
  // serial 보더레이트
  Serial.begin(9600);
  mySerial.begin(9600);
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
  bluetoothcheck(st);
  // ROS
  data.data = steer_y_m; // 아두이노 상에서 float을 Float32 메세지 형식인 ~~.data에 대입하는
  data_pub.publish(&data);
  delay(10);
  nh.spinOnce();
}



void bluetoothcheck(char st)
{
  if(mySerial.available()>0)
  {
    st = mySerial.read();
    //forward();
    if(st=='S')
    {
      steer_motor_control(motor_steer);
      front_motor_control(motor_front);
      rear_motor_control(motor_rear);
      Serial.println(st);
    }
    else if(st=='F')
    {
      motor_front=motor_rear=250;
      steer_motor_control(motor_steer);
      front_motor_control(motor_front);
      rear_motor_control(motor_rear);
      Serial.println(st);
    }
      
    }
    else if(st=='B')
    {
      motor_front=motor_rear=-250;
      steer_motor_control(motor_steer);
      front_motor_control(motor_front);
      rear_motor_control(motor_rear);
      Serial.println(st);
    }
    else if(st=='L')
    {
      motor_steer=200;
      motor_front=motor_rear=250;
      
      Serial.println(st);
    }
    else if(st=='R')
    {
      motor_steer=-200;
      motor_front=motor_rear=250; 
      Serial.println(st);
    }
    else if(st=='I')
    {
      //FRward();
      Serial.println(st);
    }
    else if(st=='J')
    {
      //RBward();
      Serial.println(st);
    }
    else if(st=='H')
    {
      //LBward();
      Serial.println(st);
    }
    else if(st=='G')
    {
      //FLward();
      Serial.println(st);
    }
}

void steer_motor_control(int motor_pwm)
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
}
void front_motor_control(int motor1_pwm)
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
void rear_motor_control(int motor2_pwm)
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


/*
  Last edited by Ji Taeho on July 30, 2023
*/
