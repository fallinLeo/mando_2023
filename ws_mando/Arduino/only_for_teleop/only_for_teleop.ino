//rosrun rosserial_python serial_node.py _baud:=57600 _port:=/dev/ttyACM0
#include <MsTimer2.h>
#include "FrontMotorPIDController.h"
#include <SPI.h>
#define ENC1_ADD 22
#define ENC2_ADD 23
//STEERING MOTOR
#define MOTOR3_PWM 8
#define MOTOR3_IN1 9
#define MOTOR3_IN2 10
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

#define STEER_AD_MIN (40 + 10)
#define STEER_AD_MAX (950 - 10)

float linear_x;
float linear_y;
float linear_z;
float angular_x;
float angular_y;
float angular_z;

//===========encoder============
signed long encoder1count = 0;
signed long encoder2count = 0;
signed long encoder1_error = 0;
signed long encoder2_error = 0;
signed long encoder1_error_d = 0;
signed long encoder2_error_d = 0;
signed long encoder1_target = 0;
signed long encoder2_target = 0;
signed long encoder1_error_old = 0; 
signed long encoder2_error_old = 0; 
signed long encoder1_error_sum = 0; 
signed long encoder2_error_sum = 0; 
float target_velocity1 = 0.0;
float target_velocity2 = 0.0;
//==============encoder end=================
void cmd_vel_callback(const geometry_msgs::Twist& msg);
void steer_motor_control(int);
//========velocity publish for ros (choice)
//geometry_msgs::Twist cmd_vel;

//forward motor control object
FrontMotorPIDController front_PID_controller(MOTOR1_PWM, MOTOR1_ENA, MOTOR2_PWM, MOTOR2_ENA);
ros::NodeHandle nh;

std_msgs::Int32 Encoder1count;
std_msgs::Int32 Encoder2count;

//ros subsrciber and publisher
ros::Subscriber<geometry_msgs::Twist> cmd_sub("teleop_cmd_vel", cmd_vel_callback);
//ros::Publisher cmd_pub("cmd_vel2", &cmd_vel);
ros::Publisher encoder_pub1("encoder1", &Encoder1count);
ros::Publisher encoder_pub2("encoder2", &Encoder2count);

int input_velocity=0, vel_gap = 0;
int velocity = 0;
int steer_angle = 0, input_steer = 0, steer_gap = 0;
int brake = 0;
int f_speed = 0, r_speed = 0;

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  input_velocity = (int)msg.linear.x;
  input_steer = (int)msg.angular.z;
  velocity += input_velocity;
  steer += input_steer;
  //brake = (int)msg.linear.z;
  velocity = constrain(velocity,-255,255);
  steer = constrain(steer,-255,255);
  steer = map(steer,STEER_AD_MIN,STEER_AD_MAX,-255,255);
  steer_motor_control(steer);
  front_PID_controller.front_motor_control(velocity);
  front_PID_controller.rear_motor_control(velocity);
  //vel_gap = velocity - input_velocity;
  //steer_gap = steer - input_steer;
  delay(10);
}






void setup() {
  // 타이머 함수
  //MsTimer2::set(10, control_callback);
  //MsTimer2::start();
  Serial.begin(57600);
  pinMode(MOTOR1_PWM,OUTPUT);
  pinMode(MOTOR1_ENA,OUTPUT);
  pinMode(MOTOR1_ENB,OUTPUT);
  pinMode(MOTOR1_PWM,OUTPUT);
  pinMode(MOTOR1_ENA,OUTPUT);
  pinMode(MOTOR1_ENB,OUTPUT);
  //encoder
  pinMode(13,OUTPUT);
  //initEncoders();
  //clearEncoderCount(1);
  //clearEncoderCount(2);
  // ROS
  nh.initNode();
  nh.subscribe(cmd_sub);
  //nh.advertise(cmd_pub);
}

void control_callback(void)
{
  static boolean output = HIGH;
  digitalWrite(13, output);
  output = !output;
  vel_gap = input_velocity - velocity;
  steer_gap = input_steer - steer_angle;
  if(brake == 0)
  {
    if(fabs(vel_gap) > 0 && fabs(vel_gap) < 10)//
    {
      velocity += vel_gap;
    }
    else if(vel_gap >= 10)
    {
      velocity += 10;
    }
    else if(vel_gap <= -10)
    {
      velocity -= 10;
    }
  }
  else velocity = input_velocity;
  
  if(velocity >= 255) velocity = 255; //pwm
  else if(velocity <=-255) velocity = -255;//pwm

  f_speed = r_speed = velocity;
  motor_control(f_speed, r_speed);

  if(fabs(steer_gap)>0 && fabs(steer_gap)<10)//
  {
    steer_angle += steer_gap;
  }
  else if(steer_gap > 10)
  {
    steer_angle += 10;
  }
  else if(steer_gap < -10)
  {
    steer_angle -= 10;
  }
  encoder1count = readEncoder(1);
//  encoder1_error = encoder1_target - encoder1count;
//  encoder1_error_sum += encoder1_error;
//  encoder1_error_d = encoder1_error - encoder1_error_old;
//  encoder1_error_sum = (encoder1_error_sum >=  100) ?  100 : encoder1_error_sum;
//  encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;
}

//===========motor steer control fuc==============


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


//===============motor steer fuc end==========

void loop()
{
  //motor_control(f_speed,r_speed);
//   velocity publish
  cmd_vel.linear.x = velocity;
  cmd_vel.angular.z = steer_angle;
  cmd_pub.publish(&cmd_vel);
  delay(10);
  nh.spinOnce();
}

//encoder fuc=====


void initEncoders() {
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT); 
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);
  SPI.begin();
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 
  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}


//encoder fun end


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
