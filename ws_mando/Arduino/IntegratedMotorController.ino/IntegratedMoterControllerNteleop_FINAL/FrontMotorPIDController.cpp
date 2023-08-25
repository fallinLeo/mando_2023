#include "Arduino.h"
#include "FrontMotorPIDController.h"

FrontMotorPIDController::FrontMotorPIDController(uint8_t front_motor_pin_PWM, uint8_t front_motor_pin_ENA, uint8_t rear_motor_pin_PWM, uint8_t rear_motor_pin_ENA)
{
  this->front_motor_pin_PWM = front_motor_pin_PWM;
  this->front_motor_pin_ENA = front_motor_pin_ENA;
  //this->front_motor_pin_ENB = front_motor_pin_ENB;
  
  this->rear_motor_pin_PWM = rear_motor_pin_PWM;
  this->rear_motor_pin_ENA = rear_motor_pin_ENA;
  //this->rear_motor_pin_ENB = rear_motor_pin_ENB;

  //Set motor pin number
  pinMode(front_motor_pin_PWM, OUTPUT);
  pinMode(front_motor_pin_ENA, OUTPUT);
  //pinMode(front_motor_pin_ENB, OUTPUT); 

  pinMode(rear_motor_pin_PWM, OUTPUT);
  pinMode(rear_motor_pin_ENA, OUTPUT);
  //pinMode(rear_motor_pin_ENB, OUTPUT); 

  //Check the executions of primary functions
  enable_gain_setting = false;
  
  //Reset the parameters
  error_s = 0.0;
  error_d = 0.0;
  error_old = 0.0;
  y_m_old = 0.0;
  motor_pwm = 0;
}

// Control front and rear motor
void FrontMotorPIDController::control(float y_m, float r){
  this->y_m = y_m;
  
  if(enable_gain_setting == false){
    return;
  }

  motor_pwm = get_pwm_by_PID(y_m, r);
  
  front_motor_control(motor_pwm);
  rear_motor_control(motor_pwm);
}

// Set Gain
void FrontMotorPIDController::set_gain(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  enable_gain_setting = true;
}

// Calculate Gain
int FrontMotorPIDController::get_pwm_by_PID(float y_m, float r){
  //int y_output;
  error = r - y_m;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;
  error_old = error;

  motor_pwm = round(Kp * error + Kd * error_d + Ki * error_s);
  motor_pwm = (motor_pwm >=  255) ?  255 : motor_pwm;
  motor_pwm = (motor_pwm <= -255) ? -255 : motor_pwm;  
  
  if(fabs(error) <= 2.0){   // 목표 값에 거의 도달 했을 때부터는 입력 x
    error_s = 0.0;
    motor_pwm = 0;
  }
  
  return motor_pwm;
}

// Front motor control
void FrontMotorPIDController::front_motor_control(int motor_pwm)
{
  if (motor_pwm > 0) // forward
  {
  digitalWrite(front_motor_pin_ENA, HIGH);
  analogWrite(front_motor_pin_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
  digitalWrite(front_motor_pin_ENA, LOW);
  analogWrite(front_motor_pin_PWM, -motor_pwm);
  }
  else
  {
  digitalWrite(front_motor_pin_ENA, LOW);
  digitalWrite(front_motor_pin_PWM, 0);
  }
}

// Rear motor control
void FrontMotorPIDController::rear_motor_control(int motor_pwm)
{
  if (motor_pwm > 0) // forward
  {
  digitalWrite(rear_motor_pin_ENA, HIGH);
  analogWrite(rear_motor_pin_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
  digitalWrite(rear_motor_pin_ENA, LOW);
  analogWrite(rear_motor_pin_PWM, -motor_pwm);
  }
  else
  {
  digitalWrite(rear_motor_pin_ENA, LOW);
  digitalWrite(rear_motor_pin_PWM, 0);
  }
}
