#include "Arduino.h"
#include <MotorPIDController.h>

/*
  motor_pwm 범위 : -255 ~ 255
*/

MotorPIDController::MotorPIDController(uint8_t motor_pin_PWM, uint8_t motor_pin_ENA, uint8_t motor_pin_ENB){

  //모터 핀 번호
  this->motor_pin_PWM = motor_pin_PWM;
  this->motor_pin_ENA = motor_pin_ENA;
  this->motor_pin_ENB = motor_pin_ENB;

  enable_low_pass_filter = false;
  enable_protect_motor = false;

  //모터 핀 번호 지정
  pinMode(motor_pin_PWM, OUTPUT);
  pinMode(motor_pin_ENA, OUTPUT);
  pinMode(motor_pin_ENB, OUTPUT); 

  //생성 시 초기화
  y_m = 0.0;
  error_s = 0.0;
  error_d = 0.0;
  error_old = 0.0;
  y_m_old = 0.0;
  alpha = 0.0; // 0.0 <= alpha < 1.0
}

void MotorPIDController::set_gain(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void MotorPIDController::set_alpha(float alpha){
  this->alpha = alpha;
  enable_low_pass_filter = true;
}

void MotorPIDController::set_maxNmin(float max, float min){
  this->max_value = max;
  this->min_value = min;
  enable_protect_motor = true;
}

float MotorPIDController::low_pass_filter(float y_m){
  float y_m_lpf = alpha * y_m_old + (1.0 - alpha) * y_m;
  y_m_old = y_m;
  return y_m_lpf;
}

int MotorPIDController::get_pwm_by_PID(float y_m, float r){
  int y_output;
  error = y_m - r;
  error_s += error;
  error_d = error - error_old;
  error_s = constrain(error_s, -100, 100);
  error_old = error;
  
  // 예외 처리
  if(enable_protect_motor == true)
    if(max_value <= y_m || min_value >= y_m){ // 읽어온 값이 최대 최소 각을 넘어서면 입력 x
      y_output = 0.0;
      error_s = 0.0;
      return y_output;
    }
  if(fabs(error) <= 1.2){   // 목표 값에 거의 도달 했을 때부터는 입력 x
    error_s = 0.0;
    y_output = 0;
    return y_output;
  }

  y_output = Kp * error + Kd * error_d + Ki * error_s;
  y_output = constrain(y_output, -255, 255);
  return y_output;
}

void MotorPIDController::control(float y_m_, float r){
  if (enable_low_pass_filter == true)
    y_m = low_pass_filter(y_m);
  else y_m = y_m_;

  motor_pwm = get_pwm_by_PID(y_m, r);

  if (motor_pwm > 0) // forward
  {
    digitalWrite(motor_pin_ENA, LOW);
    digitalWrite(motor_pin_ENB, HIGH);
    // digitalWrite(motor_pin_ENA, HIGH);
    // digitalWrite(motor_pin_ENB, LOW);
    analogWrite(motor_pin_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(motor_pin_ENA, HIGH);
    digitalWrite(motor_pin_ENB, LOW);
    // digitalWrite(motor_pin_ENA, LOW);
    // digitalWrite(motor_pin_ENB, HIGH);
    analogWrite(motor_pin_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(motor_pin_ENA, LOW);
    digitalWrite(motor_pin_ENB, LOW);
    analogWrite(motor_pin_PWM, 0);
  }
}

void MotorPIDController::change_direction(){
  uint8_t temp = this->motor_pin_ENA;
  this->motor_pin_ENA = this->motor_pin_ENB;
  this->motor_pin_ENB = temp;
}

/*
  Last edited by Ji Taeho on July 30, 2023
*/
