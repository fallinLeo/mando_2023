#include "Arduino.h"
#include "MotorPIDController.h"

/*
  motor_pwm 범위 : -255 ~ 255
*/

MotorPIDController::MotorPIDController(uint8_t motor_pin_PWM, uint8_t motor_pin_ENA){
  //모터 핀 번호
  this->motor_pin_PWM = motor_pin_PWM;
  this->motor_pin_ENA = motor_pin_ENA;

  enable_low_pass_filter = false;
  enable_protect_motor = false;

  //모터 핀 번호 지정
  pinMode(motor_pin_PWM, OUTPUT);
  pinMode(motor_pin_ENA, OUTPUT);

  //생성 시 초기화
  error_s = 0.0;
  error_d = 0.0;
  error_old = 0.0;
  y_m_old = 0.0;
  alpha = 0.0; // 0.0 <= alpha < 1.0
  
  motor_pwm = 0;
}

// 게인 설정
void MotorPIDController::set_gain(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

// 알파값 설정
void MotorPIDController::set_alpha(float alpha){
  this->alpha = alpha;
  enable_low_pass_filter = true;
}

// 최대값과 최소값 설정
void MotorPIDController::set_maxNmin(float max, float min){
  this->max_value = max;
  this->min_value = min;
  enable_protect_motor = true;
}

// 저주파 필터 실행
float MotorPIDController::low_pass_filter(float y_m_){
  float y_m_lpf = alpha * y_m_old + (1.0 - alpha) * y_m_;
  y_m_old = y_m;
  return y_m_lpf;
}

// PWM 구하기
int MotorPIDController::get_pwm_by_PID(float y_m, float r){
  //int y_output;
  error = y_m - r;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;
  error_old = error;

  motor_pwm = round(Kp * error + Kd * error_d + Ki * error_s);
  motor_pwm = (motor_pwm >=  254) ?  254 : motor_pwm;   //여기 값 255로 두면 모터작동 안함
  motor_pwm = (motor_pwm <= -254) ? -254 : motor_pwm;  //여기 값 -255로 두면 모터작동 안함
  
  // 예외 처리
  if(enable_protect_motor == true){
    if(max_value <= y_m || min_value >= y_m){ // 읽어온 값이 최대 최소 각을 넘어서면 입력 x
      motor_pwm = 0.0;
      error_s = 0.0;
    }}
  
  if(fabs(error) <= 4.0){   // 목표 값에 거의 도달 했을 때부터는 입력 x
    error_s = 0.0;
    motor_pwm = 0;
  }

  return motor_pwm;
}

// 모터 구동
void MotorPIDController::control(float y_m_, float r){
  if (enable_low_pass_filter == true)
    y_m = low_pass_filter(y_m_);
  else y_m = y_m_;

  motor_pwm = get_pwm_by_PID(y_m, r);

  if (motor_pwm > 0) // forward
  {
     digitalWrite(motor_pin_ENA, HIGH);  //모터 반시계방향
    analogWrite(motor_pin_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
     digitalWrite(motor_pin_ENA, LOW); //모터 시계방향
    analogWrite(motor_pin_PWM, -motor_pwm);
  }
  else // stop
  {
    //digitalWrite(motor_pin_ENA, LOW);
    analogWrite(motor_pin_PWM, 0);
  }
}

/*
  Last edited by Ji Taeho on July 30, 2023
  Last edited by Kim SeoungJun on Aug 9, 2023
*/
