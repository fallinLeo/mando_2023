#ifndef MOTROPIDCONTROLLER_H
#define MOTROPIDCONTROLLER_H

#include "Arduino.h"

class MotorPIDController
{
  public:
    MotorPIDController(uint8_t motor_pin_PWM, uint8_t motor_pin_ENA, uint8_t motor_pin_ENB); // 생성자

    bool enable_low_pass_filter; // low pass filter를 쓸지 말지 정하는 변수
    bool enable_protect_motor; // 최대 최소 입력 값을 넘겼을 때 입력을 0으로 줄지말지에 대한 함수

    //low pass filter에 사용할 변수 - 외부에서 접근 가능
    float y_m;
    
    void control(float y_m_, float r); // 모터 제어 실행 함수
    void set_gain(float Kp, float Ki, float Kd); // 게인값 받는 함수
    void set_alpha(float alpha);
    void set_maxNmin(float max, float min);
    void change_direction(); // PWM 방향이 반대일 때 쓸 함수

  private:
    //모터 핀 번호
    uint8_t motor_pin_PWM;
    uint8_t motor_pin_ENA;
    uint8_t motor_pin_ENB;

    //motor protect에 사용할 변수
    float max_value;
    float min_value;

    //low pass filter에 사용할 변수
    float alpha;
    float y_m_old;

    //PID 제어에 사용할 변수들
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_s;
    float error_d;
    float error_old;
    int motor_pwm;
    //PID 제어 사용 전 거칠 함수들
    float low_pass_filter(float y_m);
    int get_pwm_by_PID(float y_m, float r);
};

#endif

/*
  Last edited by Ji Taeho on July 30, 2023
*/
