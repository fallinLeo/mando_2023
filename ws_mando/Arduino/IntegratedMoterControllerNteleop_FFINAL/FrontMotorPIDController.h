#ifndef FRONTMOTROPIDCONTROLLER_H
#define FRONTMOTROPIDCONTROLLER_H

#include "Arduino.h"

class FrontMotorPIDController
{
  public: 
    FrontMotorPIDController(uint8_t front_motor_pin_PWM, uint8_t front_motor_pin_ENA, uint8_t rear_motor_pin_PWM, uint8_t rear_motor_pin_ENA);
    
    int motor_pwm;
    float y_m;

    void set_gain(float Kp, float Ki, float Kd);
    void control(float y_m, float r);
    int get_pwm_by_PID(float y_m, float r);

    float error;

    
  private:
    uint8_t front_motor_pin_PWM;
    uint8_t front_motor_pin_ENA;

    uint8_t rear_motor_pin_PWM;
    uint8_t rear_motor_pin_ENA;
  
    float Kp;
    float Ki;
    float Kd;
  
    bool enable_gain_setting;

    float error_s;
    float error_d;
    float error_old;
    float y_m_old;

    void front_motor_control(int motor_pwm);
    void rear_motor_control(int motor_pwm);
};

#endif
