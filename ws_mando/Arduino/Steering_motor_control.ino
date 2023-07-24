#include <MsTimer2.h>

#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

// 실험으로 바꿔야되는 값
#define STEER_AD_MIN (40 + 10)
#define STEER_AD_MAX (950 - 10)

#define alpha 0.3 // alpha < 1
int steer_r = 560; // Desired value 값(analog) / 실험 때 값 바꿔보면서 진행하기

float steer_y_m_old = 0.0; // xavg(k-1)
float steer_y_m_lpf = 0.0; // xavg(k)
int steer_y_m = 0;

//PID 상수 설정, 실험에 따라 정해야 함 중요!
static float Kp = 0.97;
static float Ki = 0.18;
static float Kd = 2.05;

double error, error_old;
double error_s, error_d;
int steer_y = 0;

// 이게 사실상 메인 함수
void Timer_ISR(void)
{
  steer_y_m = analogRead(A15); // steer 현재 값 받아오기 0~1023
  steer_y_m = map(steer_y_m, STEER_AD_MIN, STEER_AD_MAX, -255, 255); // 0~1023 => -255~255

  steer_y_m_lpf = low_pass_filter(steer_y_m, steer_y_m_old); // 받아온 센서값을 로우 패스 필터 적용
  steer_y = PID_Control(steer_r, steer_y_m_lpf); // PID 제어를 통해 y값 도출
  steer_motor_control(steer_y); // y값에 맞는 모터 제어
}

float low_pass_filter(int ad_value, float &avg_old)
{
	// 퍼텐셔미터 노이즈 줄이기
  float avg = alpha * avg_old + (1.0 - alpha) * ad_value;
  avg_old = avg;
  return avg;
}

int PID_Control(int r, float y_m)
{
  error = r - y_m;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;

  int y = round(Kp * error + Kd * error_d + Ki * error_s);
  y = (y >=  255) ?  255 : y;
  y = (y <= -255) ? -255 : y;

  if (fabs(error) <= 1.2)
  {
    y = 0;
    error_s = 0;
  }
  error_old = error;

  return y
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  MsTimer2::set(10,Timer_ISR);
  MsTimer2::start();

  //Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("steer_y_m = "); Serial.print(steer_y_m);
  Serial.print("avg  = "); Serial.println(avg);
}