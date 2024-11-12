#include <Wire.h>
#include <LSM303.h>

#define ENL 5        // 모터 핀 정의
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENR 10

// 왼쪽 엔코더 핀
#define ENCODER_A_C1_PIN 50  // 왼쪽 엔코더 C1 핀
#define ENCODER_A_C2_PIN 51  // 왼쪽 엔코더 C2 핀

// 오른쪽 엔코더 핀
#define ENCODER_B_C1_PIN 52  // 오른쪽 엔코더 C1 핀
#define ENCODER_B_C2_PIN 53  // 오른쪽 엔코더 C2 핀

LSM303 compass;

float target_heading_angle = 90;        // 목표 회전 각도
float kp_yaw = 0.25;                    // PID 제어 파라미터
float kd_yaw = 0.4;
float error_yaw = 0.0;
float error_yaw_old = 0.0;
float pid_out;
float target_yaw;
int mission_flag;
/*int base_speed = 100;*/

double robot_distance = 0.0;            // 전체 로봇 주행 거리
long encoder_A_pulse = 0;                  // 왼쪽 엔코더 펄스 값
long encoder_B_pulse = 0;                  // 오른쪽 엔코더 펄스 값
long prev_encoder_A = 0;                // 이전 왼쪽 엔코더 값
long prev_encoder_B = 0;                // 이전 오른쪽 엔코더 값

int current_encoder_A = 0;
int current_encoder_B = 0;
const double meter_per_pulse = 0.01428;  // 펄스당 이동 거리

// 엔코더 값을 업데이트하는 함수 
void update_encoder_values() {
  current_encoder_A = digitalRead(ENCODER_A_C1_PIN);  // C1 핀 값 읽기
  current_encoder_B = digitalRead(ENCODER_B_C1_PIN);  // C2 핀 값 읽기
}

void reset_encoder(void) {
  encoder_A_pulse = 0;                  // 왼쪽 엔코더 값 초기화
  encoder_B_pulse = 0;                  // 오른쪽 엔코더 값 초기화
  prev_encoder_A = 0;
  prev_encoder_B = 0;
}

double update_distance(void) {
  long delta_A = current_encoder_A - prev_encoder_A; // 왼쪽 엔코더 변화량 계산
  long delta_B = current_encoder_B - prev_encoder_B; // 오른쪽 엔코더 변화량 계산

  double distance = ((delta_A + delta_B) * meter_per_pulse) / 2.0; // 평균 이동 거리 계산
  robot_distance += distance;                // 누적 주행 거리 반영

  prev_encoder_A = current_encoder_A;
  prev_encoder_B = current_encoder_B;

  return distance;                           // 현재 이동 거리 반환
}

void motor_l(int speed) {
  if (speed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENL, speed);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENL, -speed);
  }
}

void motor_r(int speed) {
  if (speed >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENR, speed);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENR, -speed);
  }
}

void motor_control(int left_motor_speed, int right_motor_speed) {
  motor_l(left_motor_speed);
  motor_r(right_motor_speed);
}

void yaw_control(void) {
  float error_yaw_d;
  int l_motor_speed;
  int r_motor_speed;

  compass.read();
  float heading_angle = compass.heading();

  error_yaw = target_yaw - heading_angle;
  error_yaw_d = error_yaw - error_yaw_old;
  pid_out = kp_yaw * error_yaw + kd_yaw * error_yaw_d;
  error_yaw_old = error_yaw;

  l_motor_speed = base_speed + (int)pid_out;
  r_motor_speed = base_speed - (int)pid_out;

  motor_control(l_motor_speed, r_motor_speed);
}

void setup() {
  pinMode(ENL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENR, OUTPUT);

  Serial.begin(115200);

  Wire.begin();
  compass.init();
  compass.enableDefault();

  mission_flag = 0;
  reset_encoder();
  robot_distance = 0.0;
}

void loop() {
  switch (mission_flag) {
    case 0: // 미션 0: 로봇 정지 후 초기화
      motor_control(0, 0);
      delay(500);
      reset_encoder();
      robot_distance = 0.0;
      mission_flag = 1;
      break;

    case 1: // 미션 1: 1미터 직진
      motor_control(100, 86);
      update_distance();
      if (robot_distance >= 1.0) {
        target_yaw = 90 + compass.heading();
        yaw_control();
        mission_flag = 2;
      }
      break;

    case 2: // 미션 2: 회전 각도 조정
      if ((error_yaw < 5) && (error_yaw > -5)) // 현재 방향이 목표와 오차 3도 이내라면
      {
        motor_control(0, 0);             // 모터를 멈춤
        mission_flag = 3;                // 미션 3으로 전환
      }
      else
      {
        yaw_control();                   // 오차가 크면 회전 제어 계속 진행
      }
      break;

    case 3: // 미션 3: 대기
      // 추가 미션을 수행할 수 있는 구간
      break;
  }
}
