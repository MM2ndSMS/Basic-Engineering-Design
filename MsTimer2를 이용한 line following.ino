#include <MsTimer2.h>
#define test_pin 44

// TSL1401 라인 스캔 카메라 모듈 핀 정의
#define NPIXELS 128    // 카메라의 픽셀 수
#define CLKpin 39      // 카메라 CLK (클럭) 핀
#define SIpin 38       // 카메라 SI (시작 신호) 핀
#define A0pin A0       // 카메라 데이터 입력 핀


// 모터 제어 핀 정의
#define ENA 3          // 왼쪽 모터 속도 제어 핀
#define IN1 2          // 왼쪽 모터 방향 제어 핀 1
#define IN2 1         // 왼쪽 모터 방향 제어 핀 2
#define IN3 0          // 오른쪽 모터 방향 제어 핀 1
#define IN4 14          // 오른쪽 모터 방향 제어 핀 2
#define ENB 15         // 오른쪽 모터 속도 제어 핀

// 픽셀 데이터를 저장할 배열
byte Pixel[NPIXELS];              // 카메라에서 읽은 원본 데이터
byte Threshold_Data[NPIXELS];     // 임계값을 적용한 데이터

// 라인 센서 관련 데이터 배열
int LineSensor_Data[NPIXELS];             // 원본 센서 데이터
int LineSensor_Data_Adaption[NPIXELS];    // 적응형 센서 데이터
int MAX_LineSensor_Data[NPIXELS];         // 센서 데이터의 최대값
int MIN_LineSensor_Data[NPIXELS];         // 센서 데이터의 최소값
int flag_line_adapation;                  // 라인 적응 플래그

// 주행 제어에 필요한 상수 및 변수 정의
const int IMG_WIDTH_HALF = 64;     // 이미지 폭의 절반
const int BASE_SPEED = 100;         // 기본 모터 속도
const float KP = 3.4;              // P 제어 상수
const float KD = 1.7;              // D 제어 상수
float error_old = 0.0;             // 이전 에러 값 (D 제어에 사용)

// 고속 ADC 설정 관련
#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  // 비트 클리어 매크로
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   // 비트 설정 매크로

void setup()
{
  // 타이머 인터럽트 설정 (주기: 180ms, line_tracer 함수 호출)
  MsTimer2::set(180, line_tracer);
  MsTimer2::start();

  // 센서 데이터 초기화
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023;
    MIN_LineSensor_Data[i] = 0;
  }

  // 카메라 제어 핀 설정
  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(A0pin, INPUT);

  // 초기 신호 값 설정
  digitalWrite(SIpin, LOW);
  digitalWrite(CLKpin, LOW);

  // 고속 ADC 설정
#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;

  // 테스트 핀 설정
  pinMode(test_pin, OUTPUT);

  // 시리얼 통신 시작
  Serial.begin(115200);
}

/////////////////////////////////////////////////////
// 왼쪽 모터 제어 함수
void motor_l(int speed)
{
  if (speed >= 0)
  {
    digitalWrite(IN1, HIGH);     // 전진
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);     // 속도 설정 (0-255)
  }
  else
  {
    digitalWrite(IN1, LOW);      // 후진
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);    // 속도 설정
  }
}

// 오른쪽 모터 제어 함수
void motor_r(int speed)
{
  if (speed >= 0)
  {
    digitalWrite(IN3, HIGH);     // 전진
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);     // 속도 설정 (0-255)
  }
  else
  {
    digitalWrite(IN3, LOW);      // 후진
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);    // 속도 설정
  }
}

// 모터 제어 함수 (좌/우 모터 속도를 각각 설정)
void motor_control(int left_motor_speed, int right_motor_speed)
{
  motor_l(left_motor_speed);
  motor_r(right_motor_speed);
}

/////////////////////////////////////////////////////
// 라인 센서 데이터를 임계값을 기준으로 이진화 (흑/백)
void threshold_line_image(int threshold_value)
{
  digitalWrite(test_pin, HIGH);
  Serial.println("HIGH");
  for (int i = 0; i < NPIXELS; i++)
  {
    if (Pixel[i] >= threshold_value)
    {
      Threshold_Data[i] = 255;   // 흰색으로 판단
    }
    else
    {
      Threshold_Data[i] = 0;     // 검은색으로 판단
    }
  }
  digitalWrite(test_pin, LOW);
  Serial.println("LOW");
}

// 라인 주행을 위한 PID 제어 함수
void line_control(int line_center)
{
  digitalWrite(test_pin, HIGH);

  // 라인의 중심 위치에 따른 에러 계산
  int error = line_center - IMG_WIDTH_HALF;
  int derivative = error - error_old;
  float output = KP * error + KD * derivative;
  int speed_difference = int(output);

  // 모터 속도 계산 (기본 속도에서 에러 보정값 적용)
  int right_speed = BASE_SPEED - speed_difference;
  int left_speed = BASE_SPEED + speed_difference;

  // 모터 속도를 0~100 범위로 제한
  left_speed = constrain(left_speed, 0, 100);
  right_speed = constrain(right_speed, 0, 100);

  // 디버그용 시리얼 출력
  Serial.println(left_speed);
  Serial.println(right_speed);

  // 모터 속도 제어
  motor_control(left_speed, right_speed);

  // 이전 에러 값 업데이트
  error_old = error;
  digitalWrite(test_pin, LOW);
}

/////////////////////////////////////////////////////
// 카메라로 라인 데이터 읽기 함수
void read_line_camera(void)
{
  digitalWrite(test_pin, HIGH);
  Serial.println("HIGH");

  int i;
  delay(1);  // 카메라 안정화 대기

  // 카메라 초기화 신호 전송
  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, HIGH);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(SIpin, LOW);
  delayMicroseconds(1);

  // 128개의 픽셀 데이터를 읽어옴
  for (i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead(A0pin) / 4;  // 아날로그 값을 8비트로 스케일링
    digitalWrite(CLKpin, LOW);
    delayMicroseconds(1);
    digitalWrite(CLKpin, HIGH);
  }
  
  digitalWrite(CLKpin, LOW);
  digitalWrite(test_pin, LOW);
  Serial.println("LOW");
}

/////////////////////////////////////////////////////
// 라인의 중심 (Center of Mass, COM) 계산 함수
double line_COM(void)
{
  digitalWrite(test_pin, HIGH);

  double COM = 0.0;
  double mass_sum = 0.0;

  // 픽셀 데이터 기반으로 질량 중심 계산
  for (int i = 0; i < NPIXELS; i++)
  {
    mass_sum += Threshold_Data[i];
    COM += Threshold_Data[i] * i;
  }

  if (mass_sum == 0)
  {
    return -1;  // 라인이 없을 때는 -1 반환
  }

  COM = COM / mass_sum;

  digitalWrite(test_pin, LOW);
  return COM;
}

/////////////////////////////////////////////////////
// 라인 트레이서 실행 함수 (주기적 호출)
void line_tracer()
{
  double cx = 64;  // 초기값 설정
  read_line_camera();          // 라인 카메라 데이터 읽기 (약 16.45ms 소요)
  threshold_line_image(180);   // 임계값 적용하여 이진화 (0.5ms 소요)
  cx = line_COM();             // 질량 중심 계산 (0.005ms 소요)
  line_control(cx);            // 라인 주행 제어 (0.6ms 소요)
}

void loop()
{
  line_tracer();
}
