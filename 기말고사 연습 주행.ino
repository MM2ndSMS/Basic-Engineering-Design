#include <MsTimer2.h>
#include <LSM303.h>

#define Grid_Size 0.4

#define wpSIZE 20

#define A0pin A0
#define SIpin 31
#define CLKpin 30
#define NPIXELS 128

#define ENA 7
#define IN1 8
#define IN2 9
#define IN3 11
#define IN4 12
#define ENB 13
#define BASE_SPEED 70

float target_heading_angle = 90;
float kp_yaw = 0.21;
float kd_yaw = 0.4;
float error_yaw = 0.0;
float error_yaw_old = 0.0;
float pid_out;
float target_yaw;

LSM303 compass;

byte Pixel[NPIXELS];
byte Threshold_Data[NPIXELS];

int mission_flag = 0;
int function_flag = 3;

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

const int IMG_WIDTH_HALF = 64;
const float KP = 3.0;
const float KD = 0.2;
float error_old = 0.0;
int base_speed =100;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

const byte outPin = 13; // Output pin: digital pin 13(D13)
//const byte interruptPin1 = 2; // Interrupt pin: D2
//const byte interruptPin2 = 3; // Interrupt pin: D2
const byte encoder1_A = 20; // Interrupt pin: D2
const byte encoder1_B = 21; // Interrupt pin: D2
const byte encoder2_A = 18; // Interrupt pin: D2
const byte encoder2_B = 19; // Interrupt pin: D2
const byte resetPin = 5;
volatile byte state = 0;
volatile long encoderpos;

//unsigned long cnt1 = 0; // 추가
//unsigned long cnt2 = 0; // 추가
long cnt1 = 0; // 추가
long cnt2 = 0; // 추가

struct waypoint{
  double distance;
  double heading_angle;
};

waypoint wayway[wpSIZE];
int waypoint_way = 0;

void setup()
{
  reset_encoder();
  MsTimer2::set(20, MsTimer2_ISR);
  MsTimer2::start();

  pinMode(outPin, OUTPUT); // Output mode
  //  pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  //  pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  //  attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
  //  attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt

  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023;
    MIN_LineSensor_Data[i] = 0;
  }
  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(A0pin, INPUT);

  digitalWrite(SIpin, LOW);
  digitalWrite(CLKpin, LOW);

#if FASTADC

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;

  wayway[0] = {2 * Grid_Size, 90};
  wayway[1] = {1 * Grid_Size, 0};
  wayway[2] = {3 * Grid_Size, 90};
  wayway[3] = {0, 0};
  wayway[4] = {0, -90};
  wayway[5] = {1 * Grid_Size, -90};
  wayway[6] = {0, -90};

  Serial.begin(115200);
}

void reset_encoder()
{
  encoderpos = cnt1 = cnt2 = 0;

}

void MsTimer2_ISR()
{
  switch (function_flag)
  {
    case 1:
      line_tracer();
      break;
    case 2:
      yaw_control();
      break;
    case 3:
    default:
      motor_control(0, 0);
      break;
  }
}


void intfunc1()
{

  cnt1++;

}

void intfunc2()
{

  cnt2++;

}

void motor_l(int speed)
{
  if (speed >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed); // 0-255
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  }
}

void motor_r(int speed)
{
  if (speed >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed); // 0-255
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  }
}

void motor_control(int left_motor_speed, int right_motor_speed)
{
  motor_l(left_motor_speed);
  motor_r(right_motor_speed);
}

////////////////////////////////////////////////////

void threshold_line_image(int threshold_value)
{
  for (int i = 0; i < NPIXELS; i++)
  {
    if (Pixel[i] >= threshold_value)
    {
      Threshold_Data[i] = 255;
    }
    else
    {
      Threshold_Data[i] = 0;
    }
  }
}

void line_control(int line_center)
{
  int error = line_center - IMG_WIDTH_HALF;
  int derivative = error - error_old;
  float output = KP * error + KD * derivative;
  int speed_difference = int(output);

  int right_speed = BASE_SPEED - speed_difference;
  int left_speed  = BASE_SPEED + speed_difference;

  left_speed = constrain(left_speed, 0, 100);
  right_speed = constrain(right_speed, 0, 100);
  Serial.println(left_speed);
  Serial.println(right_speed);


  motor_control(left_speed, right_speed);

  error_old = error;
}

void read_line_camera(void)
{
  int i;
  delay(1);

  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, HIGH);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(SIpin, LOW);
  delayMicroseconds(1);

  for (i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead(A0pin) / 4;
    digitalWrite(CLKpin, LOW);
    delayMicroseconds(1);
    digitalWrite(CLKpin, HIGH);
  }
  digitalWrite(CLKpin, LOW);
}

double line_COM(void)
{
  double COM = 0.0;
  double mass_sum = 0.0;

  for (int i = 0; i < NPIXELS; i++)
  {
    mass_sum += Threshold_Data[i];
    COM += Threshold_Data[i] * i;
  }

  if (mass_sum == 0)
  {
    return -1;
  }

  COM = COM / mass_sum;

  return COM;
}

void yaw_control() 
{
  float error_yaw_d;
  int l_motor_speed;
  int r_motor_speed;

  compass.read();
  float heading_angle = compass.heading();

  error_yaw = target_yaw - heading_angle;
  if (error_yaw > 180) error_yaw -= 360;
  else if (error_yaw < -180) error_yaw += 360;

  error_yaw_d = error_yaw - error_yaw_old;
  pid_out = kp_yaw * error_yaw + kd_yaw * error_yaw_d;
  error_yaw_old = error_yaw;

  l_motor_speed = base_speed + (int)pid_out;
  r_motor_speed = base_speed - (int)pid_out;

  motor_control(l_motor_speed, r_motor_speed);
}

void line_tracer() 
{
  double cx = 64;
  read_line_camera();
  threshold_line_image(150);
  cx = line_COM();
  line_control(cx);
}

void loop()
{
  int line_center = 64;

  line_center  = line_COM();

  encoderpos = 10 * ((cnt1 + cnt2) / 2 * 0.028);

switch (mission_flag)
  {
    case 0: // 초기화
      function_flag = 3;
      delay(500);
      reset_encoder();
      function_flag = 1; // 라인 트레이싱 시작
      mission_flag = 1;
      break;

    case 1: // 거리 이동 (라인 트레이싱)
      if (encoderpos >= wayway[waypoint_way].distance)
      {
        function_flag = 3; // 정지
        delay(500);
        target_yaw = wayway[waypoint_way].heading_angle; // 목표 각도 설정
        function_flag = 2; // 각도 조정 시작
        mission_flag = 2;
      }
      break;

    case 2: // 각도 조정
      if (abs(error_yaw) < 5) 
      {
        function_flag = 3; // 정지
        delay(500);
        reset_encoder();
        mission_flag = 3;
      }
      break;

    case 3: // 다음 Waypoint로 전환
      waypoint_way++; // 다음 Waypoint로 이동
      if (waypoint_way <= wpSIZE)
      {
        mission_flag = 0; // 초기화 단계로 돌아가기
      }
      else
      {
        mission_flag = 4; // 모든 Waypoint 완료
      }
      break;

    case 4: // 종료 상태
      motor_control(0, 0); // 정지
      break;
  }
  delay(50);
}
