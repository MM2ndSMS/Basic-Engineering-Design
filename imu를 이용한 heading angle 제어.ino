/////////////////IMU/////////////////
#include <Wire.h>
#include <LSM303.h>

#define ENL 5        // 모터 핀 정의
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENR 10
#define THRESHOLD_ANGLE 10
#define motor_offset 10

LSM303 compass;

float heading_angle = 0.0;
float init_heading_angle = 17.0;
float target_heading_angle = 90;
float heading_angle_error = 0.0;

float kp_yaw = 0.25;
float kd_yaw = 0.4;
float error_yaw = 0.0;
float error_yaw_old = 0.0;
float target_angle;
float pid_out;
float target_yaw;
int mission_flag;
int base_speed = 100;

void setup()
{
  //put your setup code here, to run once:

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

  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  mission_flag = 0;
}

void motor_l(int speed)
{
    if (speed >= 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENL, speed); // 0-255
    }
    else
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENL, -speed);
    }
}

void motor_r(int speed)
{
    if (speed >= 0)
    {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENR, speed); // 0-255
    }
    else
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENR, -speed);
    }
}

void motor_control(int left_motor_speed, int right_motor_speed)
{
    motor_l(left_motor_speed);
    motor_r(right_motor_speed);
}

void yaw_control(void)
{
  float error_yaw_d = 0.0;
  int l_motor_speed = 0;
  int r_motor_speed = 0;
  compass.read();
  float heading_angle1 = compass.heading();
  compass.read();
  float heading_angle2 = compass.heading();

  float heading_angle = (heading_angle1 + heading_angle2) / 2;

  error_yaw = target_angle - heading_angle;
  error_yaw_d = error_yaw - error_yaw_old;
  pid_out = kp_yaw * error_yaw + kd_yaw * error_yaw_d;
  error_yaw_old = error_yaw;
  l_motor_speed = base_speed + (int)pid_out;
  r_motor_speed = - base_speed + motor_offset - (int)pid_out;
  motor_control(l_motor_speed, r_motor_speed);

}


void loop()
{
  switch (mission_flag)
  {
    case 0 : 
      /*while(1)
        motor_control(50,50);*/
      compass.read();
      heading_angle = compass.heading();
      target_angle = 100 + compass.heading();
      error_yaw = target_angle - heading_angle;
      mission_flag = 1;
      break;
    case 1 :
      if ((error_yaw < 5) && (error_yaw > -5))
      {
        motor_control(0,0);
        mission_flag = 2;
      }
      else
      {
        yaw_control();
      }
      break;
    case 2 :
      break;
  }
}
