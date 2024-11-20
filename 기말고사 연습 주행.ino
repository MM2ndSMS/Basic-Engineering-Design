#include <MsTimer2.h>
#include <LSM303.h>

#define test_pin 45
#define A0pin A0
#define SIpin 18
#define CLKpin 17
#define NPIXELS 128

#define ENA 44
#define IN1 43
#define IN2 42
#define IN3 41
#define IN4 40
#define ENB 39

#define base_speed 70
#define Grid_Size 0.4

float target_heading_angle = 90;
float kp_yaw = 3.1;
float kd_yaw = 0.4;
float error_yaw = 0.0;
float error_yaw_old = 0.0;
float pid_out;
float target_yaw;

LSM303 compass;

byte Pixel[NPIXELS];
byte Threshold_Data[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

long encoder_A_pulse = 0;
long encoder_B_pulse = 0;
long prev_encoder_A = 0;
long prev_encoder_B = 0;

const int IMG_WIDTH_HALF = 64;
const int BASE_SPEED = 70;
const float KP = 1.6;
const float KD = 0.4;
float error_old = 0.0;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

float robot_distance = 0.0;
float wheel_circumference = 20.0;
int pulses_per_rev = 360;

int mission_flag = 0;
int function_flag = 0;

struct Waypoint 
{
  double distance;
  double heading_angle;
};

Waypoint waypoints[20];
int waypoint_no = 0;

void setup() 
{
  reset_encoder();
  MsTimer2::set(20, MsTimer2_ISR);
  MsTimer2::start();

  for (int i = 0; i < NPIXELS; i++) 
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

  pinMode(test_pin, OUTPUT);

  Serial.begin(115200);

  waypoints[0] = {0, 0};            
  waypoints[1] = {2 * Grid_Size, 0};
  waypoints[2] = {0, 90};           
  waypoints[3] = {3 * Grid_Size, 90};
  waypoints[4] = {0, 0};             
  waypoints[5] = {0, -90};         
  waypoints[6] = {1 * Grid_Size, -90}; 
  waypoints[7] = {0, -90};

}

void reset_encoder() 
{
  encoder_A_pulse = 0;
  encoder_B_pulse = 0;
  prev_encoder_A = 0;
  prev_encoder_B = 0;
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

void motor_l(int speed) 
{
  if (speed >= 0) 
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
  } 
  else 
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -speed);
  }
}

void motor_r(int speed) 
{
  if (speed >= 0) 
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  } 
  else 
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speed);
  }
}

void motor_control(int left_motor_speed, int right_motor_speed) 
{
  motor_l(left_motor_speed);
  motor_r(right_motor_speed);
}

void read_line_camera(void)
{
  int i;
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

void update_robot_distance() 
{
  long delta_A = encoder_A_pulse - prev_encoder_A;
  long delta_B = encoder_B_pulse - prev_encoder_B;

  float avg_pulse = (delta_A + delta_B) / 2.0;
  robot_distance += (avg_pulse / pulses_per_rev) * wheel_circumference;

  //Serial.print("Distance: ");
  //Serial.println(robot_distance);

  prev_encoder_A = encoder_A_pulse;
  prev_encoder_B = encoder_B_pulse;
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

void threshold_line_image(int threshold_value)
{
  digitalWrite(test_pin, HIGH);
  for (int i = 0; i < NPIXELS; i++)
  {
    if (Pixel[i] >= threshold_value)
    {
      Threshold_Data[i] = 255;
    } else {
      Threshold_Data[i] = 0;
    }
  }
  digitalWrite(test_pin, LOW);
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

  motor_control(left_speed, right_speed);
  error_old = error;
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
  switch (mission_flag)
  {
    case 0: 
      motor_control(0, 0);
      delay(500);
      robot_distance = 0.0;  
      function_flag = 1;  
      mission_flag = 1;   
      break;

    case 1: 
      if (robot_distance >= waypoints[waypoint_no].distance) 
      {  
        function_flag = 3; 
        delay(500);       
        target_yaw = waypoints[waypoint_no].heading_angle - compass.heading(); 
        function_flag = 2; 
        mission_flag = 2;  
        waypoint_no++;     
      }
      break;

    case 2: 
      yaw_control(); 
      if (error_yaw < 5 && error_yaw > -5) 
      {  
        function_flag = 3; 
        delay(500);
        reset_encoder();
        mission_flag = 3;  
      }
      break;

    case 3:
      if (robot_distance >= waypoints[waypoint_no].distance) 
      {  
        function_flag = 3; 
        delay(500);      
        target_yaw = waypoints[waypoint_no].heading_angle - compass.heading(); 
        function_flag = 2; 
        mission_flag = 4;  
        waypoint_no++;     
      }
      break;

    case 4: 
      yaw_control(); 
      if (error_yaw < 5 && error_yaw > -5) 
      {  
        function_flag = 3; 
        delay(500);
        reset_encoder();
        mission_flag = 5;
      }
      break;

    case 5:
      if (robot_distance >= waypoints[waypoint_no].distance) 
      {  
        function_flag = 3;
        delay(500);        
        target_yaw = waypoints[waypoint_no].heading_angle - compass.heading();
        function_flag = 2;
        mission_flag = 6; 
        waypoint_no++;    
      }
      break;

    case 6: 
      yaw_control();
      if (error_yaw < 5 && error_yaw > -5) 
      {  
        function_flag = 3; 
        delay(500);
        mission_flag = 7;
      }
      break;

    case 7:
      motor_control(0, 0);  
      break;
  }
}
