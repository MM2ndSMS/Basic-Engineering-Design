#include <MsTimer2.h>
#include <NewPing.h>

#define A0pin A0
#define SIpin 23
#define CLKpin 22
#define NPIXELS 128

#define ENA 7
#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 8
#define ENB 6

#define TRIG_PIN1 32
#define ECHO_PIN1 33
#define TRIG_PIN2 41
#define ECHO_PIN2 40
#define TRIG_PIN3 37
#define ECHO_PIN3 36

#define SIZE 5
#define MAX_DISTANCE 255

#define motor_speed_offset 30

float sensorData1[SIZE] = {0};
float sensorData2[SIZE] = {0};
float sensorData3[SIZE] = {0};

float UltrasonicSensorData[3];

NewPing sonar1(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);


byte Pixel[NPIXELS];
byte Threshold_Data[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

int count = 0;

const int IMG_WIDTH_HALF = 64;
const int BASE_SPEED = 70;
const float KP = 7.0;
const float KD = 0.2;
float error_old = 0.0;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int mission_flag = 0;

void setup()
{
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

  Serial.begin(115200);
  Serial.println("TSL1401");
}

/////////////////////////////////////////////////

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


float recursive_moving_average1(float ad_value, float sensorData[])
{
  static float avg = 0.0;
  float sum = 0.0;

  for (int i = 0; i <= SIZE - 2; i++)
  {
    sensorData[i] = sensorData[i + 1];
    //Serial.print(sensorData[i]);
    //Serial.print(" ");
  }

  sensorData[SIZE - 1] = ad_value;
  //Serial.println(sensorData[SIZE - 1]);

  for (int i = 0; i < SIZE; i++)
  {
    sum += sensorData[i];
  }

  avg = sum / SIZE;

  return avg;
}

float get_average(float ad_value, float sensorData[])
{
  return recursive_moving_average1(ad_value, sensorData);
}

void calculate_and_print(float distance, float sensorData[], int sensorNumber)
{
  float average = get_average(distance, sensorData);
  //Serial.print("Sensor ");
  //Serial.print(sensorNumber);
  //Serial.print(" Distance: ");
  //Serial.println(distance);
  //Serial.print("Moving Average: ");
  //Serial.println(average);
}

void read_ultrasonic_sensor()
{
  UltrasonicSensorData[0] = sonar1.ping_cm(); // 앞 센서
  UltrasonicSensorData[1] = sonar2.ping_cm(); // 좌 센서
  UltrasonicSensorData[2] = sonar3.ping_cm(); // 우 센서

  if (UltrasonicSensorData[0] == 0) UltrasonicSensorData[0] = 255;
  if (UltrasonicSensorData[1] == 0) UltrasonicSensorData[1] = 255;
  if (UltrasonicSensorData[2] == 0) UltrasonicSensorData[2] = 255;

  calculate_and_print(UltrasonicSensorData[0], sensorData1, 1);
  calculate_and_print(UltrasonicSensorData[1], sensorData2, 2);
  calculate_and_print(UltrasonicSensorData[2], sensorData3, 3);
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
  int left_speed  = BASE_SPEED + speed_difference - 25;

  left_speed = constrain(left_speed, 0, 100);
  right_speed = constrain(right_speed, 0, 100);
  //Serial.println(left_speed);
  //Serial.println(right_speed);


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

void read_line_center()
{
  int line_center = 64;

  double cx = 0;
  read_line_camera();
  threshold_line_image(60);
  cx = line_COM();

  if (cx == -1)
  {
    motor_control(0, 0);
    delay(1000);
    Serial.println("선 없음.");
    count = 1;
    return;
  }
  else
  {
    line_center  = line_COM();
    line_control(line_center);

    for (int i = 0; i < NPIXELS; i++)
    {
      /*
        Serial.print(Pixel[i]);
        Serial.print(",");
        Serial.print(Threshold_Data[i]);
        Serial.print(",");
        Serial.print((i == (int)cx) ? 255 : 0);
        Serial.println();
      */
    }
    delay(50);
  }
}

void read_line_center_2()
{
  int line_center = 64;

  double cx = 0;
  read_line_camera();
  threshold_line_image(60);
  cx = line_COM();

  if (cx == -1)
  {
    motor_control(0, 0);
    Serial.println("선 없음.");
    motor_control(0, 0);
    return;
  }
  else
  {
    line_center  = line_COM();
    line_control(line_center);

    for (int i = 0; i < NPIXELS; i++)
    {
      /*
        Serial.print(Pixel[i]);
        Serial.print(",");
        Serial.print(Threshold_Data[i]);
        Serial.print(",");
        Serial.print((i == (int)cx) ? 255 : 0);
        Serial.println();
      */
    }
    delay(50);
  }
}

void wall_following_l(int distance, int base_speed)
{
  int i;
  float kp = 1.0;
  float kd = 10;
  int l_speed = 0;
  int r_speed = 0;
  float error = 0;
  float d_error = 0;
  float error_old = 0;
  float speed_control;
  int speed_control_max = 10;

  read_ultrasonic_sensor();
  error = UltrasonicSensorData[1] - distance;

  d_error = error - error_old;
  speed_control = kp * error + kd * d_error;

  if (speed_control >= speed_control_max) speed_control = speed_control_max;
  if (speed_control <= -speed_control_max) speed_control = -speed_control_max;

  l_speed = base_speed - speed_control - 30;
  r_speed = base_speed + speed_control + 10;
  error_old = error;
  /*
    Serial.print(error);
    Serial.print("  ");
    Serial.print(l_speed);
    Serial.print("  ");
    Serial.println(r_speed);
    Serial.print("  ");
    Serial.println(speed_control);
  */

  motor_control(l_speed + motor_speed_offset, r_speed);
}

void loop()
{
  read_ultrasonic_sensor();
  Serial.print("mission: ");
  Serial.print(mission_flag);

  switch (mission_flag)
  {
    // start_stop
    case 0:

      if (UltrasonicSensorData[2] >= 100)
      {
        read_line_center();
      }
      if (UltrasonicSensorData[2] < 100)
      {
        mission_flag = 1;
        break;
      }


      break;

    // lane_control
    case 1:

      read_ultrasonic_sensor();
      if (UltrasonicSensorData[2] < 255)
      {
        wall_following_l(70, 80);
        //delay(100);
      }

      if (UltrasonicSensorData[2] >= 255)
      {
        motor_control(70, 80);
        delay(2400);
        motor_control(0, 0);
        delay(1000);
        motor_l(255); // Move left forward
        motor_r(-255); // Move right backward
        delay(375);
        motor_control(100, 100);
        delay(2000);
        motor_control(0, 0);
        delay(200);
        mission_flag = 2;
      }

      break;


    // wall following
    case 2:

      if (UltrasonicSensorData[2] < 255)
      {
        read_ultrasonic_sensor();
        wall_following_l(40, 100);
      }

      if (UltrasonicSensorData[2] >= 255)
      {
        mission_flag = 3;
      }
      break;

    case 3:

      if (UltrasonicSensorData[0] > 50)
      {
        read_ultrasonic_sensor();
        wall_following_l(40, 80);
      }

      if (UltrasonicSensorData[0] <= 50)
      {
        motor_control(0, 0);
        delay(2000);
        motor_l(255); // Move left forward
        motor_r(-255); // Move right backward
        delay(280);
        motor_control(0, 0);
        delay(1000);
        mission_flag = 4;
      }

      break;

    case 4:

      read_ultrasonic_sensor();
      delay(100);
      motor_control(80, 60);
      delay(3500);
      mission_flag = 5;

      break;

    case 5:

      if (UltrasonicSensorData[2] < 255)
      {
        read_ultrasonic_sensor();
        wall_following_l(50, 100);
      }

      if (UltrasonicSensorData[2] >= 255)
      {
        motor_control(0, 0);
        delay(1000);
        mission_flag = 6;
      }

      break;

    case 6:

      while (1)
      {
        read_line_center_2();
      }

      break;

  }

}
