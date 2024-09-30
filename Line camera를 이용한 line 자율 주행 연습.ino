```c
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

byte Pixel[NPIXELS];
byte Threshold_Data[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

const int IMG_WIDTH_HALF = 64;
const int BASE_SPEED = 70;
const float KP = 7.0;
const float KD = 0.2;
float error_old = 0.0;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

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
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed); // 0-255
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
  int left_speed  = BASE_SPEED + speed_difference-25;

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

void loop()
{
  int line_center = 64;

  double cx = 0;
  read_line_camera();
  threshold_line_image(150);
  cx = line_COM();

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
```
