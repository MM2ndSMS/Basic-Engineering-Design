const byte ledPin = 13;
const byte interruptPin = 2;
const byte encoder_A_B  = 24;
volatile byte state = LOW;
static long encoder_A_pulse_cnt = 0;

void setup() {
  pinMode(ledPin, OUTPUT);

  pinMode(interruptPin, INPUT_PULLUP);

  pinMode(encoder_A_B,INPUT_PULLUP);

  encoder_A_pulse_cnt = 0;

  attachInterrupt(digitalPinToInterrupt(interruptPin), EncoderA, RISING);

  Serial.begin(115200);
}

void loop() {
  Serial.println(encoder_A_pulse_cnt);
}

void EncoderA() {
  encoder_A_pulse_cnt++;
}

void EncoderAB() {
  if(digitalRead(encoder_A_B) == LOW)
  {
    encoder_A_pulse_cnt++;
  }
  else
  {
    encoder_A_pulse_cnt--;
  } 
}













  
