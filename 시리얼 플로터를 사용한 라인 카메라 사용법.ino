#define TSL1401_CLK 50        // TSL1401 카메라의 CLK 핀을 Arduino의 핀 50에 연결
#define TSL1401_SI 51         // TSL1401 카메라의 SI 핀을 Arduino의 핀 51에 연결
#define NPIXELS 120           // 카메라에서 읽을 픽셀 수를 120으로 정의

byte Pixel[NPIXELS];          // 측정된 값(0-255)을 저장할 배열, 픽셀 값을 저장
int LineSensor_Data[NPIXELS]; // 라인 센서 데이터를 저장할 배열

#define FASTADC 1             // 고속 ADC 설정을 위해 정의, ADC 변환 속도를 빠르게 설정

// 레지스터 비트를 설정하고 해제하는 매크로 정의
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  // 특정 레지스터에서 비트 해제
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   // 특정 레지스터에서 비트 설정

void setup()
{
    Serial.begin(115200);          // 시리얼 통신을 115200bps로 시작
    Serial.println("Camera");      // "Camera"라는 메시지를 출력

    digitalWrite(TSL1401_CLK, LOW); // TSL1401의 CLK 핀을 LOW로 초기화
    digitalWrite(TSL1401_SI, LOW);  // TSL1401의 SI 핀을 LOW로 초기화

    pinMode(TSL1401_CLK, OUTPUT);   // TSL1401의 CLK 핀을 출력 모드로 설정
    pinMode(TSL1401_SI, OUTPUT);    // TSL1401의 SI 핀을 출력 모드로 설정
    
    #if FASTADC
      // ADC의 프리스케일러를 16으로 설정하여 변환 속도를 빠르게 설정
      sbi(ADCSRA, ADPS2);          // ADPS2 비트를 설정 (프리스케일러를 16으로 설정)
      cbi(ADCSRA, ADPS1);          // ADPS1 비트를 해제
      cbi(ADCSRA, ADPS0);          // ADPS0 비트를 해제
    #endif
}

void read_TSL1401_camera(void)
{
  // TSL1401 카메라에서 데이터를 읽기 위한 시퀀스 시작
  digitalWrite(TSL1401_CLK, HIGH);   // CLK 핀을 HIGH로 설정
  delayMicroseconds(1);              // 1 마이크로초 대기
  digitalWrite(TSL1401_CLK, LOW);    // CLK 핀을 다시 LOW로 설정
  delayMicroseconds(1);              // 1 마이크로초 대기

  digitalWrite(TSL1401_SI, HIGH);    // SI 핀을 HIGH로 설정 (시작 신호)
  delayMicroseconds(1);              // 1 마이크로초 대기
  digitalWrite(TSL1401_SI, LOW);     // SI 핀을 다시 LOW로 설정
  delayMicroseconds(1);              // 1 마이크로초 대기

  // 128개의 픽셀 데이터를 읽음
  for(int i = 0; i < 128; i++)
  {
    Pixel[i] = analogRead(A0);       // A0 핀에서 아날로그 값을 읽어 픽셀 배열에 저장
    digitalWrite(TSL1401_CLK, HIGH); // CLK 핀을 HIGH로 설정
    delayMicroseconds(1);            // 1 마이크로초 대기
    digitalWrite(TSL1401_CLK, LOW);  // CLK 핀을 다시 LOW로 설정
    delayMicroseconds(1);            // 1 마이크로초 대기
  }
}

void send_camera_data_serial()
{
  // 읽은 128개의 픽셀 데이터를 시리얼로 전송
  for(int i = 0; i < 128; i++)
  {
    Serial.println(Pixel[i]);        // 각 픽셀 값을 시리얼로 출력
  }
}

void loop() 
{
  read_TSL1401_camera();             // 카메라에서 데이터를 읽음
  send_camera_data_serial();         // 읽은 데이터를 시리얼로 전송
}
