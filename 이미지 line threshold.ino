#define TSL1401_CLK 22  // CLK 핀 정의
#define TSL1401_SI 23   // SI 핀 정의
#define TSL1401_AO A0   // AO 핀 정의
#define NPIXELS 128     // 픽셀 수 정의

byte Pixel[NPIXELS]; // 측정된 값을 저장할 배열 <0-255>

#define FASTADC 1       // 빠른 ADC 모드 활성화

// 레지스터 비트를 클리어하는 매크로
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// 레지스터 비트를 설정하는 매크로
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

byte threshold_data[NPIXELS]; // 임계값 처리된 데이터를 위한 배열

// 픽셀 데이터를 임계값으로 처리하는 함수
void line_threshold(int threshold)
{
  // i를 선언하고, 적절히 증가시켜 NPIXELS 동안 반복
  for(int i = 0; i < NPIXELS; i++)  
  {
    // 픽셀 값이 임계값보다 클 경우
    if (Pixel[i] > threshold) 
    {
      threshold_data[i] = 255; // 255를 할당
    } 
    else 
    {
      threshold_data[i] = 0; // 임계값 이하일 경우 0 할당
    }
    
  }
}

void setup()
{
    Serial.begin(115200);  // 시리얼 통신 시작 (115200 baud)
    Serial.println("Camera Initialized"); // 카메라 초기화 메시지 출력

    pinMode(TSL1401_CLK, OUTPUT); // CLK 핀을 출력으로 설정
    pinMode(TSL1401_SI, OUTPUT);  // SI 핀을 출력으로 설정

    digitalWrite(TSL1401_CLK, LOW); // 초기 CLK 값을 LOW로 설정
    digitalWrite(TSL1401_SI, LOW);  // 초기 SI 값을 LOW로 설정

    #if FASTADC
      // ADC 프리스케일러를 16으로 설정
      sbi(ADCSRA, ADPS2);
      cbi(ADCSRA, ADPS1);
      cbi(ADCSRA, ADPS0);
    #endif
}

// TSL1401 카메라에서 데이터를 읽는 함수
void read_TSL1401_camera(void)
{
    // SI와 CLK 신호 설정
    digitalWrite(TSL1401_SI, HIGH); 
    digitalWrite(TSL1401_CLK, HIGH);
    delayMicroseconds(1); // 1마이크로초 지연

    digitalWrite(TSL1401_CLK, LOW);
    digitalWrite(TSL1401_SI, LOW);
    delayMicroseconds(1);

    // NPIXELS만큼 데이터 읽기
    for(int i = 0; i < NPIXELS; i++)
    {
        digitalWrite(TSL1401_CLK, HIGH);
        delayMicroseconds(1);
        Pixel[i] = analogRead(TSL1401_AO); // 아날로그 데이터를 읽어 Pixel 배열에 저장
        digitalWrite(TSL1401_CLK, LOW);
        delayMicroseconds(1);
    }
}

// 카메라 데이터를 시리얼 통신으로 전송하는 함수
void send_camera_data_serial()
{
    // NPIXELS 동안 반복하여 픽셀 데이터 전송
    for(int i = 0; i < NPIXELS; i++)
    {
        Serial.print(Pixel[i]); // 각 픽셀 값을 시리얼로 출력
        Serial.print(",");
        Serial.println(threshold_data[i]); // 임계값 처리된 데이터 출력
    }
  
    // 추가적으로 64번 0을 출력
    for(int i = 0; i < 64; i++)
    {
        Serial.print(0);
        Serial.print(",");
        Serial.println(0); 
    }
}

void loop() 
{
    read_TSL1401_camera(); // 카메라 데이터 읽기
    line_threshold(150); // 임계값 150으로 데이터 처리
    send_camera_data_serial(); // 처리된 데이터 전송
    delay(100); // 100ms 지연
}
