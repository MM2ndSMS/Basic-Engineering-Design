#define A0pin A0           // A0 핀을 A0pin으로 정의
#define SIpin 23           // 통합 시작 핀을 22번으로 정의
#define CLKpin 22          // 클럭 핀을 23번으로 정의
#define NPIXELS 128        // 픽셀의 개수를 128로 정의

byte Pixel[NPIXELS];       // 측정된 값을 저장할 배열 (범위: 0~255)
byte Threshold_Data[NPIXELS]; // 이진화된 값을 저장할 배열

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;   // 적응형 처리를 위한 플래그

#define FASTADC 1          // 빠른 ADC 모드를 사용할지 여부
// 레지스터 비트를 설정 및 해제하기 위한 매크로
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  // 비트를 클리어하는 매크로
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   // 비트를 설정하는 매크로

void setup()
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;            // 초기화: 센서 데이터 값을 0으로 설정
    LineSensor_Data_Adaption[i] = 0;   // 초기화: 적응형 데이터 값을 0으로 설정
    MAX_LineSensor_Data[i] = 1023;     // 최대 값을 1023으로 초기화
    MIN_LineSensor_Data[i] = 0;        // 최소 값을 0으로 초기화
  }
  pinMode(SIpin, OUTPUT);   // SI 핀을 출력 모드로 설정
  pinMode(CLKpin, OUTPUT);  // CLK 핀을 출력 모드로 설정
  pinMode(A0pin, INPUT);    // A0 핀을 입력 모드로 설정

  digitalWrite(SIpin, LOW);  // SI 핀을 낮은 상태로 설정 (IDLE 상태)
  digitalWrite(CLKpin, LOW); // CLK 핀을 낮은 상태로 설정 (IDLE 상태)

#if FASTADC
  // ADC 프리스케일을 16으로 설정하여 빠르게 샘플링
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;  // 적응형 플래그를 0으로 초기화

  Serial.begin(115200);     // 시리얼 통신을 115200 baud rate로 시작
  Serial.println("TSL1401"); // "TSL1401"라는 메시지를 출력 (센서 이름)
}

void threshold_line_image(int threshold_value)
{
  for (int i = 0; i < NPIXELS; i++)
  {
    if (Pixel[i] >= threshold_value)
    {
      Threshold_Data[i] = 255; // 임계값 이상인 경우 흰색(255)로 설정
    }
    else
    {
      Threshold_Data[i] = 0;   // 임계값 미만인 경우 검정(0)으로 설정
    }
  }
}

void read_line_camera(void)
{
  int i;
  delay(1); // 1ms 지연 (카메라 안정화 대기)

  digitalWrite(CLKpin, LOW);  // CLK 핀을 낮은 상태로 설정
  digitalWrite(SIpin, HIGH);  // SI 핀을 높은 상태로 설정 (통합 시작)
  digitalWrite(CLKpin, HIGH); // CLK 핀을 높은 상태로 설정
  digitalWrite(SIpin, LOW);   // SI 핀을 낮은 상태로 설정
  delayMicroseconds(1);       // 1마이크로초 지연

  for (i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead(A0pin) / 4; // A0 핀에서 값을 읽고 8비트로 변환
    digitalWrite(CLKpin, LOW);        // CLK 핀을 낮은 상태로 설정
    delayMicroseconds(1);             // 1마이크로초 지연
    digitalWrite(CLKpin, HIGH);       // CLK 핀을 높은 상태로 설정
  }
  digitalWrite(CLKpin, LOW); // CLK 핀을 낮은 상태로 설정
}

double line_COM(void)
{
  double COM = 0.0; // 질량중심을 계산할 변수
  double mass_sum = 0;   // 전체 질량 합을 저장할 변수

  for (int i = 0; i < NPIXELS; i++)
  {
    mass_sum += Threshold_Data[i]; // 질량 합산
    COM += Threshold_Data[i] * i; // 가중치를 곱해 합산
  }

  if (mass_sum == 0)
  {
    return -1; // 질량이 없을 때 -1 반환 (오류 처리)
  }

  COM = COM / mass_sum; // 질량중심 계산
  return COM; // 계산된 질량중심 반환
}

void loop()
{
  double cx = 0;
  read_line_camera();         // 카메라 센서 데이터 읽기
  threshold_line_image(50);   // 임계값을 50으로 설정하여 이진화
  cx = line_COM();       // 질량중심 계산

  // 카메라에서 읽은 데이터, 이진화된 값, 질량중심 값을 시리얼로 출력
  for (int i = 0; i < NPIXELS; i++)
  {
    Serial.print(Pixel[i]);                    // 원본 카메라 데이터 출력
    Serial.print(",");                         // 쉼표로 구분
    Serial.print(Threshold_Data[i]); // 이진화된 데이터 출력
    Serial.print(",");                         // 쉼표로 구분
    Serial.print(cx);                          // 질량중심 값 출력
    Serial.println();                          // 줄바꿈
  }

  delay(100); // 데이터가 읽기 쉽도록 약간의 지연 추가
}
