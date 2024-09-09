void setup()
{
  // put your setup code here, to run once:
  Serial.println("test");
  Serial.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  
  for (int i = 0; i < 100; i++)
  {
    Serial.println(i);
    delay(10);
  }

}
