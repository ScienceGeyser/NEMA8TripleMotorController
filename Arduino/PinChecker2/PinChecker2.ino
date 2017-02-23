static const uint8_t analogPin[] = {A0,A1,A2,A3,A4,A5};
String AnalogString = "";

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);
  analogWrite(10, 105);
  for (int i=0; i<=5; i++ )
  {
    
    Serial.print("A");
    Serial.print(i);
    Serial.print(" = Digital ");
    Serial.println(analogPin[i]);
  }

}

void loop() {

}
