#include <Wire.h>

int buf[16];
int16_t val;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
}


void readCO() {
  int sensorCO = analogRead(A0);
  Serial.print("CO ");
  Serial.println(sensorCO);
}

float readSensor(uint8_t regAddr) {
  buf[0] = 0x00;
    Wire.beginTransmission(0x40 << 1) ;
    Wire.write(regAddr);
    Wire.write(1);
    Wire.endTransmission();

    Wire.requestFrom(0x40 << 1, 2);
    buf[0] = Wire.read();
    val = ((uint16_t) buf[0] << 8);
    val |= buf[1];
    
  return val;
}
void readTemp() {
  uint16_t temp;
  float temperature;
  temp = readSensor(0xE3);
  temperature = ((175.72*temp)/65536) - 46.85; //calculate temperature according to datasheet
  Serial.print("TP ");
  Serial.println(temperature);
}

void readHumid() {
  uint16_t temp;
  float humidity;
  temp = readSensor(0xE5);
  humidity = ((125*temp)/65536) - 6; //calculate humidity according to datasheet
  Serial.print("HM ");
  Serial.println(humidity);
}
void loop() {
  readCO();
  delay(500);
  readTemp();
  delay(500);
  readHumid();
  delay(500);
  }
  
