#include <Wire.h>
#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS 0x77
Adafruit_BMP280 bmp;
//Installeer Adafruit Unified Sensor library en Adafruit BMP280 en Adafruit BusIO libraries vanaf library manager

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS);
  if(!status) {
    Serial.print("Wrong address, sensorID was 0x");
    Serial.println(bmp.sensorID(), 16);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}


void readCO() {
  int sensorCO = analogRead(A0);
  Serial.print("CO ");
  Serial.println(sensorCO);
}


void readTemp() {
  int temperature = bmp.readTemperature();
  Serial.print("TP ");
  Serial.println(temperature);
}

void readPress() {
  int pressure = bmp.readPressure();
  Serial.print("PR ");
  Serial.println(pressure);
}

void loop() {
  readCO();
  delay(500);
  readTemp();
  delay(500);
  readPress();
  delay(500);
  }
  
