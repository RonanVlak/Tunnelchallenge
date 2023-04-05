#include "zmod4410.h"

zmod44xx_dev_t dev;
int8_t ret;
uint8_t stabilization_samples = 0;
uint8_t zmod44xx_status = 0;
uint8_t adc_result[30] = { 0 };
float rmox[1] = { 0 };

void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  delay(50);
  digitalWrite(8, HIGH);
  Serial.begin(9600);
  Wire.begin();
  
  delay(50);
  dev.read = arduino_i2c_read;
  dev.write = arduino_i2c_write;
  dev.i2c_addr = ZMOD4410_I2C_ADDRESS;
  
  delay(10);

  ret = zmod44xx_read_sensor_info(&dev);
  if (ret) {
    Serial.print(ret);
    Serial.println("   error:1");
  }
   
  ret = zmod44xx_init_sensor(&dev);
  if (ret) {
    Serial.print(ret);
    Serial.println("   error:2");
  }

  ret = zmod44xx_init_measurement(&dev);
  if (ret) {
    Serial.print(ret);
    Serial.println("   error:3");
  }

  ret = zmod44xx_start_measurement(&dev);
  if (ret) {
    Serial.print(ret);
    Serial.println("   error:4");
  }
}


void readCO() {
  int sensorCO = analogRead(A0);
  Serial.print("CO ");
  Serial.println(sensorCO);
}


void readCO2() {
  ret = zmod44xx_read_status(&dev, &zmod44xx_status);

  if (ret)
  { 
    Serial.print(ret);
    Serial.println("   error:5");
  }

  if (STATUS_SEQUENCER_RUNNING_MASK & zmod44xx_status)
  {
    delay(5);
  }

  ret = zmod44xx_read_adc_results(&dev, adc_result);
  if (ret)
  { 
    Serial.print(ret);
    Serial.println("   error:6");
  }

  /* start a new measurement before result calculation */
  
  ret = zmod44xx_start_measurement(&dev);
  if (ret)
  {
    Serial.print(ret);
    Serial.println("   error:7");
  }

  ret = zmod44xx_calc_rmox(&dev, adc_result, rmox);
  if (ret)
  { 
    Serial.print(ret);
    Serial.println("   error:8");
  }
  Serial.print("C2 ");
  int CO2_val = int(rmox[0]);
  Serial.println(rmox[0]);
}


void loop() {
  readCO();
  delay(1000);
  readCO2();
  delay(1000);
  }
  
