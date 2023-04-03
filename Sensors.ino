#include <Ethernet.h>

#include <ArduinoModbus.h>

int sensor = A0;

byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0xBD, 0x18}; //eth shield mac address (sticker on the bottom)
IPAddress ip(192, 168, 1, 47);  //IP for eth shield
IPAddress ipServer(86, 88, 46, 183); //Modbus server IP

//IP = 86.88.46.183:8090
//TCP auto connect on first modbus request
//c = ModbusClient(host="localhost", port=502, unit_id=1, auto_open=True)
EthernetClient ethClient;                           //Starting client

ModbusTCPClient modbusTCPClient(ethClient);




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
/*
  // Starting shield Ethernet
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  
  // Check if cable is onnected  
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // Check if server is connected
  if (!modbusTCPClient.connected()) {
    // client not connected, start the Modbus TCP client
    Serial.println("Attempting to connect to Modbus TCP server");
    
    if (!modbusTCPClient.begin(ipServer, 502)) {
      Serial.println("Modbus TCP Client failed to connect!");
    } else {
      Serial.println("Modbus TCP Client connected");
    }
  } 
  */
}


void loop() {

  int sensorCO = analogRead(A0);
  Serial.println(sensorCO);
  delay(500);
}
