

#include <Arduino.h>
#include <Wire.h>

#include <MemeServoAPI.h>


#define INTERFACE_TYPE_IIC       0
#define INTERFACE_TYPE_UART      1
#define INTERFACE_TYPE_SOFT_UART 2

#define INTERFACE_TYPE           INTERFACE_TYPE_UART



// -------------------------------------------------------
// user functions

#if INTERFACE_TYPE == INTERFACE_TYPE_IIC

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int nb_received)
{
//  Serial.print("received: ");

  uint8_t data;

  for (int i=0; i<nb_received; i++)
  {
    data = (uint8_t)Wire.read();
    MMS_OnData(data);

//    Serial.print(data, HEX);
//    Serial.print(" ");
  }

//  Serial.println();


  // while (Wire.available())
  // {
    // Wire.read();
  // }
}


void sendDataI2C(uint8_t addr, uint8_t *data, uint8_t size)
{
  Wire.beginTransmission(addr);  // transmit to device
  Wire.write(data, size);
  Wire.endTransmission();        // stop transmitting
}

#else

#if (INTERFACE_TYPE == INTERFACE_TYPE_SOFT_UART)
#include <SoftwareSerial.h>
#define RX_PIN 8
#define TX_PIN 9
SoftwareSerial SoftSerial(RX_PIN, TX_PIN);
#define SerialToDevice SoftSerial
#else
#define RX_PIN 0
#define TX_PIN 1
#define SerialToDevice Serial1
#endif

void recvDataUART()
{
  while (SerialToDevice.available() > 0)
  {
    uint8_t data = (uint8_t)SerialToDevice.read();
    MMS_OnData(data);
    //Serial.print(data, HEX);
    //Serial.print(" ");
  }
}

void sendDataUART(uint8_t addr, uint8_t *data, uint8_t size)
{
  while (size-- > 0)
    SerialToDevice.write(*data++);
}

#endif


void errorHandler(uint8_t node_addr, uint8_t errno)
{
  Serial.print("NODE: 0x");
  Serial.print(node_addr, HEX);
  Serial.print(", ERROR: 0x");
  Serial.println(errno, HEX);
}


void setup()
{
  uint8_t errno;

  Serial.begin(115200);           // start serial for output
  while (!Serial);

#if (INTERFACE_TYPE == INTERFACE_TYPE_IIC)

  Wire.begin(ADDRESS_MASTER);     // join i2c bus
  Wire.onReceive(receiveEvent);   // register event
  MMS_SetProtocol(MMS_PROTOCOL_I2C, sendDataI2C);

#else

  pinMode(RX_PIN, INPUT_PULLUP);

#if (INTERFACE_TYPE == INTERFACE_TYPE_UART)
  SerialToDevice.begin(115200);
#else
  SerialToDevice.begin(115200);
  SerialToDevice.listen();
#endif

  MMS_SetProtocol(MMS_PROTOCOL_UART, sendDataUART, recvDataUART);  // For non-interrupt receive mode, specify receive function.

#endif

  MMS_SetCommandTimeOut(255);
  MMS_SetTimerFunction(millis, delay);
}



void loop()
{
  uint8_t found_cnt = 0;
  uint8_t found[127];
  uint16_t ver;

  for (uint8_t address=0x02; address<0x7F; address++)
  {
    Serial.print("Trying servo: 0x");
    Serial.println(address, HEX);

    uint8_t errno = MMS_GetFirmwareVersion(address, &ver, errorHandler);

    if (errno == MMS_RESP_SUCCESS)
    {
      found[found_cnt++] = address;
      Serial.print("Found servo: 0x");
      Serial.print(address, HEX);
      Serial.print(", Firmware Version: 0x");
      Serial.println(ver, HEX);
    }
    // else
    // {
      // Serial.print("Error: 0x");
      // Serial.println(errno, HEX);
    // }
  }
  
  Serial.print("Done. Found ");
  Serial.print(found_cnt);
  Serial.print(" servo(s):");
  
  for (uint8_t i=0; i<found_cnt; i++)
  {
    Serial.print(" 0x");
    Serial.print(found[i], HEX);  
  }

  Serial.println();
  
  while(1);
}

