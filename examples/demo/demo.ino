

#include <Arduino.h>
#include <Wire.h>

#include <MemeServoAPI.h>


#define INTERFACE_TYPE_IIC       0
#define INTERFACE_TYPE_UART      1
#define INTERFACE_TYPE_SOFT_UART 2

#define INTERFACE_TYPE           INTERFACE_TYPE_UART

const uint8_t ADDRESS_SERVO = 0x02;



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
  //while (!Serial);

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
 
  do
  {
    delay(100);
    errno = MMS_StartServo(ADDRESS_SERVO, 0, errorHandler);
    Serial.print("MMS_StartServo returned: ");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);
  
  do
  {
    errno = MMS_SetProfileAcceleration(ADDRESS_SERVO, 4096, errorHandler);
    Serial.print("MMS_SetProfileAcceleration returned: 0x");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);
  
  do
  {
    errno = MMS_SetProfileVelocity(ADDRESS_SERVO, 3000, errorHandler);
    Serial.print("MMS_SetProfileVelocity returned: 0x");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);
}


void loop()
{
  uint8_t errno;
  uint8_t status;

  //Serial.println("\nloop\n");
  /*
  int16_t enc;
  MMS_GetEncoderValue(ADDRESS_SERVO, &enc, errorHandler);
  delay(500);
  //*/

  /*
  MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, millis() * (double)4096.0f / 1000 / 60, errorHandler);
  delay(500);
  //*/

  //*
  int32_t pos;

  delay(100);
  errno = MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
  Serial.print("MMS_GetAbsolutePosition returned: ");
  Serial.println(errno, HEX);

  if (errno == MMS_RESP_SUCCESS)
  {
    Serial.print("position: ");
    Serial.println(pos);
  }

  for (int i=0; i<3; i++)
  {
    delay(100);
    errno = MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, i * 4096, errorHandler);
    Serial.print("MMS_ProfiledAbsolutePositionMove returned: ");
    Serial.println(errno, HEX);
  
    //*
    status = MMS_CTRL_STATUS_NO_CONTROL;
    while (status != MMS_CTRL_STATUS_POSITION_CONTROL)
    {
      delay(100);
      errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, errorHandler);
    
      if (errno != MMS_RESP_SUCCESS)
      {
        Serial.print("MMS_GetControlStatus returned: ");
        Serial.println(errno, HEX);
      }
    }
    //*/
    
    delay(100);
    errno = MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
    Serial.print("MMS_GetAbsolutePosition returned: ");
    Serial.println(errno, HEX);
  
    if (errno == MMS_RESP_SUCCESS)
    {
      Serial.print("position: ");
      Serial.println(pos);
    }
  }
  
  for (int i=3; i>0; i--)
  {
    delay(100);
    errno = MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, i * 4096, errorHandler);
    Serial.print("MMS_ProfiledAbsolutePositionMove returned: ");
    Serial.println(errno, HEX);
  
    //*
    status = MMS_CTRL_STATUS_NO_CONTROL;
    while (status != MMS_CTRL_STATUS_POSITION_CONTROL)
    {
      delay(100);
      errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, errorHandler);
    
      if (errno != MMS_RESP_SUCCESS)
      {
        Serial.print("MMS_GetControlStatus returned: ");
        Serial.println(errno, HEX);
      }
    }
    //*/
    
    delay(100);
    errno = MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
    Serial.print("MMS_GetAbsolutePosition returned: ");
    Serial.println(errno, HEX);
  
    if (errno == MMS_RESP_SUCCESS)
    {
      Serial.print("position: ");
      Serial.println(pos);
    }
  }
  
  
  //*
  delay(100);
  errno = MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, 0, errorHandler);
  Serial.print("MMS_ProfiledAbsolutePositionMove returned: ");
  Serial.println(errno, HEX);

  status = MMS_CTRL_STATUS_NO_CONTROL;
  while (status != MMS_CTRL_STATUS_POSITION_CONTROL)
  {
    delay(100);
    errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, errorHandler);
  
    if (errno != MMS_RESP_SUCCESS)
    {
      Serial.print("MMS_GetControlStatus returned: ");
      Serial.println(errno, HEX);
    }
  }
  
  uint32_t start = millis();
  for (int i=0; i<120; i++)
  {
    delay(500);
    MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, (millis() - start) * (double)4096.0f / 1000 / 60, errorHandler);
  }
  //*/
  
  //*/

  /*
  int32_t pos;
  MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
  MMS_ProfiledRelativePositionMove(ADDRESS_SERVO, 8192, errorHandler);
  delay(10000);
  MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
  MMS_ProfiledRelativePositionMove(ADDRESS_SERVO, -8192, errorHandler);
  delay(10000);
  //*/

  /*
  int32_t pos;
  MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
  MMS_RelativePositionmove(ADDRESS_SERVO, 4096, errorHandler);
  delay(3000);
  MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
  MMS_RelativePositionmove(ADDRESS_SERVO, -4096, errorHandler);
  delay(3000);
  //*/

  /*
  int32_t pos;
  MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
  MMS_VelocityMove(ADDRESS_SERVO, 1024, errorHandler);
  delay(5000);
  MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
  MMS_VelocityMove(ADDRESS_SERVO, -1024, errorHandler);
  delay(5000);
  //*/

  /*
  MMS_ProfiledVelocityMove(ADDRESS_SERVO, 1024, errorHandler);
  delay(10000);
  MMS_ProfiledVelocityMove(ADDRESS_SERVO, -1024, errorHandler);
  delay(10000);
  //*/

  //while(1);
}
