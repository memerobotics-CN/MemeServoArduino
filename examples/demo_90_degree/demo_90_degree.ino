

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
  uint8_t status, in_position;

  Serial.begin(115200);           // start serial for output
  //while (!Serial);

#if (INTERFACE_TYPE == INTERFACE_TYPE_IIC)

  Wire.begin(ADDRESS_MASTER);     // join i2c bus
  Wire.onReceive(receiveEvent);   // register event
  MMS_SetProtocol(MMS_PROTOCOL_I2C, 0x01, sendDataI2C);

#else

  pinMode(RX_PIN, INPUT_PULLUP);

#if (INTERFACE_TYPE == INTERFACE_TYPE_UART)
  SerialToDevice.begin(115200);
#else
  SerialToDevice.begin(115200);
  SerialToDevice.listen();
#endif

  MMS_SetProtocol(MMS_PROTOCOL_UART, 0x01, sendDataUART, recvDataUART);  // For non-interrupt receive mode, specify receive function.

#endif

  MMS_SetCommandTimeOut(255);
  MMS_SetTimerFunction(millis, delay);

  do
  {
    delay(100);
    errno = MMS_StartServo(ADDRESS_SERVO, 1, errorHandler);
    Serial.print("MMS_StartServo returned: ");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);

  do
  {
    errno = MMS_SetGainP(ADDRESS_SERVO, 300, errorHandler);
    Serial.print("MMS_SetGainP returned: 0x");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);

  do
  {
    errno = MMS_SetGainI(ADDRESS_SERVO, 0, errorHandler);
    Serial.print("MMS_SetGainI returned: 0x");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);

  do
  {
    errno = MMS_SetGainD(ADDRESS_SERVO, 20000, errorHandler);
    Serial.print("MMS_SetGainD returned: 0x");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);
  

  status = MMS_CTRL_STATUS_NO_CONTROL;
  in_position = 0;
  while (status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position  != 1)
  {
    delay(100);
    errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, &in_position, errorHandler);

    if (errno != MMS_RESP_SUCCESS)
    {
      Serial.print("MMS_GetControlStatus returned: ");
      Serial.println(errno, HEX);
    }
  }
  
  do
  {
    delay(100);
    errno = MMS_ResetPosition(ADDRESS_SERVO, errorHandler);
    Serial.print("MMS_ResetPosition returned: ");
    Serial.println(errno, HEX);
  } while (errno != MMS_RESP_SUCCESS);
}


void loop()
{
  uint8_t errno;
  uint8_t status, in_position;
  int32_t pos;

  uint16_t accels[] = {800, 1600, 2400, 3200};
  uint16_t vels[] = {768, 1536, 2304, 3072};
  //uint16_t gainIs[] = {0, 0, 0, 0};

  uint8_t i = 0;

  while (1)
  {

    /*
    do
    {
      errno = MMS_SetGainI(ADDRESS_SERVO, gainIs[i], errorHandler);
      Serial.print("MMS_SetGainI returned: 0x");
      Serial.println(errno, HEX);
    } while (errno != MMS_RESP_SUCCESS);
    */

    do
    {
      errno = MMS_SetProfileAcceleration(ADDRESS_SERVO, accels[i], errorHandler);
      Serial.print("MMS_SetProfileAcceleration returned: 0x");
      Serial.println(errno, HEX);
    } while (errno != MMS_RESP_SUCCESS);

    do
    {
      errno = MMS_SetProfileVelocity(ADDRESS_SERVO, vels[i], errorHandler);
      Serial.print("MMS_SetProfileVelocity returned: 0x");
      Serial.println(errno, HEX);
    } while (errno != MMS_RESP_SUCCESS);

    i = (i + 1) % 0b11;


    delay(500);

    //
    // -90 degree

    errno = MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, 1024, errorHandler);
    Serial.print("MMS_ProfiledAbsolutePositionMove returned: ");
    Serial.println(errno, HEX);

    status = MMS_CTRL_STATUS_NO_CONTROL;
    in_position = 0;
    while (status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position  != 1)
    {
      delay(100);
      errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, &in_position, errorHandler);

      if (errno != MMS_RESP_SUCCESS)
      {
        Serial.print("MMS_GetControlStatus returned: ");
        Serial.println(errno, HEX);
      }
    }

    errno = MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
    Serial.print("MMS_GetAbsolutePosition returned: ");
    Serial.println(errno, HEX);

    if (errno == MMS_RESP_SUCCESS)
    {
      Serial.print("position: ");
      Serial.println(pos);
    }


    delay(500);

    //
    // 0 degree

    errno = MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, 0, errorHandler);
    Serial.print("MMS_ProfiledAbsolutePositionMove returned: ");
    Serial.println(errno, HEX);

    status = MMS_CTRL_STATUS_NO_CONTROL;
    in_position = 0;
    while (status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position  != 1)
    {
      delay(100);
      errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, &in_position, errorHandler);

      if (errno != MMS_RESP_SUCCESS)
      {
        Serial.print("MMS_GetControlStatus returned: ");
        Serial.println(errno, HEX);
      }
    }

    errno = MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
    Serial.print("MMS_GetAbsolutePosition returned: ");
    Serial.println(errno, HEX);

    if (errno == MMS_RESP_SUCCESS)
    {
      Serial.print("position: ");
      Serial.println(pos);
    }


    delay(500);

    //
    // 90 degree

    errno = MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, -1024, errorHandler);
    Serial.print("MMS_ProfiledAbsolutePositionMove returned: ");
    Serial.println(errno, HEX);

    status = MMS_CTRL_STATUS_NO_CONTROL;
    in_position = 0;
    while (status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position  != 1)
    {
      delay(100);
      errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, &in_position, errorHandler);

      if (errno != MMS_RESP_SUCCESS)
      {
        Serial.print("MMS_GetControlStatus returned: ");
        Serial.println(errno, HEX);
      }
    }

    errno = MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
    Serial.print("MMS_GetAbsolutePosition returned: ");
    Serial.println(errno, HEX);

    if (errno == MMS_RESP_SUCCESS)
    {
      Serial.print("position: ");
      Serial.println(pos);
    }


    delay(500);

    //
    // 0 degree

    errno = MMS_ProfiledAbsolutePositionMove(ADDRESS_SERVO, 0, errorHandler);
    Serial.print("MMS_ProfiledAbsolutePositionMove returned: ");
    Serial.println(errno, HEX);

    status = MMS_CTRL_STATUS_NO_CONTROL;
    in_position = 0;
    while (status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position  != 1)
    {
      delay(100);
      errno = MMS_GetControlStatus(ADDRESS_SERVO, &status, &in_position, errorHandler);

      if (errno != MMS_RESP_SUCCESS)
      {
        Serial.print("MMS_GetControlStatus returned: ");
        Serial.println(errno, HEX);
      }
    }

    errno = MMS_GetAbsolutePosition(ADDRESS_SERVO, &pos, errorHandler);
    Serial.print("MMS_GetAbsolutePosition returned: ");
    Serial.println(errno, HEX);

    if (errno == MMS_RESP_SUCCESS)
    {
      Serial.print("position: ");
      Serial.println(pos);
    }
  }
}
