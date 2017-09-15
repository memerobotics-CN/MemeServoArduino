
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



const char helpMessage[] PROGMEM =
{
  "nodeId,id u8    : set node id\n"
  "nodeId,zero     : set zero position\n"
  "nodeId,start u8 : start servo\n"
  "nodeId,stop     : stop\n"
  "nodeId,ver u16  : get firmware version\n"
  "nodeId,curr u16 : get current in MA\n"
  "nodeId,volt u16 : get voltage in MV\n"
  "nodeId,adc      : get adc values\n"
  "nodeId,pos      : get position\n"
  "nodeId,enc      : get encoder value\n"
  "nodeId,status   : get control status\n"
  "nodeId,sgp u16  : set gain P\n"
  "nodeId,sgi u16  : set gain I\n"
  "nodeId,sgd u16  : set gain D\n"
  "nodeId,saw u32  : set anti wind up\n"
  "nodeId,set u16  : set error tolerance\n"
  "nodeId,spd u16  : set pwm dead zone\n"
  "nodeId,spa u16  : set profile acceleration\n"
  "nodeId,spv u16  : set profile velocity\n"
  "nodeId,vm s16   : velocity move\n"
  "nodeId,pvm s16  : profiled velocity move\n"
  "nodeId,apm s32  : absolutely move to position\n"
  "nodeId,papm s32 : profiled absolutely move to position\n"
  "nodeId,rpm s32  : relatively move to position\n"
  "nodeId,prpm s32 : profiled relatively move to position\n"
};


void printProgMemStr(const char helpMessage[] PROGMEM)
{
  // read back a char
  int len = strlen_P(helpMessage);
  int k;
  for (k = 0; k < len; k++)
  {
    char myChar =  pgm_read_byte_near(helpMessage + k);
    Serial.print(myChar);
  }
}


void errorHandler(uint8_t node_addr, uint8_t errno)
{
  Serial.print("NODE: 0x");
  Serial.print(node_addr, HEX);
  Serial.print(", ERROR: 0x");
  Serial.println(errno, HEX);
}


void setup()
{
  Serial.begin(115200);           // start serial for output
  while (!Serial);

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
}


void loop()
{
  uint8_t errno;

  if (Serial.available() > 0)
  {
    // get commands via serial
    String command = Serial.readString();

    if (command.length() > 0)
    {
      String strNodeId = command.substring(0, command.indexOf(','));
      strNodeId.trim();
      uint8_t nodeId = strNodeId.toInt();

      command = command.substring(command.indexOf(',') + 1);
      command.trim();

      Serial.println();
      Serial.print("NodeId: ");
      Serial.println(nodeId);
      Serial.print("Command: ");
      Serial.println(command);

      if (command.startsWith("id"))
      {
        errno = MMS_SetNodeID(nodeId, command.substring(2).toInt(), errorHandler);
        Serial.print("MMS_SetNodeID returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("ver"))
      {
        uint16_t ver;
        errno = MMS_GetFirmwareVersion(nodeId, &ver, errorHandler);
        Serial.print("MMS_GetFirmwareVersion returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("Firmware Version: ");
          Serial.print(ver >> 8);
          Serial.print(".");
          Serial.println(ver & 0xFF);
        }
      }
      else if (command.startsWith("zero"))
      {
        errno = MMS_SetZeroPosition(nodeId, errorHandler);
        Serial.print("MMS_SetZeroPosition returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("start"))
      {
        errno = MMS_StartServo(nodeId, command.substring(5).toInt(), errorHandler);
        Serial.print("MMS_StartServo returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("stop"))
      {
        errno = MMS_StopServo(nodeId, errorHandler);
        Serial.print("MMS_StopServo returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("adc"))
      {
        uint16_t adcs[3];
        uint8_t count = 3;
        errno = MMS_GetAnalogIn(nodeId, adcs, &count, errorHandler);
        Serial.print("MMS_GetAnalogIn returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("ADC values: ");
          Serial.print(adcs[0]);
          Serial.print(", ");
          Serial.print(adcs[1]);
          Serial.print(", ");
          Serial.println(adcs[2]);
        }
      }
      else if (command.startsWith("pos"))
      {
        int32_t pos;
        errno = MMS_GetAbsolutePosition(nodeId, &pos, errorHandler);
        Serial.print("MMS_GetAbsolutePosition returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("Position: ");
          Serial.println(pos);
        }
      }
      else if (command.startsWith("enc"))
      {
        uint16_t pos;
        errno = MMS_GetEncoderValue(nodeId, &pos, errorHandler);
        Serial.print("MMS_GetEncoderValue returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("Encoder: ");
          Serial.println(pos);
        }
      }
      else if (command.startsWith("status"))
      {
        uint8_t ctrl_status, in_position;
        errno = MMS_GetControlStatus(nodeId, &ctrl_status, &in_position, errorHandler);
        Serial.print("MMS_GetControlStatus returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("Status: "); Serial.print(ctrl_status);
          Serial.print(", In position: "); Serial.println(in_position);
        }
      }
      else if (command.startsWith("curr"))
      {
        uint16_t curr;
        errno = MMS_GetCurrent(nodeId, &curr, errorHandler);
        Serial.print("MMS_GetCurrent returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("Current: ");
          Serial.print(curr);
          Serial.println(" MA");
        }
      }
      else if (command.startsWith("volt"))
      {
        uint16_t volt;
        errno = MMS_GetVoltage(nodeId, &volt, errorHandler);
        Serial.print("MMS_GetVoltage returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("Voltage: ");
          Serial.print(volt);
          Serial.println(" MV");
        }
      }
      else if (command.startsWith("temp"))
      {
        uint16_t temp;
        errno = MMS_GetTemperature(nodeId, &temp, errorHandler);
        Serial.print("MMS_GetTemperature returned: 0x");
        Serial.println(errno, HEX);

        if (errno == MMS_RESP_SUCCESS)
        {
          Serial.print("Temperature: ");
          Serial.print(temp / 100.0f);
          Serial.println(" degrees");
        }
      }
      else if (command.startsWith("sgp"))
      {
        errno = MMS_SetGainP(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetGainP returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("sgi"))
      {
        errno = MMS_SetGainI(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetGainI returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("sgd"))
      {
        errno = MMS_SetGainD(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetGainD returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("saw"))
      {
        errno = MMS_SetAntiWindUp(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetAntiWindUp returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("set"))
      {
        errno = MMS_SetErrorTolerance(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetErrorTolerance returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("spd"))
      {
        errno = MMS_SetPwmDeadZone(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetPwmDeadZone returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("spa"))
      {
        errno = MMS_SetProfileAcceleration(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetProfileAcceleration returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("spv"))
      {
        errno = MMS_SetProfileVelocity(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_SetProfileVelocity returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("vm"))
      {
        errno = MMS_VelocityMove(nodeId, command.substring(2).toInt(), errorHandler);
        Serial.print("MMS_VelocityMove returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("pvm"))
      {
        errno = MMS_ProfiledVelocityMove(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_ProfiledVelocityMove returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("apm"))
      {
        errno = MMS_AbsolutePositionMove(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_AbsolutePositionMove returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("papm"))
      {
        errno = MMS_ProfiledAbsolutePositionMove(nodeId, command.substring(4).toInt(), errorHandler);
        Serial.print("MMS_ProfiledAbsolutePositionMove returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("rpm"))
      {
        errno = MMS_RelativePositionMove(nodeId, command.substring(3).toInt(), errorHandler);
        Serial.print("MMS_RelativePositionMove returned: 0x");
        Serial.println(errno, HEX);
      }
      else if (command.startsWith("prpm"))
      {
        errno = MMS_ProfiledRelativePositionMove(nodeId, command.substring(4).toInt(), errorHandler);
        Serial.print("MMS_ProfiledRelativePositionMove returned: 0x");
        Serial.println(errno, HEX);
      }
      else
      {
        Serial.print("Unknown Command: ");
        Serial.println(command);

        Serial.println("Available Commands: ");
        printProgMemStr(helpMessage);
      }
    }
  }

  // delay(500);
}

