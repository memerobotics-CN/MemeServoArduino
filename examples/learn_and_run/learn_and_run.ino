

#include <Arduino.h>
#include <Wire.h>

#include <MemeServoAPI.h>


#define INTERFACE_TYPE_IIC       0
#define INTERFACE_TYPE_UART      1
#define INTERFACE_TYPE_SOFT_UART 2

#define INTERFACE_TYPE           INTERFACE_TYPE_UART

const uint8_t ADDRESS_MASTER = 0x01;
const uint8_t MAX_ID_TO_SCAN = 16;


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
  }
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


enum STATUS
{
  STATUS_INIT,
  STATUS_LEARNING,
  STATUS_RUNNING  
};

STATUS status;
uint8_t step_count;
uint8_t curr_step;
int32_t positions[256];

uint8_t servo_ids[8];
uint8_t servo_cnt;

const uint8_t keyLearn = 2;
const uint8_t keyRun = 3;

volatile bool keyLearnPressed;
volatile bool keyRunPressed;


void keyLearnPressedCallback()
{
  keyLearnPressed = true;
}


void keyRunPressedCallback()
{
  keyRunPressed = true;
}


void setup()
{
  uint8_t errno;

  Serial.begin(115200);           // start serial for output
  while (!Serial);

#if (INTERFACE_TYPE == INTERFACE_TYPE_IIC)

  Wire.begin(ADDRESS_MASTER);     // join i2c bus
  Wire.onReceive(receiveEvent);   // register event
  MMS_SetProtocol(MMS_PROTOCOL_I2C, ADDRESS_MASTER, sendDataI2C);

#else

  pinMode(RX_PIN, INPUT_PULLUP);

#if (INTERFACE_TYPE == INTERFACE_TYPE_UART)
  SerialToDevice.begin(115200);
#else
  SerialToDevice.begin(115200);
  SerialToDevice.listen();
#endif

  MMS_SetProtocol(MMS_PROTOCOL_UART, ADDRESS_MASTER, sendDataUART, recvDataUART);  // For non-interrupt receive mode, specify receive function.

#endif

  pinMode(keyLearn, INPUT_PULLUP);
  pinMode(keyRun, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(keyLearn), keyLearnPressedCallback, LOW);
  attachInterrupt(digitalPinToInterrupt(keyRun), keyRunPressedCallback, LOW);

  MMS_SetCommandTimeOut(100);
  MMS_SetTimerFunction(millis, delay);

  servo_cnt = 0;

  uint16_t firmware_ver;

  for (uint8_t i=2; i<MAX_ID_TO_SCAN; i++)
  {
    errno = MMS_GetFirmwareVersion(i, &firmware_ver, errorHandler);
    if (errno == MMS_RESP_SUCCESS)
    {
      Serial.print("Found servo: 0x");
      Serial.println(i, HEX);
      servo_ids[servo_cnt++] = i;
    }

    if (servo_cnt >= sizeof(servo_ids)) break;
  }

  Serial.print("Total servos found: "); Serial.println(servo_cnt);

  keyLearnPressed = false;
  keyRunPressed = false;

  status = STATUS_INIT;
  step_count = 0;
  curr_step = 0;
}


void loop()
{
  uint8_t errno;

  if (servo_cnt == 0)
  {
    Serial.println("No servo found.");
    while(true);  // halt
  }
  
  if (keyLearnPressed)
  {
    if (status != STATUS_LEARNING)
    {
      uint8_t i;
 
      Serial.println("Start learning mode.");
      
      status = STATUS_LEARNING;
      step_count = 0;

      // All servos got to zero
      for (i=0; i<servo_cnt; i++)
      {
        do
        {
          delay(100);
          errno = MMS_SetTorqueLimit(servo_ids[i], 65535, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_SetTorqueLimit returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS);
        
        do
        {
          delay(100);
          errno = MMS_StartServo(servo_ids[i], MMS_MODE_ZERO, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_StartServo returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS);
      }

      // Wait servos in position
      for (i=0; i<servo_cnt; i++)
      {
        uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
        uint8_t in_position = 0;

        do
        {
          delay(100);
          errno = MMS_GetControlStatus(servo_ids[i], &ctrl_status, &in_position, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_GetControlStatus returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS || ctrl_status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position != 1);
      }
      
      // Enter learnin mode
      for (i=0; i<servo_cnt; i++)
      {
        do
        {
          delay(100);
          errno = MMS_StartServo(servo_ids[i], MMS_MODE_LEARNING, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_StartServo returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS);

        do
        {
          delay(100);
          errno = MMS_SetTorqueLimit(servo_ids[i], 1000, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_SetTorqueLimit returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS);
      }
    }
    else
    {
      if (sizeof(positions) - step_count * servo_cnt >= servo_cnt)
      {
        // We have enougn space
        
        uint8_t i;

        // Save position for all servos
        for (i=0; i<servo_cnt; i++)
        {
          uint8_t servo_addr = servo_ids[i];
          int32_t pos;
          
          do
          {
            delay(100);
            errno = MMS_GetAbsolutePosition(servo_addr, &pos, errorHandler);
            if (errno != MMS_RESP_SUCCESS)
            {
              Serial.print("MMS_GetAbsolutePosition returned: 0x");
              Serial.println(errno, HEX);
            }
          } while (errno != MMS_RESP_SUCCESS);
  
          positions[step_count * servo_cnt + i] = pos;
        }

        Serial.print("Frame ");
        Serial.print(step_count);
        Serial.print(": ");
        
        for (i=0; i<servo_cnt; i++)
        {
            Serial.print(positions[step_count * servo_cnt + i]);
            Serial.print(",");
        }
        Serial.println("#");

        step_count++;
      }
    }

    keyLearnPressed = false;
  }

  if (keyRunPressed)
  {
    if (status != STATUS_RUNNING && step_count > 0)
    {
      uint8_t i;

      Serial.println("Start running mode.");
      
      status = STATUS_RUNNING;
      curr_step = 0;

      // All servos got to zero
      for (i=0; i<servo_cnt; i++)
      {
        do
        {
          delay(100);
          errno = MMS_SetTorqueLimit(servo_ids[i], 65535, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_ProfiledAbsolutePositionMove returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS);
        
        do
        {
          delay(100);
          errno = MMS_StartServo(servo_ids[i], MMS_MODE_ZERO, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_ProfiledAbsolutePositionMove returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS);
      }

      // Wait servos in position
      for (i=0; i<servo_cnt; i++)
      {
        uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
        uint8_t in_position = 0;
        
        do
        {
          delay(100);
          errno = MMS_GetControlStatus(servo_ids[i], &ctrl_status, &in_position, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
            Serial.print("MMS_ProfiledAbsolutePositionMove returned: 0x");
            Serial.println(errno, HEX);
          }
        } while (errno != MMS_RESP_SUCCESS || ctrl_status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position != 1);
      }
    }

    keyRunPressed  = false;
  }

  if (status == STATUS_RUNNING)
  {
    uint8_t i;

    // Wait servos in position
    for (i=0; i<servo_cnt; i++)
    {
      uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
      uint8_t in_position = 0;
      
      do
      {
        delay(100);
        errno = MMS_GetControlStatus(servo_ids[i], &ctrl_status, &in_position, errorHandler);
        if (errno != MMS_RESP_SUCCESS)
        {
          Serial.print("MMS_ProfiledAbsolutePositionMove returned: 0x");
          Serial.println(errno, HEX);
        }
      } while (errno != MMS_RESP_SUCCESS || ctrl_status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position != 1);
    }
    
    Serial.print("curr_step: ");
    Serial.print(curr_step);
    Serial.print("/");    
    Serial.print(step_count - 1);
    Serial.print(", Frame: ");

    for (i=0; i<servo_cnt; i++)
    {
      Serial.print(positions[curr_step * servo_cnt + i]);
      Serial.print(",");
    }
    Serial.println("#");
        
    // Move to frame
    for (i=0; i<servo_cnt; i++)
    {
      int32_t pos = positions[curr_step * servo_cnt + i];
      
      do
      {
        delay(100);
        errno = MMS_ProfiledAbsolutePositionMove(servo_ids[i], pos, errorHandler);
        if (errno != MMS_RESP_SUCCESS)
        {
          Serial.print("MMS_ProfiledAbsolutePositionMove returned: 0x");
          Serial.println(errno, HEX);
        }
      } while (errno != MMS_RESP_SUCCESS);
    }

    if (++curr_step >= step_count)
    {
      curr_step = 0;
    }
  }
}

