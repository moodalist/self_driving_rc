#ifndef __LIDAR_SENSOR_H
#define __LIDAR_SENSOR_H

#include <Arduino.h>     // every sketch needs this
#include <Wire.h>        // instantiate the Wire library
#include "TFLI2C.h"      // TFLuna-I2C Library v.0.2.0

class LidarSensor
{
public:
    LidarSensor()  {}
    ~LidarSensor() {}

public:
    TFLI2C      tflI2C;
    int16_t     tfPin = 32;
    int16_t     tfDist;    // distance in centimeters (Max 800 cmd)
    int16_t     tfAddr = TFL_DEF_ADR;  // use this default I2C address 

    float       currDist = 0.0;

public:
    bool  begin()
    {
      //pinMode(tfPin, INPUT);
      pinMode(tfPin, INPUT_PULLDOWN);
      Wire.begin();           // initialize Wire library
      delay(1000);

      //tflI2C.Set_Trig_Mode(tfAddr);delay(100);  
      tflI2C.Set_Cont_Mode(tfAddr);delay(100);  

      uint16_t frameRate = 250; // max 250Hz
      //uint16_t frameRate = 100; // default 100Hz
      tflI2C.Set_Frame_Rate(frameRate, tfAddr);delay(100);
      
      delay(200);  
      return true;
    }

    void  trigger_sensor() // send 'measure now' to the sensor
    {
      tflI2C.Set_Trigger(tfAddr);
    }


    bool  update_signal(int timeout=50)
    {
      static uint32_t prev_ms = millis();
      //if(millis() - prev_ms > 50)

      if(digitalRead(tfPin))
      {
        if(tflI2C.getData(tfDist, tfAddr))
        {
          if(tfDist > 800) tfDist = 800;
          if(tfDist < 0) tfDist = 0;
          //Serial.println(tfDist);

          currDist = (float)tfDist * 0.01;
          
          return true;
        }
      }
      return false;
    }
    
    /*
    bool update()
    {
      if (isNewData()) 
      {
        int cnt = getTargetCnt();

        for (int i = 0; i < cnt; i++) 
        {
          Serial.print(getRange(i));Serial.print(",");
        }          
        Serial.println();     

        return true;   
      }  
      return false;
    }
    */

};

#endif