#ifndef __USS_SENSOR_H
#define __USS_SENSOR_H

#define EBS_BUFF_SIZE  (1024)
#define EBS_STX        (0x23) // '#'
#define EBS_ETX1       (0x0D) // CR
#define EBS_ETX2       (0x0A) // LF


#include "UssData.h"

class  UssSensor
{
public:
    UssSensor()     {mData.reset();}
    ~UssSensor()    {}

public:    
    USS_DATA        mData;
    HardwareSerial* mSerial=NULL;    

    int             currValidSensorData = 0;

    bool            NewData = false;            
    char            item_buff[16][16];
    byte            item_count = 0;

    void begin(HardwareSerial* pSerial)
    {
      mSerial = pSerial;
      mData.reset();
    }
    void receive()
    {
      if(!mSerial) return;


      char  rc;
      while(mSerial->available() && NewData == false)
      {
        rc = mSerial->read();
        //Serial.print(rc);

        if(RecvInProgress == true) // PAYLOAD + ETX
        {
          if(rc == EBS_ETX1) // ETX1
          {
            RecvedETX1 = true;
          }
          else if(rc == EBS_ETX2)
          {
            if(RecvedETX1) // ETX2
            {
              recv_buff[ndx] = '\0';
              strcpy(temp_buff, recv_buff);
              RecvInProgress = false;
              ndx = 0;
              NewData = true;
              RecvedETX1 = false;
            }            
            else // Abnormal Condition
            {
              RecvInProgress = false;
            }
          }
          else // PAYLOAD
          {
            recv_buff[ndx] = rc;
            ndx++;
            if(ndx >= EBS_BUFF_SIZE) ndx = EBS_BUFF_SIZE-1;
          }          
        }
        else if(rc == EBS_STX)// STX
        {
          RecvInProgress = true;
        }
      }
    }
    void update()
    {
      static bool Got_d1=false, Got_d2=false;

      if(NewData)
      {
        parseData();
        NewData = false;
        
        //Serial.print(item_buff[0]); Serial.print(","); Serial.println(item_buff[1]);

#ifdef DSS_VERSION
        if(item_buff[0][2] == '1')      { mData.us_d1 = atof(item_buff[1]); Got_d1 = true;}
        else if(item_buff[0][2] == '2') { mData.us_d2 = atof(item_buff[1]); Got_d2 = true;}
#else
        if(item_buff[0][2] == '2')      { mData.us_d1 = atof(item_buff[1]); Got_d1 = true;}
        else if(item_buff[0][2] == '1') { mData.us_d2 = atof(item_buff[1]); Got_d2 = true;}
#endif
/*
#ifdef DSS_VERSION
        if(item_buff[0][2] == '1')      { mData.us_d1_raw = atof(item_buff[1]); mData.us_d1 += (mData.us_d1_raw-mData.us_d1) * 0.05; Got_d1 = true;}
        else if(item_buff[0][2] == '2') { mData.us_d2_raw = atof(item_buff[1]); mData.us_d2 += (mData.us_d2_raw-mData.us_d2) * 0.05; Got_d2 = true;}
#else
        if(item_buff[0][2] == '2')      { mData.us_d1_raw = atof(item_buff[1]); mData.us_d1 += (mData.us_d1_raw-mData.us_d1) * 0.05; Got_d1 = true;}
        else if(item_buff[0][2] == '1') { mData.us_d2_raw = atof(item_buff[1]); mData.us_d2 += (mData.us_d2_raw-mData.us_d2) * 0.05; Got_d2 = true;}
#endif
*/
        //Serial.print("d1: "); Serial.print(mData.us_d1, 4); Serial.print(", d2: "); Serial.println(mData.us_d2, 4); 
        //Serial.print(mData.us_d1, 4); Serial.print(","); Serial.println(mData.us_d2, 4); 

        if(Got_d1 && Got_d2)
        {
          Got_d1 = Got_d2 = false;

          if(mData.calculate())
          {
            if(mData.e_r > 0.02) // following (driving) mode
              currValidSensorData = 2;
            else // stopping mode
              currValidSensorData = 1;
            /*
            Serial.print("r: "); Serial.print(mData.r, 4); 
            Serial.print(", delta: "); Serial.print(mData.delta*RAD_TO_DEG); 
            Serial.print(", e_r: "); Serial.print(mData.e_r, 4); 
            Serial.print(", e_delta: "); Serial.print(mData.e_delta*RAD_TO_DEG); 
            Serial.println();
            */
          }  
          else
          {
            currValidSensorData = 0;
          }

          //Serial.print(mData.e_r); Serial.print(","); Serial.println(currValidSensorData);
        }         
      }
    }

    void parseData()
    {
      item_count = 0;
      char* strToken = strtok(temp_buff, ",");      
      while(strToken != NULL)
      {
        strcpy(item_buff[item_count], strToken);
        strToken = strtok(NULL, ",");
        item_count++;
      }
    }
    
protected:   
    bool   RecvInProgress = false;
    bool   RecvedETX1 = false;
    byte   ndx = 0;
    char   recv_buff[EBS_BUFF_SIZE] = {0};
    char   temp_buff[EBS_BUFF_SIZE] = {0};
    
};

#endif