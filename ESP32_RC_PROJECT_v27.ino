#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif
#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"

#endif

//#define DSS_VERSION

//-- ESP32 WiFi Setting
String WIFI_SSID = "ESP32RC";
String WIFI_PASS = "12345678";
String WIFI_MYIP = "192.168.10.1";


//-- ESP32 HW Serial1
#define HSRL_RX1 25 //white
#define HSRL_TX1 26 //red
//-- ESP32 HW Serial2
#define HSRL_RX2 16
#define HSRL_TX2 17

//-- Global Param
float G_CMD_DIST   = 2.0;
float G_CMD_THGAIN = 0.5;
float G_CMD_STGAIN = 1.5;

#ifdef DSS_VERSION
float G_CMD_MAXSPD = 0.22;
float G_CMD_MINSPD = 0.18;
#else
float G_CMD_MAXSPD = 0.20;
float G_CMD_MINSPD = 0.15;
#endif

#include <Wire.h>
#include "WebManager.h"
#include "ServoManager.h"
#include "UssSensor.h"
#include "LidarSensor.h"
//#include "RdrSensor.h"
//#include "RdrData.h"
//#include "DFRobot_URM09.h"

WebManager      web_manager;
ServoManager    servo_manager;
UssSensor       uss_sensor;
LidarSensor     lidar_sensor;
//RdrSensor       rdr_sensor;
//DFRobot_URM09   URM09;

//RDR_DATA        rdr_data[7]; // 0(-60) - 3(0) - 6(60)
//float           urm_data[31]; // 0(-60) - 15(0) - 30(60)
//float           urm_dist=0, urm_dist_f=0;; // URM09 UltraSonar Distance (in m)

//float           lidar_dist[31];   // 0(-60) - 15(0) - 30(60)
//float           lidar_angle[31];  // 0(-60) - 15(0) - 30(60)
float           lidar_dist[61];   // 0(-60) - 30(0) - 60(60)
float           lidar_angle[61];  // 0(-60) - 30(0) - 60(60)
float           lidar_angle2[61];  // 0(170) - 30(0) - 60(10)
int             lidar_indxes[10] = {0};
int             lidar_currIdx = 0;
//int             lidar_divCnt = 30;

float           _pf_average     = 0.0;
float           _pf_abs_average = 0.0;

float           _pf_angle       = 0.0;
float           _pf_angle_f     = 0.0;
float           _pf_angle_gain  = 1.2;

float           _pf_speed       = 0.0;
float           _pf_speed_f     = 0.0;
float           _pf_speed_gain  = 0.3;

float           _pf_max_obs_dst = 2.0; // over max => 1
float           _pf_min_obs_dst = 0.3; // under min => 0

unsigned long   m_Timer=0;
unsigned long   m_TimerRot=0;

float           _PI_11 = PI;
float           _PI_12 = HALF_PI;
float           _PI_14 = PI / 4.0;
float           _PI_34 = PI / 4.0 * 3.0;

void setup() 
{  
  Serial.begin(115200); while(!Serial){;} Serial.println("Serial.begin(115200)...");  
  
  Serial1.begin(115200, SERIAL_8N1, HSRL_RX1, HSRL_TX1); while(!Serial1){;}  Serial.println("Serial1.begin(115200)...");  delay(500);
  uss_sensor.begin(&Serial1); Serial.println("uss_sensor.begin()...");  
  
  lidar_sensor.begin(); Serial.println("lidar_sensor.begin()..."); 
  
/*
#ifdef OBS_USE_URM09
  delay(500); Serial.println("URM09 begin...");  
  //while(!URM09.begin()){Serial.println("I2C device number error "); delay(1000);}  delay(500);  
  //URM09.setModeRange(MEASURE_MODE_PASSIVE ,MEASURE_RANG_500); delay(500);
  //URM09.setModeRange(MEASURE_MODE_PASSIVE ,MEASURE_RANG_300);   delay(500);
  web_manager.obs_urm = true;
#else
  delay(500); Serial.println("RADAR begin...");  
  Serial2.begin(115200, SERIAL_8N1, HSRL_RX2, HSRL_TX2); while(!Serial2){;}  Serial.println("Serial2.begin(115200)...");  delay(500);
  rdr_sensor.begin(&Serial2); Serial.println("rdr_sensor.begin()...");  
  web_manager.obs_urm = false;
#endif  
*/

  web_manager.begin();        Serial.println("web_manager.begin()...");  
  servo_manager.begin();      Serial.println("servo_manager.begin()...");
    
  
  Serial.println("<<SYSTEM SETUP FINISHED>>");
  m_Timer = millis();  
  m_TimerRot = millis();
}

void loop_Timer()
{
  if(web_manager.initialized) web_manager.update();
 
  G_CMD_DIST    = web_manager.cmd_dist;     if(G_CMD_DIST <= 0.0) G_CMD_DIST = 0.0;     else if(G_CMD_DIST >= 5.0) G_CMD_DIST = 5.0;
  G_CMD_THGAIN  = web_manager.cmd_thgain;   if(G_CMD_THGAIN <= 0.0) G_CMD_THGAIN = 0.0; else if(G_CMD_THGAIN >= 5.0) G_CMD_THGAIN = 5.0;
  G_CMD_STGAIN  = web_manager.cmd_stgain;   if(G_CMD_STGAIN <= 0.0) G_CMD_STGAIN = 0.0; else if(G_CMD_STGAIN >= 5.0) G_CMD_STGAIN = 5.0;
  G_CMD_MAXSPD  = web_manager.cmd_maxspd;   if(G_CMD_MAXSPD <= 0.0) G_CMD_MAXSPD = 0.0; else if(G_CMD_MAXSPD >= 0.5) G_CMD_MAXSPD = 0.5;
  G_CMD_MINSPD  = web_manager.cmd_minspd;   if(G_CMD_MINSPD <= 0.0) G_CMD_MINSPD = 0.0; else if(G_CMD_MINSPD >= 0.5) G_CMD_MINSPD = 0.5;

  if(web_manager.cmd_reset)
  {
    uss_sensor.mData.reset();
    web_manager.cmd_reset = 0;
  }

  if(servo_manager.initialized) 
  { 
    //---------------------------------------------------------
    //-- Update Drive Mode from Web GUI
    //---------------------------------------------------------   
    servo_manager.cmd_drv = web_manager.cmd_drv;
    servo_manager.cmd_gear = web_manager.cmd_gear;
    servo_manager.cmd_rsrv = web_manager.cmd_rsrv;

    //---------------------------------------------------------
    //-- Remote Control Mode
    //---------------------------------------------------------
    if(servo_manager.cmd_drv == 1) 
    {
      servo_manager.cmd_spd = web_manager.cmd_spd;
      servo_manager.cmd_str = web_manager.cmd_str;          
      servo_manager.rdr_rot = true;
    }
    //---------------------------------------------------------
    //-- Self Control Mode
    //---------------------------------------------------------
    else if(servo_manager.cmd_drv == 2) // self control
    {
      //---------------------------------------------------------------------------------------------
      //-- Speed Control
      //---------------------------------------------------------------------------------------------
      if(uss_sensor.currValidSensorData) 
      {
        float speed_adjust = _pf_speed_f;
                
        if(web_manager.cmd_obsa) // obs avoidance mode
        {
          //servo_manager.cmd_spd = uss_sensor.mData.e_r * 0.5 + speed_adjust * 0.5;
          servo_manager.cmd_spd = uss_sensor.mData.e_r * speed_adjust;
        }  
        else 
        {
          servo_manager.cmd_spd = uss_sensor.mData.e_r;
        }          
                
        if(servo_manager.cmd_spd <= 0.0) servo_manager.cmd_spd = 0.0;      
        if(servo_manager.cmd_spd > G_CMD_MAXSPD) servo_manager.cmd_spd = G_CMD_MAXSPD;      
      }
      else servo_manager.cmd_spd = 0.0;
      
      //---------------------------------------------------------------------------------------------
      //-- Steering Control
      //---------------------------------------------------------------------------------------------
      float steer_adjust = _pf_angle_f;
      
      if(web_manager.cmd_obsa) // obs avoidance mode
      {
        if(uss_sensor.mData.e_r >= 0.01) // uss following effective
          servo_manager.cmd_str = uss_sensor.mData.e_delta + _pf_angle_f;// * _pf_speed_f;            
        else
          servo_manager.cmd_str = _pf_angle_f;// * _pf_speed_f;            
      }        
      else
      {
        if(uss_sensor.mData.e_r >= 0.01) // uss following effective
          servo_manager.cmd_str = uss_sensor.mData.e_delta;            
        else
          servo_manager.cmd_str = 0.0;
      }  
      
      if(servo_manager.cmd_str >= 1.0) servo_manager.cmd_str = 1.0;
      if(servo_manager.cmd_str <= -1.0) servo_manager.cmd_str = -1.0;

      servo_manager.rdr_rot = true;
    }
    //---------------------------------------------------------
    //-- No Control Mode
    //---------------------------------------------------------
    else
    {
      servo_manager.cmd_spd = 0.0;
      servo_manager.cmd_str = 0.0;
      servo_manager.rdr_rot = true;
    }


    //---------------------------------------------------------
    //-- Send PWM Signals to Servo Actuators
    //---------------------------------------------------------
    servo_manager.update();    

  }
}
void loop_Timer_Rot()
{
  update_ranger_timer();
}
void update_ranger_timer()
{
  lidar_dist[servo_manager.rdr_idx] = lidar_sensor.currDist;
  lidar_angle[servo_manager.rdr_idx] = servo_manager.rdr_angle;
  
  for(int i=0; i<10; i++) lidar_indxes[i] = servo_manager.rdr_arr_idx[i];
  lidar_currIdx = servo_manager.rdr_idx;

  //-- slient debug mode
  //servo_manager.rotate_ranger(lidar_divCnt);
  servo_manager.rotate_ranger();


  //--------------------------------------------------------------------------------------------------------------------------
  //-- Potential Force Calculation
  //--------------------------------------------------------------------------------------------------------------------------
  float _cd;  // distance (0.0 ~ 8.0)  
  float _ct;  // theta angle (rad) // 10~170 deg // obs[idx][1] = parseFloat( (170) - idx * 80.0 / 30.0 ) * Math.PI / 180.0;
  float _pf_ang, _pf_dst, _pf_final, _pf_abs_final;
  float _pf_sum = 0.0, _pf_abs_sum = 0.0;
  float _ang_weight = 0.0;
  int   _pf_sum_cnt = 0, _pf_abs_sum_cnt = 0;
  
  _pf_angle_gain  = web_manager.cmd_obgain;
  _pf_speed_gain   = web_manager.cmd_obgain2;
  //_pf_max_obs_dst = web_manager.cmd_obdist;

  /*
  if(uss_sensor.mData.r_f > 0.8)
    _pf_max_obs_dst = uss_sensor.mData.r_f - 0.2; // human body safety range 0.2m
  else 
    _pf_max_obs_dst = 0.8;
  */
  if(uss_sensor.mData.r_f > G_CMD_DIST + 0.2)
    _pf_max_obs_dst = uss_sensor.mData.r_f - 0.2;
  else
    _pf_max_obs_dst = G_CMD_DIST;

  int   _CNT = 21;
  for(int i=0; i<_CNT; i++)
  {
    _cd = lidar_dist[i];
    _ct = lidar_angle2[i] = (140.0 - i * 40.0 / 10.0 ) * PI / 180.0;

    //------------------------------------------------------       
    // _pf_ang : potential force from angle (-1.0 ~ 1.0)
    //------------------------------------------------------
    if      (_ct > _PI_14 && _ct < _PI_12)  _pf_ang = -sin(_ct); // right side angle (minus force)
    else if (_ct >= _PI_12 && _ct < _PI_34) _pf_ang = sin(_ct);  // left side angle (plus force)
    else                                    _pf_ang = 0.0;

    //------------------------------------------------------       
    // _pf_dst : potential force from distance (0.0 ~ 1.0)
    //------------------------------------------------------
    //_pf_max_obs_dst = 2.0;
    //_pf_dst  = (5.0 - _cd) / 5.0;     // let max_distance = 5.0
    //_pf_dst  = (2.0 - _cd) / 2.0;     // let max_distance = 2.0
    //_pf_max_obs_dst = 5.0;

    if(_cd > _pf_max_obs_dst)
    {
        _pf_dst = 0.0; // no obs
    }
    else if(_cd > _pf_min_obs_dst) // > 0.3 // sensor operating range : 0.2~8.0 (m)
    {
        _pf_dst  = (_pf_max_obs_dst - _cd) / (_pf_max_obs_dst - _pf_min_obs_dst); // _cd 

        if(_pf_dst < 0.02) _pf_dst = 0.0;       // negative limit check (no obs)
        else if(_pf_dst > 1.0)  _pf_dst = 1.0;  // positive limit check (full obs)
    }
    else
    {
        _pf_dst = 1.0; // full obs
    }
    

    //------------------------------------------------------       
    // _pf_final : final potential force (-1.0 ~ 1.0)
    // _pf_abs_final : final abs force (with negative distance factor)
    //------------------------------------------------------    
    _pf_final = _pf_ang * _pf_dst;
    _pf_abs_final = fabs(_pf_ang) * _pf_dst;
        
    //------------------------------------------------------       
    // If Considerable Obstacles, then process...
    //------------------------------------------------------    
    if(fabs(_pf_final) > 0.0) 
    {
      //Serial.print(_pf_abs_final); Serial.print(",");
      if(i != 10){
        _pf_sum += _pf_final;
        _pf_sum_cnt++;
      }

      if(i > 6 && i < 14)
      {
        _pf_abs_sum += _pf_abs_final;
        _pf_abs_sum_cnt++;
      } 
      /*
      _ang_weight = (1.0 - fabs( (i-10.0) / 10.0 ));
      {
        _pf_abs_sum += (_pf_abs_final * _ang_weight);
        _pf_abs_sum_cnt++;
      } 
      */     
    }
  }
  //Serial.println();
  
  //------------------------------------------------------       
  // _pf_avg : average of potential force (-1.0~1.0)
  //------------------------------------------------------
  _pf_average = (_pf_sum_cnt > 0) ? _pf_sum / _pf_sum_cnt : 0.0;
  _pf_abs_average = (_pf_abs_sum_cnt > 0) ? _pf_abs_sum / _pf_abs_sum_cnt : 0.0;

  _pf_angle    = _pf_average * _pf_angle_gain;
  _pf_angle_f  += (_pf_angle-_pf_angle_f) * 0.05;

  _pf_speed    = (1.0 - _pf_abs_average * _pf_speed_gain);  if(_pf_speed >= 1.0) _pf_speed = 1.0; else if(_pf_speed <= 0.0) _pf_speed = 0.0;
  _pf_speed_f  += (_pf_speed - _pf_speed_f) * 0.05;

  /*
  if(fabs(_pf_angle_f) > 0.0) {
    Serial.print("_pf_angle_f: ");
    Serial.println(_pf_angle_f);
  }
  */
  
  /*
  Serial.print("<");
  Serial.print(_pf_abs_average);Serial.print(",");Serial.print(_pf_speed_gain);Serial.print(",");
  Serial.print(_pf_speed_f);
  Serial.println(">");
  */

  Serial.print("<_pf_angle_f: "); Serial.print(_pf_angle_f); 
  Serial.print(", _pf_speed_f: "); Serial.print(_pf_speed_f); Serial.println(">");
  




  //Serial.print(servo_manager.rdr_idx);
  //Serial.print(",");
  //Serial.println(servo_manager.rdr_angle);

  //Serial.println(lidar_sensor.currDist);
}
void update_ranger_realtime()
{
  if(lidar_sensor.update_signal())
  {
    lidar_dist[servo_manager.rdr_idx] = lidar_sensor.currDist;
    lidar_angle[servo_manager.rdr_idx] = servo_manager.rdr_angle;

    //Serial.print("lidar: ");Serial.println(lidar_sensor.currDist);
  }
}
/*
void update_ranger()
{
  //static bool sentRot = false;
  static bool gotSignal = false;

  if(gotSignal) 
  { 
    gotSignal = false; 
    servo_manager.rotate_ranger(); delay(1);
    lidar_sensor.trigger_sensor();
    //sentSignal = true;

    Serial.print("servo: ");Serial.println(servo_manager.rdr_idx);
  }

  if(!gotSignal && lidar_sensor.wait_signal()) 
  {
    gotSignal = true; 
    //sentSignal = false;

    lidar_dist[servo_manager.rdr_idx] = lidar_sensor.currDist;
    lidar_angle[servo_manager.rdr_idx] = servo_manager.rdr_angle;

    Serial.print("lidar: ");Serial.println(lidar_sensor.currDist);
  }
}
*/

void loop_realTime()
{  
  //---------------------------------------------------------------------------------
  //-- EBUS UltraSonic Sensor
  //---------------------------------------------------------------------------------
  uss_sensor.receive(); 
  uss_sensor.update();   

  //---------------------------------------------------------------------------------
  //-- Lidar Sensor and Servo Ratation
  //---------------------------------------------------------------------------------
  //update_ranger();
  update_ranger_realtime();

/*
#ifdef OBS_USE_URM09
  //---------------------------------------------------------------------------------
  //-- URM UltraSonic Sensor
  //---------------------------------------------------------------------------------
  static unsigned long urm_time = millis();
  if(millis() - urm_time > 25) // dt from measurement() to getDistrance() == about 23ms
  {
    urm_time = millis();

    urm_dist = (float)URM09.getDistance()*0.01;
    servo_manager.rotate_ranger(); delay(1);   
        
    Serial.print(servo_manager.rdr_idx);
    Serial.print(": ");
    Serial.println(urm_dist,4);    
    urm_data[servo_manager.rdr_idx] = urm_dist;
    
    URM09.measurement();    
  }
  
#else
  //---------------------------------------------------------------------------------
  //-- mmWave Radar Sensor
  //---------------------------------------------------------------------------------
  if(rdr_sensor.fetchNewData())
  {
    //servo_manager.rotate_ranger(3);    
  }  

  //---------------------------------------------------------------------------------
  //-- Update Radar Data
  //---------------------------------------------------------------------------------
  static uint32_t mms1 = millis();

  //-- debug
  servo_manager.rdr_idx = 3;
  
  if(rdr_sensor.isNewData()) 
  {
    Serial.print("[");
    Serial.print(millis() - mms1);
    Serial.print("]  ");
    mms1 = millis();

    int idx = servo_manager.rdr_idx;
    if(idx >= 0)
    {
      int cnt = rdr_sensor.getTargetCnt(); if(cnt > 4) cnt = 4;

      rdr_data[idx].angle = servo_manager.rdr_angle;
      rdr_data[idx].count = cnt;

      Serial.printf("[%d] %d %d ", idx, (int)servo_manager.rdr_angle, cnt);
      
      for(int i=0; i<4; i++) { rdr_data[idx].range[i] = rdr_data[idx].strength[i] = -1; }
      for(int i=0; i<cnt; i++) {
        rdr_data[idx].range[i]    = rdr_sensor.getRange(i);
        rdr_data[idx].strength[i] = rdr_sensor.getStrength(i);

        Serial.printf("(%d, %d),", rdr_sensor.getRange(i), rdr_sensor.getStrength(i));
      }

      Serial.println();
    }    
  }
#endif
*/
}

void loop() 
{
  //---------------------------------------------------------------------------------
  //-- 50Hz Timer (Logic Processing)
  //---------------------------------------------------------------------------------
  if(millis() - m_Timer >= 20)    { m_Timer = millis();     loop_Timer();  } 
  if(millis() - m_TimerRot >= 20) { m_TimerRot = millis();  loop_Timer_Rot(); }

  //---------------------------------------------------------------------------------
  //-- Realtime (Sensor Processing)
  //---------------------------------------------------------------------------------
  loop_realTime();                                
}
