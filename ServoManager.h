#ifndef __SERVO_MANAGER_H
#define __SERVO_MANAGER_H

#if defined(ARDUINO) && ARDUINO >= 100
  #include "arduino.h"
#else
  #include "WProgram.h"
#endif

#include "ESP32Servo.h"

class ServoManager
{
public:
    ServoManager() {}
    ~ServoManager() {}

    bool initialized=false;
    int  skip_counter=0;
    int  skip_counter2=0;
    
    // 16 servo objects can be created on the ESP32
    Servo   t_servo;  // THROTTLE servo
    Servo   s_servo;  // STEERING servo 
    Servo   r_servo;  // RADAR servo 
    Servo   g_servo;  // GEAR Servo

    // Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
    const int t_servoPin = 18;  // thrust
    const int s_servoPin = 19;  // steering
    const int g_servoPin = 5;   // gearing
    const int r_servoPin = 4;  // radar
    

    float     T_CENTER = 90;
    float     S_CENTER = 90;
    float     R_CENTER = 90;
    float     G_CENTER = 90;
    
    float     G_LOWPOS = 140;
    float     G_HGHPOS = 40;

    float     T_STROKE = 60;
    float     S_STROKE = 60;    
    float     R_STROKE = 40;
    
    float     T_DEADZN = 1;
    float     S_DEADZN = 1;          
    float     R_DEADZN = 0;          
    

    int       cmd_drv=0;
    int       cmd_gear=0;
    int       cmd_gear_prev=0;
    int       cmd_rsrv=0;
    
    float     cmd_spd=0.0;
    float     cmd_str=0.0;

    int       cmd_rdr=0;    
    
    float     t_pos=90.0, t_pos_f=90.0;
    float     s_pos=90.0, s_pos_f=90.0;
    float     r_pos=90.0, r_pos_f=90.0;
    
    bool      rdr_rot=true;
    float     rdr_angle=0.0;
    int       rdr_idx=-1;
    int       rdr_arr_idx[10] = {0};

    //--------------------------------------------
    //-- For Throttle Backward Moving
    //--------------------------------------------
    int       t_curr_dir=1; // 1:forward, 2:back
    int       t_prev_dir=1; // 1:forward, 2:back

public:
    void begin()
    {    
      // Allow allocation of all timers
      ESP32PWM::allocateTimer(0);
      ESP32PWM::allocateTimer(1);
      ESP32PWM::allocateTimer(2);
      ESP32PWM::allocateTimer(3);
      t_servo.setPeriodHertz(50);    // standard 50 hz servo
      s_servo.setPeriodHertz(50);    // standard 50 hz servo
      r_servo.setPeriodHertz(50);    // standard 50 hz servo
      g_servo.setPeriodHertz(50);    // standard 50 hz servo

      //myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object  
      // using default min/max of 1000us and 2000us
      // different servos may require different min/max settings
      
      t_servo.attach(t_servoPin, 544, 2400); // attaches the servo on pin 18 to the servo object
      s_servo.attach(s_servoPin, 544, 2400); // attaches the servo on pin 18 to the servo object
      r_servo.attach(r_servoPin, 544, 2400); // attaches the servo on pin 18 to the servo object
      g_servo.attach(g_servoPin, 544, 2400); // attaches the servo on pin 18 to the servo object

      delay(1000);

      t_pos_f = t_pos = T_CENTER;
      s_pos_f = s_pos = S_CENTER;
      r_pos_f = r_pos = R_CENTER;
      
      t_servo.write((int)T_CENTER);
      s_servo.write((int)S_CENTER);
      r_servo.write((int)R_CENTER);

      g_servo.write((int)G_CENTER); delay(500);
      g_servo.write((int)G_HGHPOS); delay(500);
      g_servo.write((int)G_LOWPOS); delay(500);

      initialized = true;
    }   

    int rotate_ranger()
    {
      static bool forward = false;      

      R_STROKE = 40;          
      int div = 10;

      if(rdr_rot)
      {        
        if(forward) cmd_rdr += 1; else cmd_rdr -= 1;
        if(cmd_rdr >= div)  { cmd_rdr = div;  forward = false; }
        if(cmd_rdr <= -div) { cmd_rdr = -div; forward = true;  }

        r_pos = R_CENTER + (R_STROKE / div * cmd_rdr);

        if(cmd_rsrv) r_servo.write((int)r_pos);

        //Serial.println((int)r_pos);

        rdr_angle = R_STROKE / div * cmd_rdr;          
        rdr_idx = cmd_rdr + div;        

        if(forward) {
          for(int i=0; i<10; i++) rdr_arr_idx[i] = ((rdr_idx-i) >= 0) ? rdr_idx - i : -1;                
        }
        else {
          for(int i=0; i<10; i++) rdr_arr_idx[i] = ((rdr_idx+i) <= div*2) ? rdr_idx + i : -1;                
        }

        //for(int i=0; i<5; i++) { Serial.print(rdr_arr_idx[i]); Serial.print(",");}
        //Serial.println();
        
      }
      else
      {
        skip_counter2 = 0;
        cmd_rdr = 0.0;
        r_pos = R_CENTER + (R_STROKE * cmd_rdr);
        
        rdr_angle = 0.0;
        rdr_idx = div;
      }     
      return rdr_idx; 
    }

    void update()
    {
      if(cmd_drv == 0) { cmd_spd = cmd_str = 0.0;}
     
      //-----------------------------------------------------------------------------
      //-- Throttle & Steering
      //-----------------------------------------------------------------------------
      t_pos = T_CENTER + (T_STROKE * cmd_spd);
      s_pos = S_CENTER + (S_STROKE * cmd_str);
      
      t_pos_f += (t_pos-t_pos_f) * 0.2;
      s_pos_f += (s_pos-s_pos_f) * 0.2;

            
      if(++skip_counter >= 1)
      {
        skip_counter=0;

        //if(fabs(t_pos_f - T_CENTER) <= T_DEADZN) { t_servo.write((int)T_CENTER); }  
        //else 
        {
          t_prev_dir = t_curr_dir;
          
          (t_pos_f - T_CENTER >= 0) ? t_curr_dir = 1 : t_curr_dir = 2;          
          if(t_prev_dir == 1 && t_curr_dir == 2)
          {
            Serial.print(t_prev_dir);Serial.print(",");Serial.print(t_curr_dir);Serial.println(", CATCHED...");
            t_servo.write((int)T_CENTER+5); delay(100);    //t_servo.write((int)T_CENTER); delay(10);    t_servo.write((int)T_CENTER); delay(10); 
            //t_servo.write((int)T_CENTER-30); delay(100); //t_servo.write((int)T_CENTER-30); delay(10); t_servo.write((int)T_CENTER-30); delay(10);
            t_servo.write((int)T_CENTER); delay(100);    //t_servo.write((int)T_CENTER); delay(10);    t_servo.write((int)T_CENTER); delay(10); 
          }          
          t_servo.write((int)(t_pos_f));                   
        }
        

        if(fabs(s_pos - S_CENTER) <= S_DEADZN) 
          s_servo.write((int)S_CENTER);       
        else
          s_servo.write((int)s_pos_f);       

        //Serial.print("CMD_DRV: "); Serial.print(cmd_drv); Serial.print(", CMD_SPD: "); Serial.print(cmd_spd); Serial.print(", CMD_STR: "); Serial.print(cmd_str); Serial.println();
        //Serial.print("t_pos: "); Serial.print(t_pos_f); Serial.print(", s_pos: "); Serial.print(s_pos_f); Serial.println();
      }


      //-----------------------------------------------------------------------------
      //-- Rotating Radar
      //-----------------------------------------------------------------------------
      //if(++skip_counter2 >= 2)
      //{
      //  skip_counter2 = 0;
      //  rotate_ranger();
      //}

      //-----------------------------------------------------------------------------
      //-- Gearing
      //-----------------------------------------------------------------------------
      if(cmd_gear_prev == 0 && cmd_gear == 1)       g_servo.write((int)G_HGHPOS);
      else if(cmd_gear_prev == 1 && cmd_gear == 0)  g_servo.write((int)G_LOWPOS);
      cmd_gear_prev = cmd_gear;

     
  

      /*
      //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
      //-- SERVO TEST
      //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
      for (pos = 90;  pos <= 130; pos += 2) { Serial.println(pos); t_servo.write(pos); s_servo.write(pos); delay(100); }  t_servo.write(130); s_servo.write(130); delay(1000);
      for (pos = 130; pos >= 90;  pos -= 2) { Serial.println(pos); t_servo.write(pos); s_servo.write(pos); delay(100); }  t_servo.write(90);  s_servo.write(90);  delay(1000);

      t_servo.write(50); delay(100);
      t_servo.write(90); delay(100);

      for (pos = 90;  pos >= 50;  pos -= 2) { Serial.println(pos); t_servo.write(pos); s_servo.write(pos); delay(100); }  t_servo.write(50);  s_servo.write(50);  delay(1000);
      for (pos = 50;  pos <= 90;  pos += 2) { Serial.println(pos); t_servo.write(pos); s_servo.write(pos); delay(100); }  t_servo.write(90);  s_servo.write(90);  delay(1000);
      //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
      */
    }
    
};

#endif
