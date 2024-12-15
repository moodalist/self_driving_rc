#ifndef __USS_DATA_H
#define __USS_DATA_H


#define EBS_DX       (0.131) // distance between center & ebus1 (=ebus2), ebus1-ebus2 : 0.262
//#define EBS_LPFA1    (0.2)  // lowpass filter alpha for 'throttle'
//#define EBS_LPFA2    (0.2)  // lowpass filter alpha for 'steering'
#define EBS_LPFA1    (0.4)  // lowpass filter alpha for 'throttle'
#define EBS_LPFA2    (0.4)  // lowpass filter alpha for 'steering'

extern float G_CMD_DIST;
extern float G_CMD_THGAIN;
extern float G_CMD_STGAIN;
extern float G_CMD_MAXSPD;
extern float G_CMD_MINSPD;

//----------------------------------------------------------------------------------------------
//-- UltraSonic (EBUS) Data Structure
//----------------------------------------------------------------------------------------------
struct USS_DATA
{
  float   us_d1=0,    us_d2=0; // Two Radius of USS Sensor (meter) Filtered
  float   us_d1_raw=0,  us_d2_raw=0; // Two Radius of USS Sensor (meter) RawData

  float   r=0,    delta=0;       // OUTPUT
  float   r_f=0,  delta_f=0;   // low pass filtered OUTPUT
  float   e_r=0,  e_delta=0;   // error_r: 0 ~ 1.0, e_delta: -1.0 ~ 1.0 

  //--------------------------------------------------------------------------------------------
  // << Find the intersections (two points) of two circles >>
  // based on the math here:
  // http://math.stackexchange.com/a/1367732
  //
  // x1,y1 is the center of the first circle, with radius r1
  // x2,y2 is the center of the second ricle, with radius r2
  //--------------------------------------------------------------------------------------------
  struct CIRCLE_PARAM
  {
    float x1, y1, r1; // input of first circle
    float x2, y2, r2; // input of second circle
    float ix1, iy1;   // output of first point
    float ix2, iy2;   // output of second point

    void reset()
    {
      x1 = -EBS_DX; y1 = 0; r1 = 1.0;
      x2 = EBS_DX;  y2 = 0; r2 = 1.0;
      ix1 = iy1 = ix2 = iy2 = 0.0;
    }
  };

  bool intersectTwoCircles(CIRCLE_PARAM& pr) {
    float centerdx = pr.x1 - pr.x2;
    float centerdy = pr.y1 - pr.y2;
    float R = sqrt(centerdx * centerdx + centerdy * centerdy);
    if (!(fabs(pr.r1 - pr.r2) <= R && R <= pr.r1 + pr.r2)) { // no intersection
      return false;
    }
    // intersection(s) should exist

    float R2 = R*R;
    float R4 = R2*R2;
    float a = (pr.r1*pr.r1 - pr.r2*pr.r2) / (2 * R2);
    float r2r2 = (pr.r1*pr.r1 - pr.r2*pr.r2);
    float c = sqrt(2 * (pr.r1*pr.r1 + pr.r2*pr.r2) / R2 - (r2r2 * r2r2) / R4 - 1);

    float fx = (pr.x1+pr.x2) / 2 + a * (pr.x2 - pr.x1);
    float gx = c * (pr.y2 - pr.y1) / 2;
    pr.ix1 = fx + gx;
    pr.ix2 = fx - gx;

    float fy = (pr.y1+pr.y2) / 2 + a * (pr.y2 - pr.y1);
    float gy = c * (pr.x1 - pr.x2) / 2;
    pr.iy1 = fy + gy;
    pr.iy2 = fy - gy;

    // note if gy == 0 and gx == 0 then the circles are tangent and there is only one solution
    // but that one solution will just be duplicated as the code is currently written
    return true;
  }

  CIRCLE_PARAM param;             // INTERNAL
  float   theta, theta1, theta2;  // INTERNAL
  float   thx, thy, thx_f, thy_f, theta_f; // low pass filtered theta
  
  void reset()
  {
    us_d1 = us_d2 = 0.0;
    us_d1_raw = us_d2_raw = 0.0;

    r = delta = 0.0;
    r_f = delta_f = 0.0;
    e_r = e_delta = 0.0;

    theta = theta1 = theta2 = 0.0;
    thx = thy = thx_f = thy_f = theta_f = 0.0;

  }
  bool calculate()
  {
    if(us_d1 >= 10.0) us_d1 = 10.0; else if(us_d1 <= 0.0) us_d1 = 0.0;
    if(us_d2 >= 10.0) us_d2 = 10.0; else if(us_d2 <= 0.0) us_d2 = 0.0;

    if(us_d1 > 0.05 && us_d2 > 0.05 && (us_d1+us_d2) > (2*EBS_DX))
    {
      param.reset();
      param.r1 = us_d1;
      param.r2 = us_d2;

      if(intersectTwoCircles(param))
      {
        theta1 = atan2(param.iy1, param.ix1);
        theta2 = atan2(param.iy2, param.ix2);
        if(param.iy1 >=0.0) 
        {
          theta = theta1; 
          r = sqrt(param.ix1*param.ix1 + param.iy1*param.iy1);
        }
        else 
        {
          theta = theta2;
          r = sqrt(param.ix2*param.ix2 + param.iy2*param.iy2);
        }
        //if(theta1 >=0.0 && theta1 <= M_PI) theta = theta1; else theta = theta2;
        //Serial.print(param.iy1); Serial.print(",");Serial.print(param.iy2); Serial.print(",");Serial.print(theta * RAD_TO_DEG); Serial.println(",");
        
        if(r > 10.0) r = 10.0;
        r_f += (r - r_f) * EBS_LPFA1;
        //if(r_f > 6) r_f = r; // overshooting error handling

        
        thx = cos(theta); thx_f += (thx - thx_f) * EBS_LPFA2;
        thy = sin(theta); thy_f += (thy - thy_f) * EBS_LPFA2;        
        theta_f = atan2(thy_f, thx_f);
        delta = delta_f = theta_f - M_PI_2;        
        
        //delta = theta - M_PI_2;
        //delta_f = (delta - delta_f) * EBS_LPFA2;

        float H_C_DIST = G_CMD_DIST;
        float d_err = (r_f - H_C_DIST); // 0.0~1.0        
        
        if(d_err <= 0.0) d_err = 0.0;
        else if(d_err >= 5.0) d_err = 1.0;
        else d_err = d_err / 5.0;

        //if(e_r <= 0.0) e_r = 0.0; else if(e_r >= 0.2) e_r = 0.2 + e_r*e_r*0.05;
        
        if(d_err > 0.0)
        //  e_r = 0.2 + G_CMD_THGAIN * sin(d_err * M_PI_2);
          e_r = G_CMD_MINSPD + G_CMD_THGAIN * d_err;
        else
          e_r = 0.0;

        //Serial.print(d_err);Serial.print(",");Serial.print(e_r);Serial.println(",");
       

        e_delta = delta_f / M_PI_2 * G_CMD_STGAIN;
        if(e_delta <= -1.0) e_delta = -1.0; else if(e_delta >= 1.0) e_delta = 1.0;

        //Serial.print(r_f);Serial.print(",");Serial.print(e_r);Serial.print(",");
        //Serial.print(delta_f);Serial.print(",");Serial.println(e_delta);

        return true;
      }
    }
    
    /*
    else
    {
      r_f = thx_f = thy_f = 0.0;
    }  
    e_r = 0.0;
    e_delta = 0.0;
    */
    //reset();
    return false;
  }

  float CLAMP_RADIANS(float r)
  {
    float d = (r > 0.0) ? 2*M_PI : -2*M_PI;
    r = fmod(r, d);
    return (r > M_PI || r < -M_PI) ? r - d : r;
  }  
};

#endif