#include "arduino.h"
#include "WebManager.h"
#include <esp_wifi.h>

AsyncWebServer  web_server(80);
AsyncWebSocket  web_socket("/ws");

#include "ServoManager.h"
#include "UssSensor.h"

extern String           WIFI_SSID;
extern String           WIFI_PASS;
extern String           WIFI_MYIP;
extern WebManager       web_manager;
extern ServoManager     servo_manager;
extern UssSensor        uss_sensor;

extern float           lidar_dist[];
extern float           lidar_angle[];
extern int             lidar_indxes[];
extern int             lidar_currIdx;
extern int             lidar_divCnt;

extern float           _pf_average;
extern float           _pf_abs_average;

extern float           _pf_angle;
extern float           _pf_angle_f;
extern float           _pf_angle_gain;

extern float           _pf_speed;
extern float           _pf_speed_f;
extern float           _pf_speed_gain;

extern float           _pf_max_obs_dst;


int     vtm_DRIVE_ON=0, vtm_GEAR_MODE=0;
float   vtm_SPEED_KMH=0.0,vtm_STEER_DEG=0.0;

/*
//-- Global Param
float G_CMD_DIST = 1.8;
float G_CMD_THGAIN = 0.3;
float G_CMD_STGAIN = 1.5;

#ifdef DSS_VERSION
float G_CMD_MAXSPD = 0.20;
float G_CMD_MINSPD = 0.18;
#else
float G_CMD_MAXSPD = 0.18;
float G_CMD_MINSPD = 0.16;
#endif

#ifdef DSS_VERSION  
  var cmd_maxspd=22, cmd_minspd=18;
#else
  var cmd_maxspd=18, cmd_minspd=16;
#endif

//-- Global Param
float G_CMD_DIST = 1.5;
float G_CMD_THGAIN = 0.5;
float G_CMD_STGAIN = 1.5;

#ifdef DSS_VERSION
float G_CMD_MAXSPD = 0.20;
float G_CMD_MINSPD = 0.18;
#else
float G_CMD_MAXSPD = 0.20;
float G_CMD_MINSPD = 0.15;
*/

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><meta http-equiv="Content-Type" content="text/html; charset=utf-8"><title>ESP32_RCCAR_VIEWER</title>
<script>
  var lastTimer=0,lastGui=0,skip_cnt_Timer=0,skip_cnt_Gui=0;
  var tm_TIMER_HZ,tm_GUI_HZ,tm_DRIVE_ON,tm_SPEED_KMH,tm_STEER_DEG,tm_GEAR_MODE;
  var tm_cmd_dist,tm_cmd_thgain,tm_cmd_stgain,tm_cmd_maxspd,tm_cmd_minspd,tm_cmd_obgain,tm_cmd_obgain2, tm_cmd_obdist;
  var vtm_TIMER_HZ=0,vtm_GUI_HZ=0,vtm_DRIVE_ON=0,vtm_SPEED_KMH=0.0,vtm_STEER_DEG=0.0,vtm_GEAR_MODE=0;
  var cmd_drv=0,cmd_gear=0,cmd_rsrv=0,cmd_obsa=0,cmd_spd=0.0,cmd_str=0.0,cmd_reset=0;  
  var sts_acc=0,sts_dec=0,sts_left=0,sts_right=0;
  var canvas, ctx;

  var cmd_dist=200, cmd_thgain=50, cmd_stgain=150;

  //var cmd_maxspd=22, cmd_minspd=18;
  var cmd_maxspd=20, cmd_minspd=15;

  var cmd_obgain=120, cmd_obgain2=50, cmd_obdist=200;

  //var _pf_average=0.0, 
  var _pf_angle_f=0.0, _pf_angle_gain=1.0, _pf_speed_f=0.0, _pf_speed_gain=1.0, _pf_max_obs_dst=2.0;
 
  const OBS_URM = true;

  const CV_DX = 960, CV_CX = 480;
  //const CV_DY = 700, CV_CY = 640;
  const CV_DY = 800, CV_CY = 740;
  //const CV_RATIO = 200.0;
  const CV_RATIO = 160.0;
  
  //var CNT = 61;
  var CNT = 21;
  var obs = new Array(CNT);
  for(let i=0; i<CNT; i++) 
  {
    obs[i] = new Array(6); //[0]sensor range(meter), [1]angle, [2]range in px, [3]dx, [4]dy, [5]radius
    obs[i][0] = obs[i][1] = 0.0;    
    obs[i][2] = obs[i][3] = obs[i][4] = obs[i][5] = 0;
  }

  var trg = new Array(5); //visible,distance,angle,dx,dy
  var dist = new Array(3);
  var cross = new Array(5);

  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  window.addEventListener('load', onLoad);

  function onOpen(event)  { console.log('Connection opened'); }
  function onClose(event) { console.log('Connection closed'); setTimeout(initWebSocket, 2000); }
  function onLoad(event)  { initWebSocket(); }
  function initWebSocket() 
  {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage; // <-- add this line
  }
  function sendClientData()
  {
    if(tm_cmd_dist)   cmd_dist    = tm_cmd_dist.value;
    if(tm_cmd_thgain) cmd_thgain  = tm_cmd_thgain.value;
    if(tm_cmd_stgain) cmd_stgain  = tm_cmd_stgain.value;
    if(tm_cmd_maxspd) cmd_maxspd  = tm_cmd_maxspd.value;
    if(tm_cmd_minspd) cmd_minspd  = tm_cmd_minspd.value;
    if(tm_cmd_obgain) cmd_obgain  = tm_cmd_obgain.value;
    if(tm_cmd_obgain2) cmd_obgain2  = tm_cmd_obgain2.value;
    //if(tm_cmd_obdist) cmd_obdist  = tm_cmd_obdist.value;

    //var cmd_string = "cmd," + cmd_drv + "," + cmd_spd + "," + cmd_str + "," + cmd_gear + "," + cmd_dist + "," + cmd_thgain + "," + cmd_stgain + "," + cmd_maxspd + "," + cmd_minspd + "," + cmd_obgain + "," + cmd_obdist + ",";
    var cmd_string = "cmd," + cmd_drv + "," + cmd_spd + "," + cmd_str + "," + cmd_gear + "," + cmd_dist + "," + cmd_thgain + "," + cmd_stgain + "," + cmd_maxspd + "," + cmd_minspd + "," + cmd_obgain + "," + cmd_obgain2 + "," + cmd_rsrv + "," + cmd_obsa + "," + cmd_reset + ",";
    websocket.send(cmd_string);
    
    cmd_reset = 0;
  }
  function onMessage(event) 
  {
    var resp_string = event.data;
    var arr_resp = resp_string.split(",");

    vtm_DRIVE_ON=parseInt(arr_resp[0]);vtm_SPEED_KMH=parseFloat(arr_resp[1]);vtm_STEER_DEG=parseFloat(arr_resp[2]);vtm_GEAR_MODE=parseInt(arr_resp[3]);
    trg[0]=parseInt(arr_resp[4]);
    trg[1]=parseInt(arr_resp[5]) * 0.001;
    trg[2]=parseInt(arr_resp[6]) * 0.01;

    var ang1=trg[2]*Math.PI/180.0;
    trg[3]=parseInt(trg[1]*CV_RATIO*Math.cos(ang1));
    trg[4]=parseInt(trg[1]*CV_RATIO*Math.sin(ang1));

    dist[0] = parseInt(arr_resp[7]) * 0.001 * CV_RATIO;
    dist[1] = parseInt(arr_resp[8]) * 0.001 * CV_RATIO;
    dist[2] = parseInt(arr_resp[9]) * 0.001 * CV_RATIO;

    cross[0]=parseInt(arr_resp[10]);
    cross[1]=parseInt(arr_resp[11]) * 0.001;
    cross[2]=parseInt(arr_resp[12]) * 0.01;

    var ang2=cross[2]*Math.PI/180.0;
    cross[3]=parseInt(cross[1]*CV_RATIO*Math.cos(ang2));
    cross[4]=parseInt(cross[1]*CV_RATIO*Math.sin(ang2));
    
    //_pf_average     = parseFloat(arr_resp[13]);
    _pf_angle_f       = parseFloat(arr_resp[13]);
    _pf_angle_gain    = parseFloat(arr_resp[14]);
    _pf_speed_f       = parseFloat(arr_resp[15]);
    _pf_speed_gain    = parseFloat(arr_resp[16]);
    _pf_max_obs_dst   = parseFloat(arr_resp[17]);

        
    for(let i=0; i<21; i++)
    {
      obs[i][0] = arr_resp[18 + i];
      obs[i][1] = parseFloat( (140) - i * 40.0 / 10.0 ) * Math.PI / 180.0;
      obs[i][2] = parseInt(parseFloat(obs[i][0])*CV_RATIO); 			      // range in px
      obs[i][3] = parseInt(parseFloat(obs[i][2])*Math.cos(obs[i][1])); // dpx
      obs[i][4] = parseInt(parseFloat(obs[i][2])*Math.sin(obs[i][1])); // dpy
      obs[i][5] = 50;///obs[i][2] * 0.3; 	// radius        
    }
    
  
    update_gui();
    update_gui_hz();
    update_canvas();
  }
  
  function body_onload_handler()
  {
    tm_TIMER_HZ = document.getElementById("tm_TIMER_HZ");tm_GUI_HZ = document.getElementById("tm_GUI_HZ");tm_DRIVE_ON = document.getElementById("tm_DRIVE_ON");tm_SPEED_KMH = document.getElementById("tm_SPEED_KMH");tm_STEER_DEG = document.getElementById("tm_STEER_DEG");tm_GEAR_MODE = document.getElementById("tm_GEAR_MODE");
    tm_cmd_dist = document.getElementById("tm_cmd_dist");tm_cmd_thgain = document.getElementById("tm_cmd_thgain");tm_cmd_stgain = document.getElementById("tm_cmd_stgain");tm_cmd_maxspd = document.getElementById("tm_cmd_maxspd");tm_cmd_minspd = document.getElementById("tm_cmd_minspd");tm_cmd_obgain = document.getElementById("tm_cmd_obgain");tm_cmd_obgain2 = document.getElementById("tm_cmd_obgain2");tm_cmd_obdist = document.getElementById("tm_cmd_obdist");

    if(tm_cmd_dist) tm_cmd_dist.value = cmd_dist;
    if(tm_cmd_thgain) tm_cmd_thgain.value = cmd_thgain;
    if(tm_cmd_stgain) tm_cmd_stgain.value = cmd_stgain;
    if(tm_cmd_maxspd) tm_cmd_maxspd.value = cmd_maxspd;
    if(tm_cmd_minspd) tm_cmd_minspd.value = cmd_minspd;
    if(tm_cmd_obgain) tm_cmd_obgain.value = cmd_obgain;
    if(tm_cmd_obgain2) tm_cmd_obgain2.value = cmd_obgain2;
    //if(tm_cmd_obdist) tm_cmd_obdist.value = cmd_obdist;

    setInterval(timer_handler,50);
    update_canvas();
  }  
  function timer_handler()
  {
    drv_update();
    sendClientData();
    update_timer_hz();
  }
  
  function update_timer_hz()
  {
    if(++skip_cnt_Timer>=10)
    {
      skip_cnt_Timer=0;
      var timer_now=Date.now(); var timer_dt=timer_now-lastTimer; lastTimer=timer_now;
      vtm_TIMER_HZ=(1000.0/parseFloat(timer_dt)*10.0).toFixed(1); 
      if(tm_TIMER_HZ) tm_TIMER_HZ.value=vtm_TIMER_HZ;
    }
  }
  
  function update_gui()
  {
    if(tm_DRIVE_ON) {tm_DRIVE_ON.value=vtm_DRIVE_ON;}
    if(tm_SPEED_KMH){tm_SPEED_KMH.value=vtm_SPEED_KMH.toFixed(3);}
    if(tm_STEER_DEG){tm_STEER_DEG.value=vtm_STEER_DEG.toFixed(3);}    
    if(tm_GEAR_MODE){tm_GEAR_MODE.value=vtm_GEAR_MODE;}    
  }
  function update_gui_hz()
  {
    if(++skip_cnt_Gui>=10)
    {
      skip_cnt_Gui=0;
      var timer_now=Date.now(); var timer_dt=timer_now-lastGui; lastGui=timer_now;
      vtm_GUI_HZ=(1000.0/parseFloat(timer_dt)*10.0).toFixed(1); 
      if(tm_GUI_HZ) tm_GUI_HZ.value=vtm_GUI_HZ;
    }
  }
  function start_rc()   {cmd_drv=1;drv_stop();}
  function start_self() {cmd_drv=2;drv_stop();}
  function stop_rc()    {cmd_drv=0;drv_stop();}
  function drv_stop()   {cmd_spd=cmd_str=0.0;}
  function gear_mode()  {cmd_gear=(cmd_gear>0)?0:1;}
  function drv_update() {cmd_spd=-joystick1.value.y;cmd_str=joystick2.value.x;} 
  function rsrv_mode()  {cmd_rsrv=(cmd_rsrv>0)?0:1;}
  function obsa_mode()  {cmd_obsa=(cmd_obsa>0)?0:1;}
  function snr_reset()  {cmd_reset=1;}
  
  function update_canvas()
  {
    canvas = document.getElementById("myCanvas");
    ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.globalAlpha=0.8; //0.0 = full transparancy/1.0 = no transparancy

    
    //-----------------------------------------------------------
    //-- distance circle 
    //-----------------------------------------------------------
    ctx.globalAlpha=0.4;
    var dist_val = cmd_dist*0.01;
    ctx.beginPath(); ctx.lineWidth = 2;
    ctx.arc(CV_CX, CV_CY-40, dist_val*CV_RATIO, 0, 2 * Math.PI); ctx.closePath();    
    ctx.strokeStyle="#537673"; ctx.stroke();
    

    //-----------------------------------------------------------
    //-- CAR BODY
    //-----------------------------------------------------------
    ctx.globalAlpha=1.0;
    ctx.beginPath();
    if(cross[0] == 1) ctx.fillStyle="GreenYellow"; 
    else if(cross[0] == 2) ctx.fillStyle="Gold"; 
    else  ctx.fillStyle="DimGray"; 
    ctx.fillRect(CV_CX-30,CV_CY-40,60,80); 
    ctx.fillStyle="DarkSlateGray"; ctx.fillRect(CV_CX-35,CV_CY-32,12,20); ctx.fillRect(CV_CX+23,CV_CY-32,12,20); ctx.fillRect(CV_CX-35,CV_CY+13,12,20); ctx.fillRect(CV_CX+23,CV_CY+13,12,20);
       

    ctx.globalAlpha=1.0;
    //-----------------------------------------------------------
    //-- Distance Circles (right)
    //-----------------------------------------------------------
    //dist[0] = 100;
    ctx.beginPath();
    ctx.arc(CV_CX+25, CV_CY-40, dist[0], 0, 2 * Math.PI);    ctx.closePath();
    ctx.lineWidth = 2; ctx.strokeStyle="#fd9bc2"; 
    ctx.stroke();    

    ctx.beginPath();
    ctx.arc(CV_CX+25, CV_CY-40, 5, 0, 2 * Math.PI); ctx.closePath();
    ctx.fillStyle="#de025a"; 
    ctx.fill();
            
    //-----------------------------------------------------------
    //-- Distance Circles (left)
    //-----------------------------------------------------------
    //dist[1] = 200;
    ctx.beginPath();
    ctx.arc(CV_CX-25, CV_CY-40, dist[1], 0, 2 * Math.PI);     ctx.closePath();
    ctx.lineWidth = 2; ctx.strokeStyle="#fd9bc2"; 
    ctx.stroke();    

    ctx.beginPath();
    ctx.arc(CV_CX-25, CV_CY-40, 5, 0, 2 * Math.PI); ctx.closePath();
    ctx.fillStyle="#de025a"; 
    ctx.fill();    
    

    //trg[3] = 200;
    //trg[4] = 200;
    //-----------------------------------------------------------
    //-- Target Arrow
    //-----------------------------------------------------------
    ctx.beginPath();
    ctx.moveTo(CV_CX, CV_CY-40);  ctx.lineTo(CV_CX+cross[3], CV_CY-40-cross[4]); ctx.closePath();
    ctx.lineWidth = 2; ctx.strokeStyle="#de025a"; 
    ctx.stroke();
    
    //-----------------------------------------------------------
    //-- Start Point Circle
    //-----------------------------------------------------------
    ctx.beginPath();
    ctx.arc(CV_CX, CV_CY-40, 5, 0, 2 * Math.PI); ctx.closePath();
    ctx.fillStyle="#de025a"; 
    ctx.fill();
    
    
    //-----------------------------------------------------------
    //-- Target Circle
    //-----------------------------------------------------------
    ctx.beginPath();
    ctx.arc(CV_CX+cross[3], CV_CY-40-cross[4], 10, 0, 2 * Math.PI); ctx.closePath();
    ctx.fillStyle="#de025a"; 
    ctx.fill();
    
    ctx.beginPath();
    ctx.arc(CV_CX+cross[3], CV_CY-40+cross[4], 10, 0, 2 * Math.PI); ctx.closePath();
    ctx.fillStyle="#de025a"; 
    ctx.fill();
    
    ctx.globalAlpha=0.5;

    //-----------------------------------------------------------
    //-- Filtered Target Arrow & Circle
    //-----------------------------------------------------------
    ctx.beginPath();
    ctx.moveTo(CV_CX, CV_CY-40);   ctx.lineTo(CV_CX+trg[3], CV_CY-40-trg[4]); ctx.closePath();
    ctx.lineWidth = 2; ctx.strokeStyle="#4c00e6"; 
    ctx.stroke();
    
    ctx.beginPath();
    ctx.arc(CV_CX+trg[3], CV_CY-40-trg[4], 20, 0, 2 * Math.PI); ctx.closePath();
    ctx.fillStyle="#4c00e6"; 
    ctx.fill();
    
    if(cmd_obsa<=0) return;

    //-----------------------------------------------------------
    //-- obstacle in-range circle 
    //-----------------------------------------------------------
    ctx.globalAlpha=0.2;
    ctx.beginPath();
    ctx.arc(CV_CX, CV_CY-40, _pf_max_obs_dst*CV_RATIO, 0, 2 * Math.PI); ctx.closePath();
    ctx.fillStyle="#067F03"; ctx.fill();
        
    //-----------------------------------------------------------
    //-- Obstacle Circles
    //-----------------------------------------------------------
    ctx.globalAlpha=0.5;
    var _cx, _cy, _ct, _cd, _cdpx;
    var _ax, _ay, _ax1, _ay1, _ax2, _ay2, _al, _aw; //_asl, _asw;
    var _pf_ang, _pf_dst, _pf_final;
    var _pf_sum = 0.0;
    var _pf_sum_cnt = 0;
    
    for(let i=0; i<CNT; i++)  // CNT==21
    {
        _cd   = obs[i][0]; // distance (0.0 ~ 6.0)      
        _ct   = obs[i][1]; // theta angle (rad) // 40~140 deg // = parseFloat( (140) - i * 40.0 / 10.0 ) * Math.PI / 180.0;
        _cdpx = obs[i][2]; // distance in pixel
        _cx   = CV_CX + obs[i][3]; // x pos in pixel
        _cy   = CV_CY - obs[i][4]; // y pos in pixel

        //------------------------------------------------------       
        // _pf_ang : potential force from angle (-1.0 ~ 1.0)
        //------------------------------------------------------
        if      (_ct > Math.PI/4 && _ct < Math.PI/2)    _pf_ang = -Math.sin(_ct); // right side angle (minus force)
        else if (_ct > Math.PI/2 && _ct < 3*Math.PI/4)  _pf_ang = Math.sin(_ct);  // left side angle (plus force)
        else                                            _pf_ang = 0.0;

        //------------------------------------------------------       
        // _pf_dst : potential force from distance (0.0 ~ 1.0)
        //------------------------------------------------------
        //_pf_dst  = (5.0 - _cd) / 5.0;     // let max_distance = 5.0
        //_pf_dst  = (2.0 - _cd) / 2.0;     // let max_distance = 2.0

        _pf_dst  = (_pf_max_obs_dst - _cd) / _pf_max_obs_dst;     // let max_distance = 2.0    
        if(_cd < 0.25)          _pf_dst = 0.0;  // sensor operating range : 0.2~8.0 (m)
        else if(_pf_dst < 0.02) _pf_dst = 0.0;  // negative limit check
        else if(_pf_dst > 1.0)  _pf_dst = 1.0;  // positive limit check

        //------------------------------------------------------       
        // _pf_final : final potential force (-1.0 ~ 1.0)
        //------------------------------------------------------
        //_pf_final = _pf_ang * _pf_dst;
        _pf_final = _pf_ang * _pf_dst;
        
        //------------------------------------------------------       
        // _al : arrow length, _aw : arrow width
        //------------------------------------------------------
        _al   = Math.abs(_pf_final) * 200;
        //_aw   = Math.abs(_pf_final) * 10;
        _aw   = 4;

        //------------------------------------------------------       
        // arrow from (_cx, _cy) to (_ax, _ay)
        //------------------------------------------------------
        if(i<10) // left 
        {
          _ax  = _cx + _al;      _ay  = _cy;
          _ax1 = _cx + _al - 5;  _ay1  = _cy - 5;
          _ax2 = _cx + _al - 5;  _ay2  = _cy + 5;
        }
        else // right
        {
          _ax  = _cx - _al;      _ay  = _cy;
          _ax1 = _cx - _al + 5;  _ay1  = _cy - 5;
          _ax2 = _cx - _al + 5;  _ay2  = _cy + 5;
        } 
 
        //------------------------------------------------------       
        // If Considerable Obstacles, then process...
        //------------------------------------------------------        
        if(Math.abs(_pf_final) > 0 && i != 10) 
        {
          _pf_sum += _pf_final;
          _pf_sum_cnt++;

          //-------------------------------------------------------------------
          //-- OBS Circles
          //-------------------------------------------------------------------
          ctx.globalAlpha=0.5;
          ctx.beginPath(); 
          ctx.arc(_cx, _cy, 30, 0, 2 * Math.PI); ctx.closePath();
          ctx.fillStyle="DarkGray"; 
          ctx.fill(); 

          //-------------------------------------------------------------------
          //-- OBS Potential Field Arrow
          //-------------------------------------------------------------------
          ctx.globalAlpha=0.5;
          ctx.lineWidth = _aw; 

          if(i<10)
            ctx.strokeStyle="#de5724"; // red
          else
            ctx.strokeStyle="#2446de"; // blue

          ctx.beginPath();
          ctx.moveTo(_cx, _cy); ctx.lineTo(_ax, _ay); ctx.closePath();
          ctx.stroke(); 

          ctx.beginPath();
          ctx.moveTo(_ax, _ay); ctx.lineTo(_ax1, _ay1); ctx.closePath();
          ctx.stroke(); 

          ctx.beginPath();
          ctx.moveTo(_ax, _ay); ctx.lineTo(_ax2, _ay2); ctx.closePath();
          ctx.stroke(); 

        }              
    }    
    
    //------------------------------------------------------       
    // _pf_average : average of potential force (-1.0~1.0)
    //------------------------------------------------------
    //_pf_avg = (_pf_sum_cnt > 0) ? _pf_sum / _pf_sum_cnt : 0.0;
    //_pf_avg = _pf_sum / _pf_sum_cnt;
    
    //------------------------------------------------------       
    // _asl : pf_sum arrow length, _asw : pf_sum arrow width
    //------------------------------------------------------
    //_asl   = _pf_average * _pf_angle_gain * 300;
    //_asw   = _pf_average * _pf_angle_gain * 20;

    //------------------------------------------------------       
    // arrow from (BODY) to (BODY ARROW)
    //------------------------------------------------------
    ctx.globalAlpha=1.0;

    /*
    if(_asl < -10) // LEFT BODY ARROW
    {
      ctx.strokeStyle="#2446de"; // blue
      ctx.fillStyle="#2446de";   // blue

      ctx.beginPath();ctx.lineWidth = 20;
      ctx.moveTo(CV_CX-40,          CV_CY);  
      ctx.lineTo(CV_CX-40+_asl,     CV_CY);
      ctx.closePath();
      ctx.stroke(); 

      ctx.beginPath();
      ctx.moveTo(CV_CX-40+_asl-20,  CV_CY);  
      ctx.lineTo(CV_CX-40+_asl,     CV_CY-20);
      ctx.lineTo(CV_CX-40+_asl,     CV_CY+20);
      ctx.closePath();
      ctx.fill(); 

    }
    else if(_asl > 10) // RIGHT BODY ARROW
    {
      ctx.strokeStyle="#de5724"; // red
      ctx.fillStyle="#de5724";   // red

      ctx.beginPath();ctx.lineWidth = 20;
      ctx.moveTo(CV_CX+40,          CV_CY);  
      ctx.lineTo(CV_CX+40+_asl,     CV_CY);
      ctx.closePath();
      ctx.stroke(); 

      ctx.beginPath();
      ctx.moveTo(CV_CX+40+_asl+20,  CV_CY);  
      ctx.lineTo(CV_CX+40+_asl,     CV_CY-20);
      ctx.lineTo(CV_CX+40+_asl,     CV_CY+20);
      ctx.closePath();
      ctx.fill(); 
    }
    */
        
    //-------------------------------------------------------------------------       
    // Final Target Arrow & Circle (filtered target arrow + obstacle adjusted)
    //-------------------------------------------------------------------------    
    var f_trgAng = (trg[2] - _pf_angle_f * 45) * Math.PI/180.0;    
    var f_trgX = parseInt(trg[1]*CV_RATIO*Math.cos(f_trgAng));
    var f_trgY = parseInt(trg[1]*CV_RATIO*Math.sin(f_trgAng));
   
    ctx.globalAlpha=0.5;

    ctx.beginPath();
    ctx.moveTo(CV_CX, CV_CY-40);    ctx.lineTo(CV_CX+f_trgX, CV_CY-40-f_trgY);    ctx.closePath();
    ctx.lineWidth = 2; ctx.strokeStyle="Gold"; 
    ctx.stroke();    

    ctx.beginPath();
    ctx.arc(CV_CX+f_trgX, CV_CY-40-f_trgY, 20, 0, 2 * Math.PI);
    ctx.closePath();
    ctx.fillStyle="Gold"; 
    ctx.fill();
    

    //-------------------------------------------------------------------------       
    // Obstacle Arrow
    //-------------------------------------------------------------------------    
    ctx.globalAlpha=1.0;

    var _obl = 25.0;
    var _oba_length = 25 + _pf_speed_f / _pf_speed_gain * 100;
    var _oba_angle  = _pf_angle_f * 45 * Math.PI/180.0;   
    var _oba_sin = Math.sin(_oba_angle);
    var _oba_cos = Math.cos(_oba_angle);
    var _oba_ct = parseInt(250.0 * _pf_speed_f / _pf_speed_gain);
    var _oba_clr = "rgb(" + (250-_oba_ct) + " " + (_oba_ct) + " 50 / 70%)";
    //console.log(_oba_clr);

    var _oba_x = CV_CX    + _oba_length * _oba_sin;
    var _oba_y = CV_CY-40 - _oba_length * _oba_cos;
    var _oba_x1 = _oba_x + _obl * _oba_sin;
    var _oba_y1 = _oba_y - _obl * _oba_cos;
    var _oba_x2 = _oba_x + _obl*0.7 * _oba_cos;
    var _oba_y2 = _oba_y + _obl*0.7 * _oba_sin;
    var _oba_x3 = _oba_x - _obl*0.7 * _oba_cos;
    var _oba_y3 = _oba_y - _obl*0.7 * _oba_sin;
    
    ctx.beginPath();
    ctx.moveTo(CV_CX, CV_CY-40); ctx.lineTo(_oba_x, _oba_y);ctx.closePath();
    ctx.lineWidth = 10; //ctx.strokeStyle="#2EBA2A"; 
    ctx.strokeStyle=_oba_clr; 
    ctx.stroke();
    
    ctx.beginPath();
    ctx.moveTo(_oba_x1, _oba_y1); ctx.lineTo(_oba_x2, _oba_y2);ctx.lineTo(_oba_x3, _oba_y3);  ctx.lineTo(_oba_x1, _oba_y1);ctx.closePath();
    //ctx.fillStyle="#2EBA2A"; 
    ctx.fillStyle=_oba_clr; 
    ctx.fill();
    

  }
</script>
<style>
  input[type=text] { text-align: center; font-familiy: '굴림'; font-size: 16px; font-weight: bold; width: 60px;}
  input[type=button] { text-align: center; font-familiy: '굴림'; font-size: 26px; font-weight: bold; height:50px;}    
</style>
<body onload="body_onload_handler()">
  <table width="100%" style="text-align: center; font-weight: bold;">
  <tr>
  <td width="18%"> 갱신속도 <input type="text" id="tm_TIMER_HZ" value="0"></td>
  <td width="16%"> 화면속도 <input type="text" id="tm_GUI_HZ" value="0"></td>
  <td width="16%"> 운전모드 <input type="text" id="tm_DRIVE_ON" value="0"></td>
  <td width="16%"> 차량속력 <input type="text" id="tm_SPEED_KMH" value="0"></td>
  <td width="16%"> 차량회전 <input type="text" id="tm_STEER_DEG" value="0"></td>
  <td width="18%"> 장애물회피 <input type="text" id="tm_GEAR_MODE" value="0"></td>
  </tr>
  <tr><td colspan="6"></td></tr>
  <tr><td colspan="6">
  <table width="100%" style="text-align: center; font-weight: bold;">
  <tr>
    <td style="background-color:lightgray; width:250px; height:250px;">
      <div id="stick1" align="center" style="position:relative;">
      <table><tr><td style="width:50px; height:50px; background-color:darkgreen;"></td></tr></table>
      </div>
    </td>
    <td>
    <input type="button" value="원격시작" style="width:30%; "  onclick="start_rc()">&nbsp;
    <input type="button" value="자율시작" style="width:30%;"  onclick="start_self()">&nbsp;
    <input type="button" value="종    료"  style="width:30%;" onclick="stop_rc()"><br><br>
    <input type="button" value="장애회피" style="width:30%;"  onclick="obsa_mode();">&nbsp;
    <input type="button" value="센서회전" style="width:30%;"  onclick="rsrv_mode();">&nbsp;
    <input type="button" value="센서리셋" style="width:30%;"  onclick="snr_reset();"><br><br>
    <h style="font-size:16px;">추종간격(cm):</h>    <input id="tm_cmd_dist"   type="number" value="0" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" >&nbsp;&nbsp;
    <h style="font-size:16px;">속도배율(%):</h> <input id="tm_cmd_thgain" type="number" value="0" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" >&nbsp;&nbsp;
    <h style="font-size:16px;">조향배율(%):</h> <input id="tm_cmd_stgain" type="number" value="0" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" ><br><br>
    <h style="font-size:16px;">[RC차량] 최소속도(%):</h> <input id="tm_cmd_minspd" type="number" value="0" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" >&nbsp;&nbsp;   
    <h style="font-size:16px;">최대속도(%):</h> <input id="tm_cmd_maxspd" type="number" value="0" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" ><br><br>
    <h style="font-size:16px;">[장애물회피] 각도배율(%):</h> <input id="tm_cmd_obgain" type="number" value="100" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" >&nbsp;&nbsp;   
    <h style="font-size:16px;">거리배율(%):</h> <input id="tm_cmd_obgain2" type="number" value="100" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" >   
    <!--<h style="font-size:16px;">장애물인식거리(cm):</h> <input id="tm_cmd_obdist" type="number" value="200" style="width:32px; font-size:14px; font-weight:bold; text-align:center;" >-->
      

    <!--<input type="button" value="비상 정지" style="width:45%;"  onclick="drv_stop();">&nbsp;
    <input type="button" value="기어 모드" style="width:45%;"  onclick="gear_mode();">-->    
    </td>
    <td style="background-color:lightgray; width:250px; height:250px;">
      <div id="stick2" align="center" style="position:relative;">
      <table><tr><td style="width:50px; height:50px; background-color:darkblue;"></td></tr></table>
      </div>
    </td>
  </tr>
  </table>
  </td></tr>
  <tr><td colspan="6"></td></tr>
  <tr><td colspan="6"></td></tr>
  <tr><td colspan="6"></td></tr>
  <tr>
  <td colspan="6">
    <table width="100%"><tr><td style="text-align:center;">
      <canvas id="myCanvas" width="960" height="800" style="background-color:LightGray;"></canvas>
    </td></tr></table>
  </td>
  </tr>
  </table>  
<script>
  class JoystickController
  {
    constructor( stickID, maxDistance, deadzone, joytype=0)
    {
      this.id = stickID;
      let stick = document.getElementById(stickID);
      this.dragStart = null;
      this.touchId = null;
      this.active = false;
      this.value = { x: 0, y: 0 };
      this.jtype = joytype;
      let self = this;
      function handleDown(event)
      {
        self.active = true;
        stick.style.transition = "0s";
        event.preventDefault();
        if (event.changedTouches) self.dragStart = { x: event.changedTouches[0].clientX, y: event.changedTouches[0].clientY };
        else self.dragStart = { x: event.clientX, y: event.clientY };
        if (event.changedTouches) self.touchId = event.changedTouches[0].identifier;
      }
      function handleMove(event)
      {
        if ( !self.active ) return;
        let touchmoveId = null;
        if (event.changedTouches)
        {
          for (let i = 0; i < event.changedTouches.length; i++)
          {
            if(self.touchId == event.changedTouches[i].identifier)
            {
              touchmoveId = i;
              event.clientX = event.changedTouches[i].clientX;
              event.clientY = event.changedTouches[i].clientY;
            }
          }
          if(touchmoveId == null) return;
        }
        let xDiff = 0; let yDiff = 0;
        switch(self.jtype)
        {
          case 0: xDiff = event.clientX - self.dragStart.x; yDiff = event.clientY - self.dragStart.y; break;
          case 1: xDiff = 0; yDiff = event.clientY - self.dragStart.y; break;
          case 2: xDiff = event.clientX - self.dragStart.x; yDiff = 0; break;
        }
        const angle = Math.atan2(yDiff, xDiff);
        const distance = Math.min(maxDistance, Math.hypot(xDiff, yDiff));
        const xPosition = distance * Math.cos(angle);
        const yPosition = distance * Math.sin(angle);
        switch(self.jtype)
        {
          case 0: stick.style.transform = String("translate3d("+xPosition+"px, "+yPosition+"px, 0px)"); break;
          case 1: stick.style.transform = String("translate3d(0px, "+yPosition+"px, 0px)"); break;
          case 2: stick.style.transform = String("translate3d("+xPosition+"px, 0px,	0px)"); break;
        }
        const distance2 = (distance < deadzone) ? 0 : maxDistance / (maxDistance - deadzone) * (distance - deadzone);
        const xPosition2 = distance2 * Math.cos(angle);
        const yPosition2 = distance2 * Math.sin(angle);
        const xPercent = parseFloat((xPosition2 / maxDistance).toFixed(4));
        const yPercent = parseFloat((yPosition2 / maxDistance).toFixed(4));	    
        self.value = { x: xPercent, y: yPercent };
      }
      function handleUp(event)
      {
        if ( !self.active ) return;
        if (event.changedTouches && self.touchId != event.changedTouches[0].identifier) return;
        stick.style.transition = ".2s";
        stick.style.transform = "translate3d(0px, 0px, 0px)";
        self.value = { x: 0, y: 0 };
        self.touchId = null;
        self.active = false;
      }
      stick.addEventListener("mousedown", handleDown);
      stick.addEventListener("touchstart", handleDown);
      document.addEventListener("mousemove", handleMove, {passive: false});
      document.addEventListener("touchmove", handleMove, {passive: false});
      document.addEventListener("mouseup", handleUp);
      document.addEventListener("touchend", handleUp);
    }
  }

  let joystick1 = new JoystickController("stick1", 100, 8, 1);//1:vert
  let joystick2 = new JoystickController("stick2", 100, 8, 2);//2:horz
  function loop(){requestAnimationFrame(loop);}
  loop();
</script>
</body>
</html>
)rawliteral";

void WebManager::begin()
{
    //----------------------------------------------------------------------------------------------------------
    //-- Initialize WiFi AP
    //----------------------------------------------------------------------------------------------------------
    IPAddress local_ip(192,168,10,1);
    IPAddress subnet_ip(255,255,255,0);
    local_ip.fromString(WIFI_MYIP);
    
    WiFi.mode(WIFI_AP); 
    if(!WiFi.softAPConfig(local_ip, local_ip, subnet_ip)) Serial.println("[WIFI-AP] softAPConfig() failed.");
    delay(1000);

    String strMacAddress;
    uint8_t baseMac[6];
    if(ESP_OK == esp_wifi_get_mac(WIFI_IF_AP, baseMac)) strMacAddress = String(baseMac[4], HEX) + String(baseMac[5], HEX);   
    String strFullSSID = WIFI_SSID + "_" + strMacAddress;

    if(!WiFi.softAP(strFullSSID, WIFI_PASS)) Serial.println("[WIFI-AP] softAP() failed."); delay(2000);
      
    IPAddress myIP = WiFi.softAPIP();      
    Serial.print("[WIFI-AP] Initialized. ("); Serial.print(myIP); Serial.println(")");
    Serial.print("[WIFI-AP] (SSID: "); Serial.print(strFullSSID); Serial.print(") (PWD: "); Serial.print(WIFI_PASS); Serial.println(")");
      
    //----------------------------------------------------------------------------------------------------------
    //-- Begin WebServer
    //----------------------------------------------------------------------------------------------------------
    web_socket.onEvent(onEvent);
    web_server.addHandler(&web_socket);
    //web_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", index_html, processor); });
    web_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", index_html, NULL); });
    web_server.begin();

    Serial.print("[WIFI-WebServer] Open 'http://");      
    Serial.print(WiFi.softAPIP());    
    Serial.println("' in your browser to view device's contents...");
 

    initialized = true;
}

void WebManager::update()
{
    //web_server.handleClient();
    static uint32_t currMS = millis();
    if(millis() - currMS >= 1000)
    {
      web_socket.cleanupClients();
      currMS = millis();
    }    
}

String WebManager::processor(const String& var)
{
  Serial.println(var);

  if(var == "VAL_DRV")      { return String(servo_manager.cmd_drv); }
  else if(var == "VAL_SPD") { return String(servo_manager.cmd_spd); }
  else if(var == "VAL_STR") { return String(servo_manager.cmd_str); }
  else return String();
}

void WebManager::notifyClients() 
{
  web_socket.textAll(buildResponseString());  
}

void WebManager::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) 
{
  static char t_buff[1024] = {0};
  static char i_buff[16][16] = {{0}};

  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
  {
    data[len] = 0;
    
    //var cmd_string = "cmd," + cmd_drv + "," + cmd_spd + "," + cmd_str + ",";
    memcpy(t_buff, data, len+1);
    int c_cnt = 0;
    char* strToken = strtok(t_buff, ",");      
    while(strToken != NULL)
    {
      strcpy(i_buff[c_cnt], strToken);

      strToken = strtok(NULL, ",");
      c_cnt++;
    }

    if(strcmp(i_buff[0], "cmd") == 0)
    {
      web_manager.cmd_drv     = String(i_buff[1]).toInt();
      web_manager.cmd_spd     = String(i_buff[2]).toFloat();
      web_manager.cmd_str     = String(i_buff[3]).toFloat();
      web_manager.cmd_gear    = String(i_buff[4]).toInt();

      web_manager.cmd_dist    = String(i_buff[5]).toInt() * 0.01;
      web_manager.cmd_thgain  = String(i_buff[6]).toInt() * 0.01;
      web_manager.cmd_stgain  = String(i_buff[7]).toInt() * 0.01;
      web_manager.cmd_maxspd  = String(i_buff[8]).toInt() * 0.01;
      web_manager.cmd_minspd  = String(i_buff[9]).toInt() * 0.01;
      web_manager.cmd_obgain  = String(i_buff[10]).toInt() * 0.01;
      web_manager.cmd_obgain2 = String(i_buff[11]).toInt() * 0.01;
      web_manager.cmd_rsrv    = String(i_buff[12]).toInt();
      web_manager.cmd_obsa    = String(i_buff[13]).toInt();
      web_manager.cmd_reset   = String(i_buff[14]).toInt();
      //web_manager.cmd_obdist = String(i_buff[11]).toInt() * 0.01;

      //Serial.println(web_manager.cmd_gear);
      

      prepareResponse();
      notifyClients();
    }
  }
}
void WebManager::prepareResponse()
{
  vtm_DRIVE_ON  = servo_manager.cmd_drv;
  vtm_SPEED_KMH = servo_manager.cmd_spd;
  vtm_STEER_DEG = servo_manager.cmd_str;
  vtm_GEAR_MODE = servo_manager.cmd_gear;

  web_manager.sts_trg[0] = 1;
  web_manager.sts_trg[1] = uss_sensor.mData.r_f*1000;
  web_manager.sts_trg[2] = uss_sensor.mData.theta_f*RAD_TO_DEG*100;
  
  web_manager.sts_dist[1] = uss_sensor.mData.us_d1*1000;
  web_manager.sts_dist[0] = uss_sensor.mData.us_d2*1000;
  web_manager.sts_dist[2] = 0;

  web_manager.sts_cross[0] = uss_sensor.currValidSensorData;
  web_manager.sts_cross[1] = uss_sensor.mData.r*1000;
  web_manager.sts_cross[2] = uss_sensor.mData.theta*RAD_TO_DEG*100;

  //---------------------------------------
  //-- TESTING
  //---------------------------------------
  /*
  static float theta = 0;
  theta += 0.1;
  if(theta > 2*M_PI) theta=0;  
  */
  /*
  web_manager.sts_trg[0] = 1;
  web_manager.sts_trg[1] = 3000;
  web_manager.sts_trg[2] = theta*180.0/M_PI;
  */
  /*
  int cycle = 600 * sin(theta);

  for(int i=0;i<19;i++)
  {
    for(int k=0;k<3;k++)
    {
      web_manager.sts_obs[3*i+k] = cycle + 2000 + 1000 * k; // 2m, 3m, 4m
    }
  } 
  */ 
}
String WebManager::buildResponseString()
{
  char str_resp[1600];   

  sprintf(str_resp, "%d,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", 
    //vtm_DRIVE_ON,             vtm_SPEED_KMH,            vtm_STEER_DEG,          vtm_GEAR_MODE,   
    vtm_DRIVE_ON,             vtm_SPEED_KMH,            vtm_STEER_DEG,          web_manager.cmd_obsa,   
    web_manager.sts_trg[0],   web_manager.sts_trg[1],   web_manager.sts_trg[2],
    web_manager.sts_dist[0],  web_manager.sts_dist[1],  web_manager.sts_dist[2],
    web_manager.sts_cross[0], web_manager.sts_cross[1], web_manager.sts_cross[2],   
    _pf_angle_f,              _pf_angle_gain,           _pf_speed_f,           _pf_speed_gain,    _pf_max_obs_dst,
    lidar_dist[0],lidar_dist[1],lidar_dist[2],lidar_dist[3],lidar_dist[4],lidar_dist[5],lidar_dist[6],lidar_dist[7],lidar_dist[8],lidar_dist[9],
    lidar_dist[10],lidar_dist[11],lidar_dist[12],lidar_dist[13],lidar_dist[14],lidar_dist[15],lidar_dist[16],lidar_dist[17],lidar_dist[18],lidar_dist[19],lidar_dist[20]
  );  

 // Serial.println(str_resp);

  /*
  if(web_manager.obs_urm)
  {
    sprintf(str_resp, "%d,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\
                    %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\
                    %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\
                    %.3f,", 
  
    vtm_DRIVE_ON,          vtm_SPEED_KMH,          vtm_STEER_DEG,         vtm_GEAR_MODE,   
    web_manager.sts_trg[0],   web_manager.sts_trg[1],   web_manager.sts_trg[2],
    web_manager.sts_dist[0],  web_manager.sts_dist[1],  web_manager.sts_dist[2],
    web_manager.sts_cross[0], web_manager.sts_cross[1],  web_manager.sts_cross[2],    
    web_manager.sts_urm[0],  web_manager.sts_urm[1],   web_manager.sts_urm[2],   web_manager.sts_urm[3],   web_manager.sts_urm[4],   web_manager.sts_urm[5],   web_manager.sts_urm[6],   web_manager.sts_urm[7],   web_manager.sts_urm[8],   web_manager.sts_urm[9],
    web_manager.sts_urm[10], web_manager.sts_urm[11],  web_manager.sts_urm[12],  web_manager.sts_urm[13],  web_manager.sts_urm[14],  web_manager.sts_urm[15],  web_manager.sts_urm[16],  web_manager.sts_urm[17],  web_manager.sts_urm[18],  web_manager.sts_urm[19],
    web_manager.sts_urm[20], web_manager.sts_urm[21],  web_manager.sts_urm[22],  web_manager.sts_urm[23],  web_manager.sts_urm[24],  web_manager.sts_urm[25],  web_manager.sts_urm[26],  web_manager.sts_urm[27],  web_manager.sts_urm[28],  web_manager.sts_urm[29],
    web_manager.sts_urm[30]
    );  
  }
  else
  {
    sprintf(str_resp, "%d,%.3f,%.3f,%d,%d,%d,%d,\
                      %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                      %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                      %d,%d,%d,%d,%d,%d,%d,%d,\
                      %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                      %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                      %d,%d,%d,%d,%d,%d,%d,%d,", 
    
      vtm_DRIVE_ON,          vtm_SPEED_KMH,          vtm_STEER_DEG,         vtm_GEAR_MODE,   web_manager.sts_trg[0],   web_manager.sts_trg[1],   web_manager.sts_trg[2],
      web_manager.sts_obs[0],  web_manager.sts_obs[1],   web_manager.sts_obs[2],   web_manager.sts_obs[3],   web_manager.sts_obs[4],   web_manager.sts_obs[5],   web_manager.sts_obs[6],   web_manager.sts_obs[7],   web_manager.sts_obs[8],   web_manager.sts_obs[9],
      web_manager.sts_obs[10], web_manager.sts_obs[11],  web_manager.sts_obs[12],  web_manager.sts_obs[13],  web_manager.sts_obs[14],  web_manager.sts_obs[15],  web_manager.sts_obs[16],  web_manager.sts_obs[17],  web_manager.sts_obs[18],  web_manager.sts_obs[19],
      web_manager.sts_obs[20], web_manager.sts_obs[21],  web_manager.sts_obs[22],  web_manager.sts_obs[23],  web_manager.sts_obs[24],  web_manager.sts_obs[25],  web_manager.sts_obs[26],  web_manager.sts_obs[27], 
      web_manager.sts_sth[0],  web_manager.sts_sth[1],   web_manager.sts_sth[2],   web_manager.sts_sth[3],   web_manager.sts_sth[4],   web_manager.sts_sth[5],   web_manager.sts_sth[6],   web_manager.sts_sth[7],   web_manager.sts_sth[8],   web_manager.sts_sth[9],
      web_manager.sts_sth[10], web_manager.sts_sth[11],  web_manager.sts_sth[12],  web_manager.sts_sth[13],  web_manager.sts_sth[14],  web_manager.sts_sth[15],  web_manager.sts_sth[16],  web_manager.sts_sth[17],  web_manager.sts_sth[18],  web_manager.sts_sth[19],
      web_manager.sts_sth[20], web_manager.sts_sth[21],  web_manager.sts_sth[22],  web_manager.sts_sth[23],  web_manager.sts_sth[24],  web_manager.sts_sth[25],  web_manager.sts_sth[26],  web_manager.sts_sth[27]      
    );  
  }
  */
  return String(str_resp);
}
/*
String WebManager::buildResponseString()
{
  char str_resp[1600];   
  sprintf(str_resp, "%d,%.3f,%.3f,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
                    %d,%d,", 
  
    vtm_DRIVE_ON,          vtm_SPEED_KMH,          vtm_STEER_DEG,         vtm_GEAR_MODE,   web_manager.sts_trg[0],   web_manager.sts_trg[1],   web_manager.sts_trg[2],
    web_manager.sts_obs[0],  web_manager.sts_obs[1],   web_manager.sts_obs[2],   web_manager.sts_obs[3],   web_manager.sts_obs[4],   web_manager.sts_obs[5],   web_manager.sts_obs[6],   web_manager.sts_obs[7],   web_manager.sts_obs[8],   web_manager.sts_obs[9],
    web_manager.sts_obs[10], web_manager.sts_obs[11],  web_manager.sts_obs[12],  web_manager.sts_obs[13],  web_manager.sts_obs[14],  web_manager.sts_obs[15],  web_manager.sts_obs[16],  web_manager.sts_obs[17],  web_manager.sts_obs[18],  web_manager.sts_obs[19],
    web_manager.sts_obs[20], web_manager.sts_obs[21],  web_manager.sts_obs[22],  web_manager.sts_obs[23],  web_manager.sts_obs[24],  web_manager.sts_obs[25],  web_manager.sts_obs[26],  web_manager.sts_obs[27],  web_manager.sts_obs[28],  web_manager.sts_obs[29],
    web_manager.sts_obs[30], web_manager.sts_obs[31],  web_manager.sts_obs[32],  web_manager.sts_obs[33],  web_manager.sts_obs[34],  web_manager.sts_obs[35],  web_manager.sts_obs[36],  web_manager.sts_obs[37],  web_manager.sts_obs[38],  web_manager.sts_obs[39],
    web_manager.sts_obs[40], web_manager.sts_obs[41],  web_manager.sts_obs[42],  web_manager.sts_obs[43],  web_manager.sts_obs[44],  web_manager.sts_obs[45],  web_manager.sts_obs[46],  web_manager.sts_obs[47],  web_manager.sts_obs[48],  web_manager.sts_obs[49],
    web_manager.sts_obs[50], web_manager.sts_obs[51], 
    web_manager.sts_sth[0],  web_manager.sts_sth[1],   web_manager.sts_sth[2],   web_manager.sts_sth[3],   web_manager.sts_sth[4],   web_manager.sts_sth[5],   web_manager.sts_sth[6],   web_manager.sts_sth[7],   web_manager.sts_sth[8],   web_manager.sts_sth[9],
    web_manager.sts_sth[10], web_manager.sts_sth[11],  web_manager.sts_sth[12],  web_manager.sts_sth[13],  web_manager.sts_sth[14],  web_manager.sts_sth[15],  web_manager.sts_sth[16],  web_manager.sts_sth[17],  web_manager.sts_sth[18],  web_manager.sts_sth[19],
    web_manager.sts_sth[20], web_manager.sts_sth[21],  web_manager.sts_sth[22],  web_manager.sts_sth[23],  web_manager.sts_sth[24],  web_manager.sts_sth[25],  web_manager.sts_sth[26],  web_manager.sts_sth[27],  web_manager.sts_sth[28],  web_manager.sts_sth[29],
    web_manager.sts_sth[30], web_manager.sts_sth[31],  web_manager.sts_sth[32],  web_manager.sts_sth[33],  web_manager.sts_sth[34],  web_manager.sts_sth[35],  web_manager.sts_sth[36],  web_manager.sts_sth[37],  web_manager.sts_sth[38],  web_manager.sts_sth[39],
    web_manager.sts_sth[40], web_manager.sts_sth[41],  web_manager.sts_sth[42],  web_manager.sts_sth[43],  web_manager.sts_sth[44],  web_manager.sts_sth[45],  web_manager.sts_sth[46],  web_manager.sts_sth[47],  web_manager.sts_sth[48],  web_manager.sts_sth[49],
    web_manager.sts_sth[50], web_manager.sts_sth[51]
  );  
  
  return String(str_resp);
}
*/
void WebManager::onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
          void *arg, uint8_t *data, size_t len) 
{
  switch (type) 
  {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}





/*
void WebManager::setup_handlers()
{
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send_P(200, "text/html", index_html, processor);
    });

    //web_server.on("/",        handle_Root);
    //web_server.on("/query",   handle_Query);
    //web_server.onNotFound(    handle_NotFound);   
}
void WebManager::handle_Query() 
{
  for (uint8_t i = 0; i < web_server.args(); i++) 
  {   
    if(web_server.argName(i).equals("cmd_drv")) { web_manager.cmd_drv = web_server.arg(i).toInt(); }
    if(web_server.argName(i).equals("cmd_spd")) { web_manager.cmd_spd = web_server.arg(i).toFloat(); }
    if(web_server.argName(i).equals("cmd_str")) { web_manager.cmd_str = web_server.arg(i).toFloat(); }
  }
  //Serial.print("CMD_DRV: "); Serial.print(web_manager.cmd_drv); Serial.print(", CMD_SPD: "); Serial.print(web_manager.cmd_spd); Serial.print(", CMD_STR: "); Serial.print(web_manager.cmd_str); Serial.println();

  vtm_DRIVE_ON = servo_manager.cmd_drv;
  vtm_SPEED_KMH = servo_manager.cmd_spd;
  vtm_STEER_DEG = servo_manager.cmd_str;
  
  web_manager.sts_trg[0] = 1;
  web_manager.sts_trg[1] = ebus_data.r_f*1000;
  web_manager.sts_trg[2] = ebus_data.theta_f*RAD_TO_DEG;

  for(int i=0;i<36;i++)
  {
    for(int k=0;k<3;k++)
    {
      web_manager.sts_obs[3*i+k] = 0;
    }
  }


  //------------------------------------------------------
  //-- FOR TESTING
  //------------------------------------------------------

  char str_prm[512];   
  sprintf(str_prm, "%d,%.3f,%.3f,%d,%d,%d,", vtm_DRIVE_ON,vtm_SPEED_KMH,vtm_STEER_DEG,web_manager.sts_trg[0],web_manager.sts_trg[1],web_manager.sts_trg[2]);  

  char str_obs[1024];
  sprintf(str_obs,   
  "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
   %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
   %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
   %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", // 36*3 = 108 = 30+30+30+18
   web_manager.sts_obs[0],  web_manager.sts_obs[1],   web_manager.sts_obs[2],   web_manager.sts_obs[3],   web_manager.sts_obs[4],   web_manager.sts_obs[5],   web_manager.sts_obs[6],   web_manager.sts_obs[7],   web_manager.sts_obs[8],   web_manager.sts_obs[9],
   web_manager.sts_obs[10], web_manager.sts_obs[11],  web_manager.sts_obs[12],  web_manager.sts_obs[13],  web_manager.sts_obs[14],  web_manager.sts_obs[15],  web_manager.sts_obs[16],  web_manager.sts_obs[17],  web_manager.sts_obs[18],  web_manager.sts_obs[19],
   web_manager.sts_obs[20], web_manager.sts_obs[21],  web_manager.sts_obs[22],  web_manager.sts_obs[23],  web_manager.sts_obs[24],  web_manager.sts_obs[25],  web_manager.sts_obs[26],  web_manager.sts_obs[27],  web_manager.sts_obs[28],  web_manager.sts_obs[29],
   web_manager.sts_obs[30], web_manager.sts_obs[31],  web_manager.sts_obs[32],  web_manager.sts_obs[33],  web_manager.sts_obs[34],  web_manager.sts_obs[35],  web_manager.sts_obs[36],  web_manager.sts_obs[37],  web_manager.sts_obs[38],  web_manager.sts_obs[39],
   web_manager.sts_obs[40], web_manager.sts_obs[41],  web_manager.sts_obs[42],  web_manager.sts_obs[43],  web_manager.sts_obs[44],  web_manager.sts_obs[45],  web_manager.sts_obs[46],  web_manager.sts_obs[47],  web_manager.sts_obs[48],  web_manager.sts_obs[49],
   web_manager.sts_obs[50], web_manager.sts_obs[51],  web_manager.sts_obs[52],  web_manager.sts_obs[53],  web_manager.sts_obs[54],  web_manager.sts_obs[55],  web_manager.sts_obs[56],  web_manager.sts_obs[57],  web_manager.sts_obs[58],  web_manager.sts_obs[59],
   web_manager.sts_obs[60], web_manager.sts_obs[61],  web_manager.sts_obs[62],  web_manager.sts_obs[63],  web_manager.sts_obs[64],  web_manager.sts_obs[65],  web_manager.sts_obs[66],  web_manager.sts_obs[67],  web_manager.sts_obs[68],  web_manager.sts_obs[69],
   web_manager.sts_obs[70], web_manager.sts_obs[71],  web_manager.sts_obs[72],  web_manager.sts_obs[73],  web_manager.sts_obs[74],  web_manager.sts_obs[75],  web_manager.sts_obs[76],  web_manager.sts_obs[77],  web_manager.sts_obs[78],  web_manager.sts_obs[79],
   web_manager.sts_obs[80], web_manager.sts_obs[81],  web_manager.sts_obs[82],  web_manager.sts_obs[83],  web_manager.sts_obs[84],  web_manager.sts_obs[85],  web_manager.sts_obs[86],  web_manager.sts_obs[87],  web_manager.sts_obs[88],  web_manager.sts_obs[89],
   web_manager.sts_obs[90], web_manager.sts_obs[91],  web_manager.sts_obs[92],  web_manager.sts_obs[93],  web_manager.sts_obs[94],  web_manager.sts_obs[95],  web_manager.sts_obs[96],  web_manager.sts_obs[97],  web_manager.sts_obs[98],  web_manager.sts_obs[99],
   web_manager.sts_obs[100],web_manager.sts_obs[101], web_manager.sts_obs[102], web_manager.sts_obs[103], web_manager.sts_obs[104], web_manager.sts_obs[105], web_manager.sts_obs[106], web_manager.sts_obs[107], web_manager.sts_obs[108]);
   
  sprintf(buff, "<input type='hidden' id='str_prm' value='%s'><input type='hidden' id='str_obs' value='%s'>\n", str_prm, str_obs);
  web_server.send(200, "text/html", buff);  
}
void WebManager::handle_Root() 
{ 
  web_server.send(200, "text/html", index_html);          
}
//sandbox='allow-scripts allow-modals allow-same-origin'

void WebManager::handle_NotFound() 
{
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += web_server.uri();
  message += "\nMethod: ";
  message += (web_server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += web_server.args();
  message += "\n";
  for (uint8_t i = 0; i < web_server.args(); i++) {
    message += " " + web_server.argName(i) + ": " + web_server.arg(i) + "\n";
  }
  web_server.send(404, "text/plain", message);      
}
*/
