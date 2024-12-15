#ifndef __WEB_MANAGER
#define __WEB_MANAGER

#if defined(ARDUINO) && ARDUINO >= 100
  #include "arduino.h"
#else
  #include "WProgram.h"
#endif

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

class WebManager
{
public:
    WebManager() { initialized = false; }
    ~WebManager() {}

    
public:

    int   cmd_drv = 0;
    int   cmd_gear = 0;
    int   cmd_rsrv = 0;
    int   cmd_obsa = 0; // obastacle avoidance driving mode
    int   cmd_reset = 0;
    
    float cmd_spd = 0.0;
    float cmd_str = 0.0;
    float cmd_dist = 1.0;
    float cmd_thgain = 0.5;
    float cmd_stgain = 1.5;
    float cmd_maxspd = 0.22;
    float cmd_minspd = 0.18;
    float cmd_obgain = 1.0;
    float cmd_obgain2 = 1.0;
    float cmd_obdist = 2.0;


    int   sts_trg[3]={0}; // visible, distance, angle
    int   sts_dist[3]={0};
    int   sts_cross[3]={0};

    int   sts_obs[7*4]={0}; // range (mm)
    int   sts_sth[7*4]={0}; // strength 

    float sts_urm[31]={0.0}; // ultrasonic (m)

public:
    void begin();        
    void update();
    bool initialized;

    static  void    notifyClients();    
    static  void    handleWebSocketMessage(void *arg, uint8_t *data, size_t len);    
    static  void    onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len); 
    static  String  processor(const String& var);
    
    static  void    prepareResponse();
    static  String  buildResponseString();        
};

#endif
