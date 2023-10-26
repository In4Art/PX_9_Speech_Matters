#ifndef _WIFI_CONTROL_H_
#define _WIFI_CONTROL_H_

#include <Arduino.h>
#include <ESP8266WiFi.h>

#define WIFI_TIMEOUT 30000UL

void emptyRecon();

void handler(System_Event_t *event);

class WifiControl{
    public:
        WifiControl(char *ssid, char *pass, uint8_t pxn);
        int8_t init();
        void setTimeOut(uint32_t timeout);
        void setPreConn(void (*preCon)(void));
        void setPostConn(void (*postCon)(void));
        void run();
        void reConn();
        wl_status_t getStatus();
        uint16_t getReconCount();
        void resetReconCount();

        
        
    private:
        uint32_t _wifiCheckTime = 0;
        uint32_t _wifiReconPeriod = 60000; //attempt reconnect once per minute by default
        
        uint16_t _reconCount = 0; // counter for failed (re)connection attempts

        IPAddress _nodeAddr {192, 168, 1, 100};  //ip addresses from 101 for px-I
        const IPAddress _gate {192,168,1,1};
        const IPAddress _subn { 255, 255, 255, 0};

        char *_ssid;
        char *_pass;

        uint32_t _t_out = WIFI_TIMEOUT;

        //these functions are executed before and after reconnecting attempts
        //they can be used by the px under control to change some things in the main system
        void (*_preCon)(void) {NULL};
        void (*_postCon)(void) {NULL};

        
}; 


#endif