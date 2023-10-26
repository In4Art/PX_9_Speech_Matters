#ifndef _MODECONTROL_H_
#define _MODECONTROL_H_

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WifiControl.h>

typedef enum{
    PX_CC_MODE, //PX Central Command Mode
    PX_DEMO_MODE //PX Demo Mode
}px_mode_t;

class ModeControl {
    public:
        ModeControl(uint8_t demoPin, void (*demoCB) (uint32_t deltaTime, px_mode_t mode), uint32_t interval, WifiControl *wifi);
        px_mode_t init();//initialize mode control system, includes a WiFi status check
        void run(void);
        px_mode_t getControlMode();
        void setControlMode(px_mode_t mode);
        

    private:
        uint8_t _demoPin;
        uint32_t _oldTime = 0;
        uint32_t _currTime = 0;
        uint32_t _interval = 5000;
        px_mode_t _mode = PX_CC_MODE;

        // the demoCB is also called when switching from demo mode back to C&C mode
        // therefor the mode is also passed to the callback
        // like this it is possile to set the prosthetic to a specific state after leavin demo mode
        void (*_demoCB) (uint32_t deltaTime, px_mode_t mode);

        WifiControl *_wifi;
        
        
};





#endif