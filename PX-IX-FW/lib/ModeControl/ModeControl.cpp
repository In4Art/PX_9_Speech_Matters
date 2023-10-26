#include "ModeControl.h"

ModeControl::ModeControl(uint8_t demoPin, void (*demoCB)(uint32_t deltaTime, px_mode_t mode), uint32_t interval, WifiControl *wifi)
{

    _demoPin = demoPin;
    pinMode(_demoPin, INPUT_PULLUP);

    _demoCB = demoCB;

    _interval = interval;

    _wifi = wifi;
    
}

px_mode_t ModeControl::init()
{

    wl_status_t status = _wifi->getStatus();

    if (status != WL_CONNECTED)
    {
        _mode = PX_DEMO_MODE;
        _oldTime = millis();
        return PX_DEMO_MODE;
    }
    else
    {
        return PX_CC_MODE;
    }
}

void ModeControl::run()
{
    _currTime = millis();
    //check for mode change on demopin
    //only when connected -> like that it is possible to enter demo mode when C&C is not found on startup
    //when it later connects the unit will switch to C&C mode if not manually held to demo mode
    if (digitalRead(_demoPin) == LOW && _wifi->getStatus() == WL_CONNECTED)
    {
        _mode = PX_DEMO_MODE;
    }
    else if (digitalRead(_demoPin) == HIGH) 
    {
        
        if(_wifi->getStatus() == WL_CONNECTED)
        {
            if(_mode == PX_DEMO_MODE)
            {
                uint32_t dtime = _currTime - _oldTime;
                //Serial.println("exiting demo mode, wifi (re)connected");
                _mode = PX_CC_MODE;
                _demoCB(dtime, _mode);
            }
        }else{
            if(_mode == PX_CC_MODE){
                uint32_t dtime = _currTime - _oldTime;
                _mode = PX_DEMO_MODE;
                _demoCB(dtime, PX_CC_MODE);
                _oldTime = _currTime;
                //_mode = PX_DEMO_MODE;
            }
        }
    }

    
    if (_mode == PX_DEMO_MODE)
    {   
        uint32_t dtime = _currTime - _oldTime;
        if (dtime > _interval)
        {
            Serial.println("calling cb from run");
            _demoCB(dtime, _mode); //call the calback with the actual ellapsed time in ms and the mode
            _oldTime = _currTime;
        }
    }
}

px_mode_t ModeControl::getControlMode()
{
    return _mode;
}

void ModeControl::setControlMode(px_mode_t mode)
{
    _mode = mode;
}