#include "WifiControl.h"

wl_status_t connStatus = WL_DISCONNECTED;

WifiControl::WifiControl(char *ssid, char *pass, uint8_t pxn )
{

    _ssid = ssid;
    _pass = pass;
    //set last octet of ip address 100 + PX number
    _nodeAddr[3] = 100 + pxn;

    WiFi.config(_nodeAddr, _gate, _subn);
    WiFi.setAutoReconnect(false);
    wifi_set_event_handler_cb(handler);

    //these assignments are workarounds
    //the standard initialization used NULL which gave bugs
    //nullptr might offer the solution -> testing required!!!
    _preCon = emptyRecon;
    _postCon = emptyRecon;

    _reconCount = 0;

    

}

int8_t WifiControl::init()
{
    WiFi.begin(_ssid, _pass);
    _wifiCheckTime = millis();
    if(WiFi.waitForConnectResult(_t_out) != WL_CONNECTED){
        _reconCount++; //increment reconcount in case of failed first connection attempt
        return -1;   
    }else{
        return WL_CONNECTED;
    }
    
    
}

//this is the prefered method of changing the timeout for init and reconnection functionality over changing the #define in the header
//reconnection uses _t_out / 6 as a time out value
void WifiControl::setTimeOut(uint32_t timeout)
{
    _t_out = timeout;
}

void WifiControl::setPreConn(void (*preCon)(void))
{
    _preCon = preCon;
}
        
void WifiControl::setPostConn(void (*postCon)(void))
{
    _postCon = postCon;
}


//this function check for wifi connection and attempts to re-establish a connection after the timeout _wifiCheckTime expires after 60 seconds
//this function should not be used if the prosthetic cannot handle the mcu 'hanging' for quite some time while reconnecting
//for manual control of the reconnection timing use: WifiControl::reConn()
void WifiControl::run()
{
    //Serial.println("wifi run");
    if(getStatus() == WL_CONNECTED){
        _wifiCheckTime = millis();
    }
    if(getStatus() != WL_CONNECTED && millis() - _wifiCheckTime > 60000)
    {
        _wifiCheckTime = millis();
        if(_preCon != NULL)
        {

            _preCon();
        }
        _reconCount++;
        Serial.print("Wifi Reconnecting attempt: ");
        Serial.println( _reconCount);
        WiFi.disconnect();
        WiFi.begin(_ssid, _pass);
        WiFi.waitForConnectResult(_t_out / 2);
        if(_postCon != NULL)
        {
            _postCon();

        }
        if(getStatus() == WL_CONNECTED)
        {
            resetReconCount();
        }
        
    }
    
}

//call this function when the PX prosthetic can only reconnect under 'manual' control
//the user must manually reset _reconCount to 0 for other processes (like ModeControl) to see succesful reconnection
void WifiControl::reConn()
{
    
    if(getStatus() != WL_CONNECTED)
    {
        _wifiCheckTime = millis();
        if(_preCon != NULL)
        {

            _preCon();
        }
        _reconCount++;
        WiFi.disconnect();
        WiFi.begin(_ssid, _pass);
        WiFi.waitForConnectResult(_t_out / 2);
        if(_postCon != NULL)
        {
            _postCon();

        }
        if(getStatus() == WL_CONNECTED)
        {
            resetReconCount();
        }
        
    }

}

uint16_t WifiControl::getReconCount(){

    return _reconCount;
}

void WifiControl::resetReconCount(){
    _reconCount = 0;
}



wl_status_t WifiControl::getStatus()
{
    return connStatus;
}


void handler(System_Event_t *event){
    //connStatus = ;
    switch(event->event){
        case EVENT_STAMODE_CONNECTED:
            Serial.println("wifi handler: connected");
            connStatus = WL_CONNECTED;
            break;
        case EVENT_STAMODE_DISCONNECTED:
            Serial.println("wifi handler: disconnected");
            connStatus = WL_DISCONNECTED;
            break;
        case EVENT_STAMODE_AUTHMODE_CHANGE:
            Serial.println("wifi handler: authmode change");
            break;
        case EVENT_STAMODE_GOT_IP:
            Serial.println("wifi handler: got ip");
            connStatus = WL_CONNECTED;
            break;
        case EVENT_SOFTAPMODE_STACONNECTED:
            Serial.println("wifi handler: sta connected");
            break;
        case EVENT_SOFTAPMODE_STADISCONNECTED:
            Serial.println("wifi handler: sta disconnected");
            break;
        default:
            Serial.println("wifi handler called");
            break;

    }
    
}


void emptyRecon(){

}