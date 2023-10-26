#include <Arduino.h>
#include <TMCStepper.h>


#include "WifiControl.h"
#include "creds.h"
#include "ModeControl.h"


#include <ModbusIP_ESP8266.h>

#define PX_NUM 9
#define PX_REG 100 + (PX_NUM * 10)
#define PX_STATE_REG 110 + PX_NUM //lowest PX_NUM = 1 !!!!



enum {
  PX_ERR = -1,
  PX_OK
};

char ssid[] = SSID ;        // your network SSID (name)
char pass[] = PW;                    // your network password
WifiControl pxWifi(ssid, pass, PX_NUM);

#define EN_PIN           D1 // Enable
#define DIR_PIN          D4 // Direction
#define STEP_PIN         D2 // Step
#define CS_PIN           D8 // Chip select
#define FORWARD false
#define BACKWARD true

#define MAX_STEPS 65000
#define NUM_STATES 3


#define R_SENSE 0.11f 


TMC2130Stepper driver(CS_PIN, R_SENSE);                           


int32_t stepper_cur_position = 0;
int32_t stepper_goal_position = 0;
uint8_t step_pin_state = LOW;
void timer1_cb(void);

bool stepper_is_counting = false;

int32_t n_steps = 35000;

#define LIMIT_SW_PIN 3  //limit switch on GPIO3 aka RX
void IRAM_ATTR limit_sw_irq(void);
bool limit_sw_pressed = false;
bool count_error = false;
void reset_to_zero(void);
void go_to_zero(void);

void wifi_recon_func(void);


ModbusIP pxModbus;

#define DEMO_SW_PIN 16
int8_t demoState = 0;
void demoCallback(uint32_t dTime, px_mode_t mode);
ModeControl pxMC(DEMO_SW_PIN, &demoCallback, 15000, &pxWifi);

int8_t pxState = 0;
void setState(int8_t state);

int32_t stepper_pos[NUM_STATES];


void setup() {

  //Serial for debugging
  Serial.begin(57600, SERIAL_8N1, SERIAL_TX_ONLY);

  delay(3000);

  Serial.print(ESP.getResetReason());
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);      // Disable driver for now

                                  // Enable one according to your setup
  SPI.begin();                    // SPI drivers


  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(64);          // Set microsteps to 1/64th

  driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.pwm_autoscale(true);     // Needed for stealthChop



  timer1_attachInterrupt(timer1_cb);
  
  timer1_write(1666 / 2); //approx 3000Hz

  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);

  pxWifi.setPostConn(wifi_recon_func);
  pxWifi.setTimeOut(30000);
   
  if(digitalRead(DEMO_SW_PIN) == HIGH){//skip 1st wifi connection attempt if demo switch is on
    Serial.println("Connecting to C&C...");
    int8_t res = pxWifi.init();
    if(res == -1){
      Serial.println("No C&C found, starting up in demo mode!");
    }
  }

  pxModbus.server(502);
  pxModbus.addHreg(PX_REG, 0);
  pxModbus.addIreg(PX_REG, 0);
  pxModbus.addHreg(PX_STATE_REG, PX_OK);

  //ESP.wdtDisable();
  digitalWrite(EN_PIN, LOW); // enable stepper driver
  go_to_zero();

  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limit_sw_irq, FALLING);
  digitalWrite(EN_PIN, LOW);


  for(uint8_t i = 0; i < NUM_STATES; i++){
    stepper_pos[i] = i * (MAX_STEPS / (NUM_STATES - 1));
  }

  pxMC.init();

  Serial.println("Setup complete");
}

bool direction = false;
uint32_t microsTime = 0;
uint32_t steps = 0;

bool steps_printed = false;



void loop() {


  pxModbus.task();

  pxMC.run();

  pxWifi.run();

  
  if(pxModbus.Hreg(PX_REG) != pxModbus.Ireg(PX_REG)){
    pxModbus.Ireg(PX_REG, pxModbus.Hreg(PX_REG));
  }

  
  if(pxState != pxModbus.Ireg(PX_REG)){
    setState((int8_t)pxModbus.Ireg(PX_REG));
  }

  if(limit_sw_pressed || count_error){

    if(limit_sw_pressed){
      Serial.println("LIMIT SWITCH INTERRUPT TRIGGERED");
    }else if(count_error){
      Serial.println("COUNT ERROR");
    }
    reset_to_zero();
  }

  
}

void timer1_cb(){
  
  if(step_pin_state == LOW){
      if(direction == FORWARD){
        stepper_cur_position++;
        if(stepper_cur_position > MAX_STEPS){
            count_error = true;
            return; //exit function on detection of counter error
        }
      }else{
        stepper_cur_position--;
        if(stepper_cur_position < 0){
          count_error = true;
          return;//exit function on detection of count error
        }
      }

      digitalWrite(STEP_PIN, HIGH);
      step_pin_state = HIGH;

  }else{

    digitalWrite(STEP_PIN, LOW);
    step_pin_state = LOW;
    if(stepper_cur_position == n_steps ){
      Serial.println("stepper pos reached");
      if(timer1_enabled()){
        Serial.println("disabling timer1");
        timer1_disable();
      }
      stepper_is_counting = false;
    }
  }
}


void  IRAM_ATTR limit_sw_irq(void){
  //disable the stepper driver
  digitalWrite(EN_PIN, HIGH);
  timer1_disable();
  stepper_is_counting = false;
  limit_sw_pressed = true;

}

void reset_to_zero(void)
{
  detachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN));
  go_to_zero();
  //set system to zero state no matter the input
  demoState = 0;
  pxModbus.Hreg(PX_REG, 0);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN), limit_sw_irq, FALLING);
  limit_sw_pressed = false;
  count_error = false;
}


void go_to_zero(void)
{
  Serial.println("finding zero");
  if(digitalRead(LIMIT_SW_PIN) == LOW){
    Serial.println("LIMIT SWITCH PRESSED, MOVING FORWARD");
    //the device is already touching the limit switch
    //move forward until you no longer touch it
    direction = FORWARD;
    driver.shaft(direction);
    n_steps = 10000;
    stepper_cur_position = 0;
    //step forward until we flip the limit switch
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    stepper_is_counting = true;
    while(digitalRead(LIMIT_SW_PIN) == LOW){
      ESP.wdtFeed(); //feed the watchdog so we don't reset
    }
    //zero == found, stop moving after 100ms (debounce)
    delay(100);
    digitalWrite(EN_PIN, HIGH);
    timer1_disable();
    stepper_is_counting = false;
    stepper_cur_position = 0;
    Serial.println("switch opened, zero found");

  }else{
    Serial.println("LIMIT SWITCH NOT PRESSED, MOVING BACKWARD");
    direction = BACKWARD;
    driver.shaft(direction);
    n_steps = 0;
    stepper_cur_position = 100000;

    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    stepper_is_counting = true;

    //step backwards until we flip the switch
    while(digitalRead(LIMIT_SW_PIN) == HIGH){
      ESP.wdtFeed();//feed the watchdog so we don't reset
    }
    Serial.println("switch pressed, moving forward to mark zero");
    digitalWrite(EN_PIN, HIGH);
    timer1_disable();
    delay(1);
    direction = FORWARD;
    driver.shaft(direction);
    digitalWrite(EN_PIN, LOW);
    n_steps = 10000;
    stepper_cur_position = 0;
    //step forward until we flip the limit switch
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    stepper_is_counting = true;
    delay(1);
    Serial.println("waiting for switch to open up");
    while(digitalRead(LIMIT_SW_PIN) == LOW){
      ESP.wdtFeed();//feed the watchdog so we don't reset
    }
    //zero == found, stop moving after 100ms (debounce)
    delay(100);
    digitalWrite(EN_PIN, HIGH);
    timer1_disable();
    stepper_is_counting = false;
    stepper_cur_position = 0;
    Serial.println("switch opened after closing ,zero marked");
    
  }

    

}



void setState(int8_t state)
{
  if( state < NUM_STATES){
    int32_t goal_pos = stepper_pos[state];
    int32_t step_distance = goal_pos - stepper_cur_position;
    Serial.print("goal pos: ");
    Serial.println(goal_pos);
    if(step_distance > 0){
      Serial.println("stepping forwards");
        direction = FORWARD;
        driver.shaft(direction);
        n_steps = goal_pos;
        digitalWrite(EN_PIN, LOW);

          timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);

        stepper_is_counting = true;
    }else if(step_distance < 0){
      Serial.println("stepping backwards");
        direction = BACKWARD;
        driver.shaft(direction);
        n_steps = goal_pos;
        digitalWrite(EN_PIN, LOW);

          timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);

        stepper_is_counting = true;
    }
    pxState = state;
  }
}

void demoCallback(uint32_t dTime, px_mode_t mode)
{
  

 if(mode == PX_DEMO_MODE){
    if(demoState < NUM_STATES){
      demoState++;
      if(demoState < NUM_STATES - 1){
        pxModbus.Hreg(PX_REG, demoState);
      }else{
        pxModbus.Hreg(PX_REG, ((NUM_STATES - 1) * 2) - demoState);
      }
    }else{
      demoState = 0;
      pxModbus.Hreg(PX_REG, 0);
    }
  }else if(mode == PX_CC_MODE){
    Serial.println("to CC mode demo calback");
    
    demoState = 0;
    //force PX-9 to recalibrate upon reconnection!
    count_error = true;


  }
}

void wifi_recon_func(void)
{
  if(pxWifi.getStatus() == WL_CONNECTED){
    //after reconnecting to wifi some stuff has to happen with flash
      //if this fails because timer1 is firing, the ESP8266 crashes
      //so here timer1 gets disabled if it's running (no more stepping)
      //and 20 seconds of waiting to make sure we're good to continue
      if(timer1_enabled()){
        timer1_disable();
      }
      delay(20000);
  }
}
