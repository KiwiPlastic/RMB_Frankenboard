#define VERSION 35
/* 22-3-25

RMB written By RICHRD NICHOLSON from NEW ZEALAND

Licensing

Non-commercial license is in effect. This license flows to derivative works.
You are free to remix / modify the blaster as you see fit. Please share.
If you post a remix, please link back to the original
Commercial use is allowed with the purchase of a RMB-Frankin-board at a ratio of one board = one commercial build

Attributions
(c) 2019-2021 Michael Ireland / Ireland Software / Airzone for example code used early on
For BLE Server examples
    Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara and MoThunderz

License
RMB-Frankin Board
by Team C.R.A.P is licensed under the Creative Commons - Attribution - Non Commercial Licence
https://creativecommons.org/licenses/by-nc/4.0/

Summary
  This code is for a Signal, or Two stage Brushless Nerf type Blaster. Using a solenoid pusher with closed loop feedback sensors
  It uses a three position switch for select fire
  Configuration is via a Android phone app

Hardware
 - Seeed XIAO ESP32C3 or Seeed XIAO ESP32S3 (S3 is recommended it faster)
 - Brushless 
 - Solenoid Pusher 
 - Single-Stage Trigger 
 - 2 x limit Switch's,  to make closed loop Solenoid = High ROF
 - Select fire, 3 position switch
 - Mofet PCB, for Solenoid
 - 2 x ESC
 - 1 x Buck voltage regulator

 - Android phone app (BLE Server) does not work on iPhone

Getting Started:
  This is an advanced project. 
  We recommend Flashing the ESP32 before doing anything else. This can save you money
  If you can not get this done you wont move forward.

  NOTE: You need a USB lead that supports data, NOT a phone charger lead which do not   transfer data and are common

  https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/

  Arduino IDE 2.2.1
  Select Board Manger, from menu on left of screen (2nd one down)
  look for esp32 by Espressif Systems, select install, could take a little while 
 
  Libary for ESC's:
  Select Libaray manager, from menu on left of screen (3rd one down)
  Type in ESP32, then scroll down and look for ESP32Servo By Kevin Harrington
  Alternative Method:
  https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite
  Download and install via Sketch/Include Libary/Add .Zip Libary

  Libary for PCF8575 Expander:
  Select Libaray manager, from menu on left of screen (3rd one down)
  Typ in Adafruit PCF8574 then select it 
  Alternative Method:
  https://github.com/adafruit/Adafruit_PCF8574
  Download and install via Sketch/Include Libary/Add .Zip Libary
  
*FLASH ESP32 - No WIRES connected
  It is highly recommended that you flash the firmware before you wire anything up to ensure you have a working chip
  This firmware has feature select options, please read past the libary files to see options

  While the chip has no wires on it, it will start and work with the phone app
  Bat Flat on the Serial Monitor (115200 baud) is RMB looking for the 2s/3s/4s battery on D0, Put 47k/10K on and a Bat and it will work
  The code works out what Type your battery is

*FLASH ESP32 WIRES connected
  Once wired with ESC etc... 
  Power from LiPo Bat, FIRST, then plug in USB to laptop
  The ESP32 will not start if the ESC are not powered up first, ie plugging in a USB first is a fail. Its a bootstrapping thing
  Ensure the MOSFET/Solenoid is disconnected. before Flashing firmware, it can activate and possibly make magic smoke

  Other Notes
  The Phone app has a demo mode switch so it can show the connected screen, when its not connected

  Mosfet (pusher output D6)  must be a 2-3v Gate. 5v ones will not work

  D8/D9 Bootstap pins must be high at Power Up/Boot
  We are using Boot strapping pins for ESC output,
  ESCs need to be powered up to get a boot if there signal wires are connected to ESP32. Else will look dead.
  Can be powered up with no ESC's if signal wires are not connected

  Two Build Options
    1 ESP32 C3 or S3, Basic no expander.  BLE interface only, Select Fire Sw, 2 Stage ESC, Bat_V, Pusher Limit Sw's, Trig, Opt Mag Sw, Opt RevSw or Tracer Dart not both
    2 Fully Featured using Expander. All of the basic but with OLed, Encoder, Rev_Sw, Tracer Dart, and more options
    code is configured to use Compile time switches, allowing options on what features you want

  BLE (Bluetooth Low Energy) this is different to the older Blutooth 2.0 serial and not compatableSupports Rev Ideal mode, set RevIdeal >90 for pre spin. Change Select fire Mode Switch to turn off RevIdeal


 
  //------------------ Dev Notes -----------------------------------------------------
   ESC HW
    ESC frq = 48Khz
    ESC min Trotel = 1040 (1 milisecond)
    ESC Max Trotel = 1960  (2 Millisecond)
    8 bit PWM resolution 0 = 254

   PWM Values for this code
   freq = 1000. 1kHz dont no why, but it works
    42 = off
    45 is lowest ideal speed, lower stalls
    84 is highest speed = ESC Max Trotel, can make number higher but it does  not go any faster

  //------------------------------------------------------------------------------------------------------------------------------------------------
*/
//================================================================================================================================
//Load libraries

#include <ESP32Servo.h>                  // ESC  libary by Kevin Harrington 
ESP32PWM ESC1pwm;                        // 
ESP32PWM ESC2pwm;                        // 

#include <BLEDevice.h>                   // Blue Tooth Libary
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Preferences.h>                 // ESP flash Storage Libary, seamed to be already installed, default
Preferences preferences;


//==================================================================================================================================================
//               DEFAULT SETTINGS - USER EDITABLE DEFAULT VALUES - Saved to Flash, as default blaster settings. Jold down trig on startup

//#define DEBUG_MODE            // Un-comment to test derect serial(USB) read of HW inputs, slows down and performance issues. leave as is 

#define BURST_SIZE 4            // Burst Mode Size #number of darts
#define MOTOR_ESC1_PWR 84       // Stage 1 : ESC1 Defualt Setting Range: 45-84 max
#define MOTOR_ESC2_PWR 84       // 42 = Stop, 45 = Slow idel Rev, 84 = Full Speed
#define MOTOR_REV_IDEAL 42      // 42 = Stop, 45 = Slow idel Rev, 84 = Full Speed
#define MOTOR_MIN_SPEED 45      // 42 = Stop, 45 = Slow idel Rev, 84 = Full Speed

#define BATTERY_CALFACTOR 1.0  // Adjustment for battery calibration calibration

#define MIN_DWELL_TIME 1        // NOT USED YET WIP
#define AMMO_COUNTER 18         // Mag Size, NOT USED YET WIP

//====================================================================================================================================================

//--------------------------------------------------------------------------------------------------------------------------------------
// Seeed ESP32 C3 or ESP32 S3 -  Pin Definitions (BIOS) NO Expander
//--------------------------------------------------------------------------------------------------------------------------------------

#define PIN_BATT_MON A0     // Bat Volts  A0/D0       IP           Analog LiPo battery reading, Resistor devideder 47K/10K
#define PIN_PUSHER_F D1     // Pusher Limit Sw Front  IP
#define PIN_PUSHER_R D2     // Pusher Limit Sw Rear   IP
#define PIN_TRIGGER D3      // Trigger                IP
#define PIN_SELF_A D4       // Select Fire A Sw       IP
#define PIN_SELF_B D5       // Select Fire B Sw       IP
#define PIN_RUN D6          // Pusher                 OP           D6 should be used as O/P only, to do with boot straping
//#define PIN_RUN D10          // Pusher                 OP        4 my one D6 broken   D6 should be used as O/P only, to do with boot straping
#define PIN_REV_SW D7       // RevSw   PB             IP
#define PIN_ESC_1 D8        // Stage 1 ESC output     OP           ESC Stage 1  D8 is a boot strap pin. must be high at startup
#define PIN_ESC_2 D9        // Stage 2 ESC output     OP           ESC Stage 2  D9 is a boot strap pin. must be high at startup
#define PIN_MAG_SW D10      // Mag switch             IP
//#define PIN_MAG_SW D6      // Mag switch             IP

//----------------------------------------------------------------------------------------
//Motors 
int MinMotorSpeed = MOTOR_MIN_SPEED;       //
int MaxMotorSpeed1 = MOTOR_ESC1_PWR ;      //
int MaxMotorSpeed2 = MOTOR_ESC2_PWR ;      //

// Battery Controls
#define BATTERY_2S_MIN 6.4                 //
#define BATTERY_3S_MIN 9.6                 //
#define BATTERY_4S_MIN 13.2                //
float BatteryCurrentVoltage = 99.0;        //
float BatteryOffset = BATTERY_CALFACTOR;   // asign to a floating point veriable so can be adjusted in config menu (work in progress)
float BatteryMinVoltage = BATTERY_2S_MIN;  // this is just to load the float with a value
bool BatteryFlat = false;

// ISR Flags
bool ISR_PUSHER_F_Flag = false;            // Interrupt triggered flag
bool ISR_PUSHER_R_Flag = false;            // Interrupt triggered flag
bool ISR_TRIGGER_Flag = false;             // Interrupt triggered flag
bool ISR_SELF_A_Flag = true;               // Interrupt triggered flag
bool ISR_SELF_B_Flag = true;               // Interrupt triggered flag
bool ISR_REV_SW_Flag = false;              // Interrupt triggered flag
bool ISR_MAG_SW_Flag = false;              // Interrupt triggered flag

// System Modes
#define SYSTEM_MODE_NORMAL 0               // Ideal/ok to Fire
#define SYSTEM_MODE_LOWBATT 1              // All stop
#define SYSTEM_MODE_FLYWHEELFAIL 2         // not used wip
byte SystemMode = SYSTEM_MODE_NORMAL;      //

// Firing Controls
#define DPS 50                             // not used wip
#define ONESEC 1000                        // One second dah

//Selectfire 
#define AUTO 0
#define BURST 1
#define SINGLE 2
byte CurrentFireMode = SINGLE;
volatile bool SF_Changed_Flag = false;    // Select Fire Changed
volatile bool MotorRunningFlag = false;   // 

// Inputs
#define DebounceWindow 5                  // Debounce Window = 5ms   **** CHK
#define RepollInterval 250                // Just check the buttons ever xxx just in case we missed an interrupt. ***** CHK

// Input Button Status, For ISR output
#define BTN_LOW 0
#define BTN_HIGH 1
#define BTN_ROSE 2
#define BTN_FELL 3

// main internal registers/veriables
byte TriggerButtonState = BTN_HIGH;
byte BLETriggerButtonState = false;
byte BLE_SF_PB_State = false;
byte BLERev_PB_State = false;
byte BurstSize = BURST_SIZE;
byte ESC1_Pwr = MOTOR_ESC1_PWR;
byte ESC2_Pwr = MOTOR_ESC2_PWR;
byte RevIdeal = MOTOR_REV_IDEAL;
float DPS_Value = 0;                        // calculated value by firing ctrl repported by nofify
byte CurrentShot_Value = 0;

// Map Flash Memory loads defaults on start up, if not exist
unsigned int burstsize = BURST_SIZE;        //
unsigned int esc1 = MOTOR_ESC1_PWR;
unsigned int esc2 = MOTOR_ESC2_PWR;
unsigned int revideal = MOTOR_REV_IDEAL;

volatile bool DefaultFlag = false;          // Default flash stroage values
volatile bool PusherTickTock = false;       // To capture the first edge of each pusher in / out cycle

volatile bool PusherFront = false;
volatile bool PusherRear = false;

int stat_LED = 0;                           // status of LED: 1 = ON, 0 = OFF
unsigned long LastBlinkLED;                 // start time in milliseconds


//================================== BLE Server Config ==================================
// Some General Veriables

bool deviceConnected = false;              //
bool oldDeviceConnected = false;           //
unsigned long BLE_TimeConnected = 0;       //
bool NotifyFlag = false;                   // Used by RMB to send Notify to App
char BLEValue[2];                          // used by Callbacks to read characteristics values

uint32_t BatVoltsValue = 0;                //
uint32_t BLE_SF_ModeValue = 0;             //
uint32_t AmmoCounterValue = AMMO_COUNTER;  // <<<< chnage this to get working
uint32_t BurstSizeValue = BurstSize;       //
uint32_t ESC1Value = ESC1_Pwr;             //
uint32_t ESC2Value = ESC2_Pwr;             //
uint32_t RevIdealValue = RevIdeal;         //
uint32_t TriggerValue = 0;                 //

volatile byte LastBatVolts = 99;           //
volatile byte LastSF_Mode = 99;            //
volatile byte LastAmmoCounter = 99;        //
volatile byte LastBurstSize = 99;          //
volatile byte LastESC1 = 99;               //
volatile byte LastESC2 = 99;               //
volatile byte LastRevIdeal = 99;           //
volatile byte BLE_LastCurrentShot = 99;    //
volatile byte LastDPS = 99;                //       possible delete???
volatile byte LastBatteryFlat = 99;        //
volatile byte BLE_LastRev_PB = 99;         //
volatile byte BLE_Last_SF_PB = 99;         //

//------------------------------- BLE Characteristic Labels ------------------------------

BLEServer* pServer = 0;                 // Server Advertising
BLECharacteristic* pStatus_Update = 0;  // Notify Status Update
BLECharacteristic* pTrigger = 0;        // R/W Trigger PB
BLECharacteristic* pSF_PB = 0;          // R/W SF PB value
BLECharacteristic* pRev = 0;            // R/W Rev PB
BLECharacteristic* pBurst_Size = 0;     // R/W Burst Size via slider
BLECharacteristic* pESC1 = 0;           // R/W ESC1 Pwr via slider
BLECharacteristic* pESC2 = 0;           // R/W ESC2 pwr via slider
BLECharacteristic* pRevIdeal = 0;       // R/W Rev Ideal Spd via slider

BLEDescriptor* pDescr;
BLE2902* pBLE2902;

//------------------------ BLE Characteristics UUID --------------------------------------
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

static BLEUUID BLESERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");  // Severice advertisment
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"              //
#define STATUS_UPDATE_UUID "38ed8a4d-f45d-4323-956a-759072589315"        // Status Update Notifiyer
#define FIRE_PB_UUID "b377ef36-7247-46fa-aca3-d21da530c782"              // Trigge Push Button R/W
#define SF_PB__UUID "9c3d379d-28e7-4974-a563-04d7c0b24a51"               // SelectFire setting R/W
#define REV_PB_UUID "738faa41-e3ee-418d-b370-e3ba613dc99b"               // Rev Push button R/W
#define BURST_UUID "e3223119-9445-4e96-a4a1-85358c4046a2"                // Burst Size Slider setting R/W
#define ESC1_UUID "7460002d-70cc-4ed0-98b4-1e778842e64e"                 // ESC1 Slider setting R/W
#define ESC2_UUID "0c188d05-26f6-4548-9325-c541ee31e856"                 // ESC2 Slider setting R/W
#define REVIDEAL_UUID "e8c8936e-fc18-4bb7-8004-73ad956a37eb"             // Rev Idel Mode Slider setting R/W

//---------------------- BLE Server Callbacks (from phone client App) --------------------

// --------------- BLE Connect/Disconnect ------------------------------------------------
class ESP32ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    BLE_TimeConnected = millis();
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

//-------------------- BLE Characteristic CallBack - Trigger PB -------------------------------
class CharacteristicTrigger : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    String pChar_value_stdstr = String(pChar->getValue());
    String pChar_value_string = String(pChar_value_stdstr.c_str());   // convert to auduino string format
    BLEValue[0] = (pChar_value_string[0]);                            // BLEValue[] is general use
    if (strcmp(BLEValue, "T") == 0) {                                 // Test for trigger,if true ...
      BLEValue[0] = 0;                                                // Reset value to random number
      if (BLETriggerButtonState == false) {                           // flip flop the trigger input
        BLETriggerButtonState = true;
      } else {
        BLETriggerButtonState = false;
      }
      Serial.println("BLE Trigger = " + String(BLETriggerButtonState)); // Debug
    }
  }
};

//---------------------- BLE Characteristic CallBack - SF_PB) -------------------------
class CharacteristicSF_PB : public BLECharacteristicCallbacks {      // Select fire via PB in app
  void onWrite(BLECharacteristic* pChar) override {                  //
    String pChar_value_stdstr = String(pChar->getValue());           //
    String pChar_value_string = String(pChar_value_stdstr.c_str());  // convert to auduino string format
    BLEValue[0] = (pChar_value_string[0]);                           // BLEValue[] is general use

    if (strcmp(BLEValue, "S") == 0) {                                // Test for Single Fire,if true ...
      BLEValue[0] = 0;                                               // Reset value to random number
      BLE_SF_PB_State = SINGLE;                                      // set Select fire value
    }
    if (strcmp(BLEValue, "B") == 0) {                                // Test for Burst,if  true ...
      BLEValue[0] = 0;                                               // Reset value to random number
      BLE_SF_PB_State = BURST;                                       //
    }
    if (strcmp(BLEValue, "A") == 0) {                                // Test for Auto,if true ...
      BLEValue[0] = 0;                                               // Reset value to random number
      BLE_SF_PB_State = AUTO;                                        //
    }
  }
};

//-------------------- BLE Characteristic CallBack - Rev PB -------------------------------
class CharacteristicRevPB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    String pChar_value_stdstr = String(pChar->getValue());
    String pChar_value_string = String(pChar_value_stdstr.c_str());  // convert to auduino string format
    BLEValue[0] = (pChar_value_string[0]);                           // BLEValue[] is general use
    if (strcmp(BLEValue, "R") == 0) {                                // Test for Rev PB,if true ...
      BLEValue[0] = 0;                                               // Reset value to random number
      if (BLERev_PB_State == false) {                                // flip flop the Rev input
        BLERev_PB_State = true;
      } else {
        BLERev_PB_State = false;
      }
      //Serial.println("Rev_PB = " + String(BLERev_PB_State));       // Debug
    }
  }
};

//---------------------- BLE Characteristic CallBack - Burst Size -------------------------
class CharacteristicBurstSize : public BLECharacteristicCallbacks {  // Burst Size via slider in app
  void onWrite(BLECharacteristic* pChar) override {                  //
    String pChar_value_stdstr = String(pChar->getValue());
    String pChar_value_string = String(pChar_value_stdstr.c_str());  // convert to auduino string format
    int pChar_value_int = pChar_value_string.toInt();                // convert string to interger
    Serial.println("Burst Size: " + String(pChar_value_int));      // Debug: displays slider value as interger
    BurstSize = pChar_value_int;                                     // set Burst Size
    burstsize = BurstSize;                                           // copy to flash variable
    preferences.putUInt("burstsize", burstsize);                     // save value to Flash
  }
};

//---------------------- BLE Characteristic CallBack - ESC1 -------------------------
class CharacteristicESC1 : public BLECharacteristicCallbacks {       // ESC1 Pwr via slider in app
  void onWrite(BLECharacteristic* pChar) override {                  //
    String pChar_value_stdstr = String(pChar->getValue());
    String pChar_value_string = String(pChar_value_stdstr.c_str());  // convert to auduino string format
    int pChar_value_int = pChar_value_string.toInt();                // convert string to interger
    Serial.println("ESC1: " + String(pChar_value_int));            // Debug: displays slider value as interger
    ESC1_Pwr = pChar_value_int;                                      // save value to ESC1 Power
    MaxMotorSpeed1 = ESC1_Pwr;
    esc1 = ESC1_Pwr;
    preferences.putUInt("esc1", esc1);                               // save value to Flash
  }
};

//---------------------- BLE Characteristic CallBack - ESC2 -------------------------
class CharacteristicESC2 : public BLECharacteristicCallbacks {       // ESC2 Pwr via slider in app
  void onWrite(BLECharacteristic* pChar) override {         
    String pChar_value_stdstr = String(pChar->getValue());           //
    String pChar_value_string = String(pChar_value_stdstr.c_str());  // convert to auduino string format
    int pChar_value_int = pChar_value_string.toInt();                // convert string to interger
    Serial.println("ESC2: " + String(pChar_value_int));            // Debug: displays slider value as interger
    ESC2_Pwr = pChar_value_int;
    MaxMotorSpeed2 = ESC2_Pwr;                                       // save value to ESC2 Power
    esc2 = ESC2_Pwr;
    preferences.putUInt("esc2", esc2);                               // save value to Flash
  }
};

//---------------------- BLE Characteristic CallBack - Rev Ideal -------------------------
class CharacteristicRevIdeal : public BLECharacteristicCallbacks {   //Rev Ideal via slider in app
  void onWrite(BLECharacteristic* pChar) override {      
    String pChar_value_stdstr = String(pChar->getValue());           //
    String pChar_value_string = String(pChar_value_stdstr.c_str());  // convert to auduino string format
    int pChar_value_int = pChar_value_string.toInt();                // convert string to interger
    Serial.println("RevSpd: " + String(pChar_value_int));          // Debug: displays slider value as interger
    RevIdeal = pChar_value_int;                                      // save value to Ideal Rev Spd
    MinMotorSpeed = RevIdeal;
    revideal = RevIdeal;
    preferences.putUInt("revideal", revideal);                       // save value to Flash
  }
};

//============================================================================================================
void setup() {
  unsigned long BootStart = millis();                               // Just for any initial timing..

  // ----------------------------------- Outout --------------------------------------------------------------
  pinMode(PIN_RUN, OUTPUT);                                         // Pusher activates on Boot, so turn off ASAP
  digitalWrite(PIN_RUN, LOW);                                       // Set Pusher OFF
  
  //pinMode(LED_BUILTIN, OUTPUT);                                   // Config Built in Led as an output ESPC3 does not have   

  LastBlinkLED = millis();

  //----------------------------------------------------------------------------------------------------------
  // Allow allocation of all PWM timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  Serial.begin(115200);                       // Needs time to start
 
  //-------------------------------------------------------------------------------------------------------------
  ESC1pwm.attachPin(PIN_ESC_1, 1000, 10);     // 1KHz 8 bit
  ESC2pwm.attachPin(PIN_ESC_2, 1000, 10);     // 1KHz 8 bit
  delay (2000);                               // wait improtant... for startup tones....

  ESC1pwm.writeScaled(0);                     // Set min output to ESC = 0.000
  ESC2pwm.writeScaled(0);                     // Set min output to ESC = 0.000
  delay (2000);                               // wait important....

  ESC1pwm.writeScaled(0.040);                 // set min pwm to esc, makes startup tone
  ESC2pwm.writeScaled(0.040);   
  delay (2000);

  //-------------------------------------------------------------------------------------------------------------
  analogReadResolution(12);                                         // Set analog input resolution to max, 12-bits
  pinMode(PIN_BATT_MON, INPUT);                                     // Analog Battery Monitor Input

  pinMode(PIN_PUSHER_F, INPUT_PULLUP);                              // Pusher Limit Switch Front
  attachInterrupt(digitalPinToInterrupt(PIN_PUSHER_F), ISR_PUSHER_F, FALLING);

  pinMode(PIN_PUSHER_R, INPUT_PULLUP);                              // Pusher Limit Switch Rear
  attachInterrupt(digitalPinToInterrupt(PIN_PUSHER_R), ISR_PUSHER_R, FALLING);

  pinMode(PIN_TRIGGER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TRIGGER), ISR_TRIGGER, FALLING);

  pinMode(PIN_SELF_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SELF_A), ISR_SELF_A, CHANGE);

  pinMode(PIN_SELF_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SELF_B), ISR_SELF_B, CHANGE);

  pinMode(PIN_REV_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REV_SW), ISR_REV_SW, CHANGE);

  pinMode(PIN_MAG_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_MAG_SW), ISR_MAG_SW, CHANGE);

  //------------------------BLE SEVER startup --------------------------------------------
  // Create the BLE Device
  BLEDevice::init("RMB-Frankenboard");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ESP32ServerCallbacks());                  // BLE Sevre Callback

  // Create the BLE Service
  BLEService* pService = pServer->createService(BLESERVICE_UUID, 30, 0);  // the 30 defines the number of chars to be used, increase as needed

  // Create a BLE Characteristic
  pStatus_Update = pService->createCharacteristic(
    STATUS_UPDATE_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  pTrigger = pService->createCharacteristic(
    FIRE_PB_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pBurst_Size = pService->createCharacteristic(
    BURST_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pESC1 = pService->createCharacteristic(
    ESC1_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pESC2 = pService->createCharacteristic(
    ESC2_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pSF_PB = pService->createCharacteristic(
    SF_PB__UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pRevIdeal = pService->createCharacteristic(
    REVIDEAL_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pRev = pService->createCharacteristic(
    REV_PB_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  // Create a BLE Descriptor
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("A very interesting variable");
  pStatus_Update->addDescriptor(pDescr);

  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);

  // Add all Descriptors here
  pStatus_Update->addDescriptor(pBLE2902);
  pTrigger->addDescriptor(new BLE2902());
  pBurst_Size->addDescriptor(new BLE2902());
  pESC1->addDescriptor(new BLE2902());
  pESC2->addDescriptor(new BLE2902());
  pSF_PB->addDescriptor(new BLE2902());
  pRevIdeal->addDescriptor(new BLE2902());
  pRev->addDescriptor(new BLE2902());

  // add Callbacks
  pTrigger->setCallbacks(new CharacteristicTrigger());          // Tigger PB from Client
  pSF_PB->setCallbacks(new CharacteristicSF_PB());                    // SF PB From Client
  pRev->setCallbacks(new CharacteristicRevPB());                      // Rev PB from Client
  pBurst_Size->setCallbacks(new CharacteristicBurstSize());     // Burst Count Slider value
  pESC1->setCallbacks(new CharacteristicESC1());                // ESC1 Slider value
  pESC2->setCallbacks(new CharacteristicESC2());                // ESC2 Slider value
  pRevIdeal->setCallbacks(new CharacteristicRevIdeal());        // RevSpd Slider value

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);                            // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("BLE Waiting...");

  //---------------------------------------------------------------------------------------
  Serial.print(F("Serial Started...Ver: "));                     // Show version on USB serial on BOOT
  Serial.println(VERSION);

  //---------------------------------------------------------------------------------------
  // Pre-charge the battery indicator and Wait for the sync - 10 seconds
  while (BatteryCurrentVoltage >= 99.0)  
  {
    ProcessDebouncing();                                          // Read trigger for Defaulting Flash Storage
    ProcessBatteryMonitor();                                      // Check battery voltage occasionally
    
    delay(10);                                                    // slow it down a bit
    if ( (TriggerButtonState == BTN_LOW) )                        // If the trigger was held on power-on for a reset set Default Flag
      DefaultFlag = true;
  }

 //-------------------------------------------------------------------------------------------------------------------
  // Config Flash Stroarge
  preferences.begin("rmb-app", false);                          // config Flash for R/W. True = Read only.
  
  // Load default vales to flash if Default Flag set, Trigger held on start up
  if (DefaultFlag == true){                                     // If Defaults flag load Defaults values to Flash
    preferences.putUInt("burstsize", BURST_SIZE);               // save value to Flash
    preferences.putUInt("esc1", MOTOR_ESC1_PWR);                // save value to Flash
    preferences.putUInt("esc2", MOTOR_ESC2_PWR);                // save value to Flash
    preferences.putUInt("revideal",  MOTOR_REV_IDEAL);          // save value to Flash
    DefaultFlag = false;                                        // Reste flag
  }

  // Read Flash Storage, if no values (new chip) use default values
  burstsize = preferences.getUInt("burstsize", BURST_SIZE);     // Read Flash Storage
  esc1 = preferences.getUInt("esc1", MOTOR_ESC1_PWR);
  esc2 = preferences.getUInt("esc2", MOTOR_ESC2_PWR);
  revideal = preferences.getUInt("revideal", MOTOR_REV_IDEAL);

  BurstSize = burstsize;
  BurstSizeValue = BurstSize;

  ESC1_Pwr = esc1;
  ESC1Value = ESC1_Pwr;
  MaxMotorSpeed1 = ESC1_Pwr;

  ESC2_Pwr = esc2;
  ESC2Value = ESC2_Pwr;
  MaxMotorSpeed2 = ESC2_Pwr;

  RevIdeal = revideal;
  RevIdealValue = RevIdeal;
  MinMotorSpeed = RevIdeal;

  //----------------------------------------------------------------------------------------
  // Calculate battery Min from bat types
  if (BatteryCurrentVoltage >= BATTERY_4S_MIN){
    BatteryMinVoltage = BATTERY_4S_MIN;
     Serial.println("Bat Type: 4S");
    BatteryFlat = false;
  } else if (BatteryCurrentVoltage >= BATTERY_3S_MIN) {
    BatteryMinVoltage = BATTERY_3S_MIN;
    Serial.println("Bat Type: 3S");
    BatteryFlat = false;
  } else {
    BatteryMinVoltage = BATTERY_2S_MIN;
    Serial.println("Bat Type: 2S");
    BatteryFlat = false;
  }

  //------------------------------------------------------------------------------------------
  // Ready Go...
  Serial.println("Bat Min = " + String(BatteryMinVoltage));    
  Serial.println("Bat Volts = " + String(BatteryCurrentVoltage));    
  Serial.println("F_BS = " + String(burstsize));                    // Show Flash settinngs on Serial 
  Serial.println("F_ESC1 = " + String(esc1));
  Serial.println("F_ESC2 = " + String(esc2));
  Serial.println("F_RevI = " + String(revideal));
  
  Serial.println(F("READY..."));
}
//------------------------------------------------------ END SETUP ----------------------------------------------------------------

//======================================== Interupt Servce Routines ==============================================+=================

//--------------------------------------------------
void ISR_PUSHER_F() {
  if (!PusherTickTock)      // Switch the tick-tock
    PusherTickTock = true;
    PusherFront = true;
}

//--------------------------------------------------
void ISR_PUSHER_R() {
  if (PusherTickTock)       // Switch the tick-tock
    PusherTickTock = false;
    PusherRear = true;
}

//--------------------------------------------------
void ISR_TRIGGER() {
  ISR_TRIGGER_Flag = true;  //  Set a flag if an interrupt is triggered
}

//--------------------------------------------------
void ISR_SELF_A() {
  ISR_SELF_A_Flag = true;  //  Set a flag if an interrupt is triggered
}

//--------------------------------------------------
void ISR_SELF_B() {
  ISR_SELF_B_Flag = true;  //  Set a flag if an interrupt is triggered
}

//---------------------------------------------------
void ISR_REV_SW() {
  ISR_REV_SW_Flag = true;  // Set a flag if interrupt is triggered
}
//--------------------------------------------------
void ISR_MAG_SW() {
  ISR_MAG_SW_Flag = true;  //  Set a flag if an interrupt is triggered
}

//================================ MAIN LOOP =====================================================================                    
void loop() {

#ifdef DEBUG_MODE                          // Displays raw I/O on serial
  ProcessDebug();                          // This buggers up timing for normal runtime firing code, we dont process other stuff when this is on
#else                                      //
  ProcessDebouncing();                     // Process the Trigger input, and handle any debouncing
  ProcessBatteryMonitor();                 // Check battery voltage occasionally
  ProcessSystemMode();                     // Handle the system mode (Low battery chk)
  ProcessBLESelectFire();                  // Chk Selectfire
  ProcessSelectFire();                     // Chk Selectfire
  ProcessMagSwitch();                      // process Mag switch
  ProcessBLERevSwitch();                   // process Rev switch from BLE
  ProcessRevSwitch();                      // Chk Rev and Mag switch inputs 
  ProcessFiring();                         // Handle any firing here
  ProcessBLE();                            // BLE Server
  //ProcessBlinkLED();                        // one sec hart beat
#endif
}

//===========================================================================================================
//------------------------------------ Process Debounce - Trigger  -------------------
void ProcessDebouncing() {
  unsigned long CurrentMillis = millis();           // Single call to millis() for better performance
  static bool RunTriggerTimer = false;
  static unsigned long LastTriggerPress = 0;
  static byte TriggerDebounceState = 0;
  byte ButtonState = 0;

  // Set up a repoll interval, just in case the interrupt is missed.
  if (CurrentMillis - LastTriggerPress > RepollInterval) ISR_TRIGGER_Flag = true;
  // Move from edge to steady state
  if (TriggerButtonState == BTN_ROSE) TriggerButtonState = BTN_HIGH;
  if (TriggerButtonState == BTN_FELL) TriggerButtonState = BTN_LOW;

  if (ISR_TRIGGER_Flag) {
    ISR_TRIGGER_Flag = false;
    if (!RunTriggerTimer) {
      LastTriggerPress = CurrentMillis;

      TriggerDebounceState = 0b01010101;  // We need to give the button time to settle. This will track.
      RunTriggerTimer = true;
    }
  }
  if (RunTriggerTimer && (CurrentMillis - LastTriggerPress > DebounceWindow)) {
    //TriggerDebounceState = (TriggerDebounceState << 1) | ((PIND >> 6) & 0b00000001); // Shift the register pin to the left, Shift the pin result to the right most position, and tack it onto the debounce state. Ensure that only the last position can be a 1.
    TriggerDebounceState = 255;
    if ((TriggerDebounceState == 0) || (TriggerDebounceState == 255))  // All 0's or all 1's. This means we have settled.
    {
      RunTriggerTimer = false;
      ButtonState = digitalRead(PIN_TRIGGER);
      if (ButtonState) {
        if (TriggerButtonState != BTN_HIGH) TriggerButtonState = BTN_ROSE;
      } else {
        if (TriggerButtonState != BTN_LOW) TriggerButtonState = BTN_FELL;
      }
    }
  }
}

//------------------------------------ Process Battery Monitor -------------------------------------------------
void ProcessBatteryMonitor() {
#define NUM_SAMPLES 8
  static byte CollectedSamples = 0;
  static float SampleAverage = 0;
  uint32_t Vbatt = 0;

  if ((SystemMode == SYSTEM_MODE_NORMAL) || (SystemMode == SYSTEM_MODE_LOWBATT))  // Only read if system mode is normal or low batery
  {
    if (CollectedSamples < NUM_SAMPLES) {
      CollectedSamples++;
      SampleAverage += analogReadMilliVolts(PIN_BATT_MON);  // Read ADC with correction
    } else {
      //BatteryCurrentVoltage = 6 * SampleAverage / 8 / 1000.0;  // attenuation ratio 1/6, mV --> v
      BatteryCurrentVoltage = ((6 * SampleAverage / 8 / 1000.0) - BatteryOffset); // attenuation ratio 1/6, mV --> v
      BatVoltsValue = int(BatteryCurrentVoltage * 10);         //convert to int for BLE value

      if ((BatVoltsValue > (LastBatVolts + 3)) || (BatVoltsValue < (LastBatVolts - 3)))  //Provide hysteresis
      {
        //BatVoltsValue = BatVoltsValue;                                                //update, dumy as already loaded above
      } else {
        BatVoltsValue = LastBatVolts;  // dont update
      }

      if (BatteryCurrentVoltage < BatteryMinVoltage) {
        if (BatteryCurrentVoltage > 1.6)        // If the current voltage is 0, we are probably debugging
        {
          BatteryFlat = true;
          Serial.println("BAT FLAT V = " + String(BatteryCurrentVoltage));
          Serial.println("BatMin = " + String(BatteryMinVoltage));                          //Debug
        } else {
          BatteryFlat = false;
        }
      } else {
        BatteryFlat = false;
      }
      CollectedSamples = 0;
      SampleAverage = 0;
    }
  }
}

//-------------------------------- Process System Mode ---------------------------------------------------------
void ProcessSystemMode() {

  static byte LastSystemMode = 99;
  //static unsigned long LastFlywheelJam = 0;

  if (BatteryFlat) {
    SystemMode = SYSTEM_MODE_LOWBATT;
  } else {
    SystemMode = SYSTEM_MODE_NORMAL;
  }
  LastSystemMode = SystemMode;

  // Optic Int Readout for testing PCB 
  if (SystemMode == SYSTEM_MODE_NORMAL) {
    if (PusherRear == true){
      byte pr= digitalRead(PIN_PUSHER_R);
      Serial.println ("Pusher Rear = " + String (pr));
      PusherRear = false;
    }
    if (PusherFront == true){
      byte pf = digitalRead(PIN_PUSHER_F);
      Serial.println ("Pusher Front = " + String (pf));
      PusherFront = false;
    }
  }
}

//----------------------------------------- Process SelectFire ---------------------------------------------------
void ProcessBLESelectFire() {
  if (BLE_Last_SF_PB != BLE_SF_PB_State) {
    if (BLE_SF_PB_State == BURST) {
      CurrentFireMode = BURST;
      BLE_SF_ModeValue = BURST;  // BLE notify BURST
      Serial.println("Burst Mode");
    } else if (BLE_SF_PB_State == SINGLE) {
      CurrentFireMode = SINGLE;
      BLE_SF_ModeValue = SINGLE;  // BLE notify SINGLE shot
      Serial.println("Single Shot");
    } else {
      CurrentFireMode = AUTO;
      BLE_SF_ModeValue = AUTO;  // BLE notify AUTO
      Serial.println("Full Auto");
    }
    SF_Changed_Flag = true;  //indicate SF has changed to stop RevIdeal
    BLE_Last_SF_PB = BLE_SF_PB_State;
    //Serial.println("SF PB = " + String(BLE_SF_PB_State)); // Debug
  }
  if ((SF_Changed_Flag == true) && (RevIdeal >= MOTOR_MIN_SPEED) && (MotorRunningFlag == true)) {  // turn off motors if RevIdeal > 42
    MinMotorSpeed = MOTOR_REV_IDEAL;
    SF_Changed_Flag = false;
    MotorRunningFlag = false;
    StopMotors();
  }
  SF_Changed_Flag = false;
}

//----------------------------------------- Process SelectFire ---------------------------------------------------
void ProcessSelectFire(){
  if (ISR_SELF_A_Flag || ISR_SELF_B_Flag)       // if ISR flag set read pins
  {
    byte SFA = digitalRead(PIN_SELF_A);
    byte SFB = digitalRead(PIN_SELF_B);

    ISR_SELF_A_Flag = false;
    ISR_SELF_B_Flag = false;

    if (SFA == HIGH && SFB == HIGH) {
      CurrentFireMode = BURST;
      BLE_SF_ModeValue = BURST;                     // BLE notify BURST
      Serial.println("Burst Mode");
    } else if (SFA == HIGH && SFB == LOW) {
      CurrentFireMode = SINGLE;
      BLE_SF_ModeValue = SINGLE;                    // BLE notify SINGLE shot
      Serial.println("Single Shot");
    } else {
      CurrentFireMode = AUTO;
      BLE_SF_ModeValue = AUTO;                      // BLE notify AUTO
      Serial.println("Full Auto");
    }
    SF_Changed_Flag = true;                     //indicate SF has changed to stop RevIdeal
  }
  if ((SF_Changed_Flag == true) && (RevIdeal >= MOTOR_MIN_SPEED) && (MotorRunningFlag == true)){     // turn off motors if RevIdeal > 42
    MinMotorSpeed = MOTOR_REV_IDEAL;
    SF_Changed_Flag = false;
    MotorRunningFlag = false;
    StopMotors();
  }
  SF_Changed_Flag = false;
}

//------------------------- Process Mag Switch Input - WIP ------------------------------------------------------
void ProcessMagSwitch() {
  static byte LastMagSwButtonState = 99;

  if (ISR_MAG_SW_Flag) {
    byte MagSwButtonState = digitalRead(PIN_MAG_SW);      // Read Mag Switch
    if (MagSwButtonState != LastMagSwButtonState) {
      Serial.println("MagSw = " + String(MagSwButtonState));
      //Put handeler code here
      LastMagSwButtonState = MagSwButtonState;
    }
    ISR_MAG_SW_Flag = false;
  }
}

//------------------------- Process BLE Rev Switch  ------------------------------------------------------
void ProcessBLERevSwitch() {
  if (SystemMode == SYSTEM_MODE_NORMAL) {
    if (BLE_LastRev_PB != BLERev_PB_State) {
      Serial.println("Rev_PB = " + String(BLERev_PB_State));  // Debug
      if (BLERev_PB_State == false) {                         // off, not pressed, Stop motors
        MinMotorSpeed = MOTOR_REV_IDEAL;                      // stop
        StopMotors();
      }
      if (BLERev_PB_State == true) {  // on, pressed, start motors at Rev Ideal
        if (RevIdeal >= MOTOR_MIN_SPEED) {
          byte Temp1 = MaxMotorSpeed1;  // save value
          MinMotorSpeed = RevIdeal;
          MaxMotorSpeed1 = RevIdeal;
          StartMotors();
          MaxMotorSpeed1 = Temp1;  // restore values
        }
      }
      BLE_LastRev_PB = BLERev_PB_State;
    }
  }
}

//------------------------- Process Rev Switch Input ------------------------------------------------------
void ProcessRevSwitch() {
  static byte LastRevSwButtonState = 99;
  if (RevIdeal != LastRevIdeal) {
    if (RevIdeal > MaxMotorSpeed1) {
      MaxMotorSpeed1 = RevIdeal;
      MaxMotorSpeed2 = RevIdeal;
      ESC1Value = MaxMotorSpeed1;
      ESC2Value = MaxMotorSpeed2;
    }
  }
  if (ISR_REV_SW_Flag) {                    // If int flag
    byte RevSwButtonState = digitalRead(PIN_REV_SW);          // Read Rev Switch
    if (SystemMode == SYSTEM_MODE_NORMAL) {
      if (RevSwButtonState != LastRevSwButtonState) {         // Test if Rev button changed
        Serial.println("RevSw = " + String(RevSwButtonState));      // show status
        if (RevSwButtonState == BTN_HIGH) {                    // off, not pressed, Stop motors
          MinMotorSpeed = MOTOR_REV_IDEAL;                    // stop
          StopMotors();
          Serial.println("STOP REV");
        }
        if (RevSwButtonState == BTN_LOW) {                     // on, pressed, start motors at Rev Ideal
          if (RevIdeal >= MOTOR_MIN_SPEED) {
            byte Temp1 = MaxMotorSpeed1;                      // save values
            byte Temp2 = MaxMotorSpeed2;
            MinMotorSpeed = RevIdeal;
            MaxMotorSpeed1 = RevIdeal;
            MaxMotorSpeed2 = RevIdeal;
            StartMotors();
            Serial.println("START REV");
            MaxMotorSpeed1 = Temp1;                           // restore values
            MaxMotorSpeed2 = Temp2;
          }
        }
        LastRevSwButtonState = RevSwButtonState;
      }
      ISR_REV_SW_Flag = false;
    }
  }
}

//---------------------------------- Process Firing control logic ----------------------------------------------------
void ProcessFiring() {
  bool StartFiringCycle = false;                                          //
  bool Trigger_On_Flag = false;                                           // Physical or BLE

  static unsigned long RevStart = 0;                                      //
  unsigned long CurrentMillis = millis();                                 // Single call to millis() for better performance

  if (TriggerButtonState == BTN_FELL)                                    // Check to see if the trigger fell
    StartFiringCycle = true;

  if ((BLETriggerButtonState == true) && (StartFiringCycle == false))     // Check if the BLE trigger is true
    StartFiringCycle = true;

  // Only fire in normal mode
  if (StartFiringCycle && (SystemMode != SYSTEM_MODE_NORMAL)) {           // return if not Normal Mode
    return;
  }

  if (StartFiringCycle) {                                                 // Start Fireing sequence - we have a Trigger
    if (RevIdeal >= MOTOR_MIN_SPEED)                                                   // pre set minMotor to RevIdeal if >90
      MinMotorSpeed = RevIdeal;
    RevStart = millis();
    StartMotors();                                                        // Start motors
    while (millis() - RevStart < 200)                                     // ************nobal this line if rev ideal sw **************
    {
      delay(1);
    }

    unsigned long CurrentShot = 0;
    //float DPS = 0;
    unsigned long DPSStart = millis();
    do                                                                      // Firing loop, time critical, dont fuck with it
    {                                                                       //
      FireSolenoid();                                                       //
      ProcessDebouncing();                                                  // get TriggerButtonState == BTN_LOW (Active), BTN_HIGH (released)
      CurrentShot++;                                                        //
      if (TriggerButtonState == BTN_LOW || BLETriggerButtonState == true)  // chk triggers
        Trigger_On_Flag = true;                                             // if trigger set flag
      else                                                                  //
        Trigger_On_Flag = false;                                            // Reset trigger flag
    } while ((CurrentFireMode == AUTO && Trigger_On_Flag) || (CurrentFireMode == BURST && CurrentShot < BurstSize && Trigger_On_Flag));     // LOOP

    CurrentMillis = millis();                                               // update current millis
    
    //DPS_Value = int(CurrentMillis - DPSStart);                              // caclcute time Taken
    //DPS_Value = ((CurrentShot / DPS_Value) * 1000);                         // Divid Number of shots by Time Taken and multi floating point by 1000 millis = DPS
    //Serial.println("DPS_Value = " + String(DPS_Value));                     //
    
    CurrentShot_Value = CurrentShot;                                // this will be the BLE load
    Serial.println("Darts Fired = " + String(CurrentShot));

    StopMotors();
  }
}

// ------------------------------------ Solenoid Firing Sequence -------------------------------------------------
void FireSolenoid() {
  unsigned long StartThrow = millis();
  unsigned long StartTimer = millis();
  bool Failed = false;

  digitalWrite(PIN_RUN, HIGH);                // Turn On Solenoid

  StartTimer = millis();                      // Wait for the sensor to report an extended pusher
  Failed = true;
  while (millis() - StartTimer < 200) {
    if (PusherTickTock)                       // Solenoid Tick Tock
    {
      Failed = false;
      break;
    }
  }

  digitalWrite(PIN_RUN, LOW);                 // Turn Off Solenoid

  StartTimer = millis();                      // Wait for the sensor to report an extended pusher
  Failed = true;
  while (millis() - StartTimer < 200) {
    if (!PusherTickTock) {
      Failed = false;
      break;
    }
  }

  // Wait for the calculated dwell time for DPS adjustment
  unsigned long ThrowTime = millis() - StartThrow;
  unsigned long BufferTime = 1000 / DPS;
  if (BufferTime < ThrowTime)
    BufferTime = 0;
  else
    BufferTime = BufferTime - ThrowTime;
  unsigned long WaitTime = (BufferTime);
  //Serial.println(WaitTime);
  //unsigned long WaitTime = (max( BufferTime, MIN_DWELL_TIME ));
  unsigned long StartWait = millis();
  while ((millis() - StartWait) < WaitTime)
    ;
}

//------------------------------------- Start Motors --------------------------------------------------------
void StartMotors() {

  float PWMValue1 = (0.001 * MaxMotorSpeed1);      // convert to float
  float PWMValue2 = (0.001 * MaxMotorSpeed2);      // convert to float
  ESC1pwm.writeScaled(PWMValue1);                  // start motor
  ESC2pwm.writeScaled(PWMValue2);                  // start motor
  MotorRunningFlag = true;                         // For RevIdeal Flag

  //Serial.println(MaxMotorSpeed1);
  //Serial.println(MaxMotorSpeed2);
  //Serial.println(PWMValue1,3);
  //Serial.println(PWMValue2,3);
}

//---------------------------------------Stop Motors ------------------------------------------------------------
void StopMotors() {

  float PWMValue1 = (0.001 * MinMotorSpeed);      // convert to float
  float PWMValue2 = (0.001 * MinMotorSpeed);      // convert to float
  ESC1pwm.writeScaled(PWMValue1);                 // stop motors
  ESC2pwm.writeScaled(PWMValue2);

  //Serial.println("Motor OFF");
}

//---------------------- Process BLE, send value to phone app -----------------------------------------------
void ProcessBLE() {

  unsigned long CurrentMillis = millis();  // Single call to millis()
  int BLE_NotifyFlag = 0;               // process one value per cycle

  if (NotifyFlag == true)  // If NotifiyFlag, BLE is UP
  {
    BLE_NotifyFlag = false;  // Notify works best at 1 value per scan. Set buffer empty

    if ((LastBatVolts != BatVoltsValue) && (BLE_NotifyFlag == false))  // if Bat volts changed
    {
      String pChar_value_string = ("[BV" + String(BatVoltsValue) + "]");
      LastBatVolts = BatVoltsValue;
      //Serial.println("BLE Bat volts : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((LastSF_Mode != BLE_SF_ModeValue) && (BLE_NotifyFlag == false))  // If Select Fire Mode Chnaged
    {
      String pChar_value_string = ("[SF" + String(BLE_SF_ModeValue) + "]");
      LastSF_Mode = BLE_SF_ModeValue;
      //Serial.println("BLE SEll Fire : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((LastAmmoCounter != AmmoCounterValue) && (BLE_NotifyFlag == false))  // If Select Fire Mode Chnaged
    {
      String pChar_value_string = ("[AC" + String(AmmoCounterValue) + "]");
      LastAmmoCounter = AmmoCounterValue;
      //Serial.println("BLE AmmoCount : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((LastBurstSize != BurstSizeValue) && (BLE_NotifyFlag == false))  // If Burst size changed
    {
      String pChar_value_string = ("[BS" + String(BurstSizeValue) + "]");
      LastBurstSize = BurstSizeValue;
      //Serial.println("BLE BurstSize : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((LastESC1 != ESC1Value) && (BLE_NotifyFlag == false))  // If ESC1 pwr changed
    {
      String pChar_value_string = ("[E1" + String(ESC1Value) + "]");
      LastESC1 = ESC1Value;
      //Serial.println("BLE ESC1 : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((LastESC2 != ESC2Value) && (BLE_NotifyFlag == false))  // If ESC1 pwr changed
    {
      String pChar_value_string = ("[E2" + String(ESC2Value) + "]");
      LastESC2 = ESC2Value;
      //Serial.println("BLE ESC2 : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((LastRevIdeal != RevIdealValue) && (BLE_NotifyFlag == false))  // If ESC1 pwr changed
    {
      String pChar_value_string = ("[RI" + String(RevIdealValue) + "]");
      LastRevIdeal = RevIdealValue;
      //Serial.println("BLE RevIdeal : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((LastDPS != int(DPS_Value)) && (BLE_NotifyFlag == false))  // If DPS changed
    {
      String pChar_value_string = ("[DPS" + String(DPS_Value) + "]");
      LastDPS = int(DPS_Value);
      //Serial.println("BLE DPS : " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if ((BatteryFlat != LastBatteryFlat) && (BLE_NotifyFlag == false))  // Low Battery Alarm
    {
      String pChar_value_string = ("[LB" + String(BatteryFlat) + "]");
      LastBatteryFlat = BatteryFlat;
      //Serial.println("BLE Low Bat = " + String(pChar_value_string));

      pStatus_Update->setValue(pChar_value_string.c_str());
      pStatus_Update->notify();
      BLE_NotifyFlag = true;
    }

    if (BLE_NotifyFlag = true) {  // provide small delay between values
      delay(50);                  //
    }                             //
  }                               // end if device connected (Notify Flag)

  // Disconnecting
  if (!deviceConnected && oldDeviceConnected) {  // If disconnect
    delay(500);                                  // give the bluetooth stack the chance to get things ready
    NotifyFlag = false;                          // Reset Notify flag

    BurstSizeValue = BurstSize;  // Save values else on reconnect will be wrong (This is Not Flash memory)
    LastBurstSize = BurstSize;   //

    ESC1Value = ESC1_Pwr;
    LastESC1 = ESC1_Pwr;

    ESC2Value = ESC2_Pwr;
    LastESC2 = ESC2_Pwr;

    RevIdealValue = RevIdeal;
    LastRevIdeal = RevIdeal;

    pServer->startAdvertising();  // restart advertising
    Serial.println("********** Start Advertising *****************");
    oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected)  // do stuff here on connecting - preload IO values
  {
    if (CurrentMillis > (BLE_TimeConnected + ONESEC))  // provide a startup delay
    {
      Serial.println("Connected");
      oldDeviceConnected = deviceConnected;  // deviceConnected flag = true or flase

      LastBatVolts = 99;
      LastSF_Mode = 99;
      LastAmmoCounter = 99;
      LastBurstSize = 99;
      LastESC1 = 99;
      LastESC2 = 99;
      LastRevIdeal = 99;
      LastBatteryFlat = 99;  // reset notifiy compare values

      NotifyFlag = true;  // allow Notifys to start
    }
  }
}
//------------------------------------------- End BLE Server ----------------------------------------------------

//------------------------------------------------
// Blink LED l
void ProcessBlinkLED() {
  if ((millis() - LastBlinkLED) > ONESEC) {
    stat_LED = !stat_LED;
    LastBlinkLED = millis();
  }
  //digitalWrite(LED_BUILTIN, stat_LED);              //flip flop on/off indicate alive
}

//================= Process Debug - Display Raw Pin Status =============================================================
void ProcessDebug() {
  #define BIOS                                                                              // comment out to disable BIOS output
  #ifdef BIOS                                                                               //
    Serial.println("============= ESP32 BIOS =============");                               //
    Serial.println("A0  IP Battery Raw = " + String ((float)analogRead(PIN_BATT_MON)));     //
    Serial.println("D1  IP Pusher Front Limit = " + String (digitalRead(PIN_PUSHER_F)));    //
    Serial.println("D2  IP Pusher Rear Limit = " + String (digitalRead(PIN_PUSHER_R)));     //
    Serial.println("D3  IP Trigger = " + String (digitalRead(PIN_TRIGGER)));                //
    #ifndef EN_PCF8575_EXP                                                                  // No Expander
      Serial.println("D4  IP SF A = " + String (digitalRead(PIN_SELF_A)));                  // Selectfire A
      Serial.println("D5  IP SF B = " + String (digitalRead(PIN_SELF_B)));                  // Select Fire B
    #else                                                                                   // With Expander PCF8575
      Serial.println("D4  IO I2C SDA");                                                     // I2C SDA
      Serial.println("D5  IO I2C SCL");                                                     // I2C SCL
    #endif                                                                                  //
    Serial.println("D6  OP Pusher = " + String (digitalRead(PIN_RUN)));                     //
    #ifdef EN_PCF8575_EXP                                                                   //
      Serial.println("D7  IP PCF8575 int = " + String (digitalRead(PIN_PCF8575_INT)));      // IO Expander INT    
    #else                                                                                   // NO EXP
      Serial.println("D7  IP Rev Sw = " + String (digitalRead(PIN_REV_SW)));                // Rev Switch
    #endif                                                                                  //
    Serial.println("D8  OP ESC 1 = " + String (digitalRead(PIN_ESC_1)));                    // ESC 1
    Serial.println("D9  OP ESC 2 = " + String (digitalRead(PIN_ESC_2)));                    // ESC 2
    Serial.println("D10 IP MAG SW = " + String (digitalRead(PIN_MAG_SW)));                  // Mag Switch
    #ifdef EN_PCF8575_EXP                                                                   // If EXPANDER
      Serial.println("PCF P00 En PB SW = " + String (pcf.digitalRead(PIN_ENCODER_SW)));     // Encoder PB SW
      Serial.println("PCF P01 IP En CLK = " + String (pcf.digitalRead(PIN_ENCODER_CLK)));   // Encoder CLK
      Serial.println("PCF P02 IP En DT = " + String ((pcf.digitalRead(PIN_ENCODER_DT)));    // Encoder DT
      Serial.println("PCF P03 IP Rev SW = " + String (pcf.digitalRead(PIN_REV_SW)));        // Rev Switch
      Serial.println("PCF P04 IP SF A = " + String (pcf.digitalRead(PIN_SELF_A)));          // Selectfire A
      Serial.println("PCF P05 IP SF B = " + String (pcf.digitalRead(PIN_SELF_B)));          // Select Fire B
      Serial.println("PCF P10 OP Tracer dart = " + String (pcf.digitalRead(PIN_TRACER_D)));  // Tracer Dart
    #endif
    Serial.println("======================================");
  #endif

  Serial.println("BatteryCurrentVoltage = " + String ( BatteryCurrentVoltage));
  Serial.println("BatMin = " + String(BatteryMinVoltage));                          //Debug
  Serial.println("PusherTickTock = " + String (PusherTickTock));

  Serial.println("F_BS = " + String(burstsize));
  Serial.println("F_ESC1 = " + String(esc1));
  Serial.println("F_ESC2 = " + String(esc2));
  Serial.println("F_RevI = " + String(revideal));

  //Serial.println ("BurstSize = " + String (BurstSize));
  //Serial.println("ESC1_Pwr = " + String(ESC1_Pwr));
  //Serial.println("ESC2_Pwr = " + String(ESC2_Pwr));
  //Serial.println("RevIdel = " + String(RevIdeal));

  Serial.println("BLEValue = " + String (BLEValue));
  Serial.println("BurstSize = " + String (BurstSize));
  Serial.println("ESC1_Pwr = " + String(ESC1_Pwr));
  Serial.println("ESC2_Pwr = " + String(ESC2_Pwr));
  Serial.println("RevIdel = " + String(RevIdeal));
  Serial.println();

  delay(500);
}
