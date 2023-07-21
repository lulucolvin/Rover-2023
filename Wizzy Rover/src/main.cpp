/*
  ACBR RGB One

  Controls the rover used for the HSU Foundation & Be The Magic Foundation Summer Class 2021 (Wizzy Rover).

  Hx: 
  05.24.21 DJP - Started coding
  05.25.21 DJP - Added headlight functionality
  05.29.21 DJP - Added obstacle avoidance LiDar sensors
  06.01.21 DJP - Added topside leds for better visibility of rover status in sunlight
  07.07.21 DJP - Changed "star" button on controller to show a red & blue back & forth fade
  07.07.21 DJP - Tweaked lightbar light sensitivity due to new location of the photoresistor
  07.07.21 DJP - Tweaked LiDar sensitivity due to new location of the LiDar sensors
  07.29.21 DJP - Incorporated Doug & Sarah's in class changes
  08.03.21 DJP - Moved code from the OnNotify() to the Loop() to resolve BT response issues between the PS3 and the ESP32
  08.04.21 DJP - Changed ReadLiDarSensors() to not clear the ground effect leds between reads when ground effect is on
  07.22.22 DJP - Changed SetupLiDarSensort() to not entire a never ending while loop when a LiDar sensor fails to initialize
  07.23.22 DJP - Added toggle option to turn on/off LiDar via PS3 L1 & R1 combination
  08.03.22 DJP - Added ground effect lighting
  08.15.22 DJP - Added Bit-Wizards demo mode, by setting pin 34 high the ground effect lighting and FLB strobe blue & yellow
                    keeping pin 34 low keeps the rover in normal mode (non-Demo mode)...for testing, I used a jumper between pin 34
                    and pin 0 to turn on demo mode, leaving 34 empty or using a pin already in low state (IE pin 2) will take it out 
                    of demo mode...this can be done on the fly
  07.20.23 DJP - Changed debug led functionality as follows:
                  1) The blue led on the hat of the Wizzy controller board will blink 3 times if not BT connection and will otherwise
                     stay on (blue)
                  2) The white eyes on the face of the Wizzy controller board are the debug leds and will blink accordingly depending 
                     on the issue detected
  07.20.23 DJP - Changed front & rear lightbars to function as one lightbar
  07.20.23 DJP - Added ground effect lightbars as part of the standard rover startup
*/

//confirm the com is correct in platform.ini file then open a new terminal window, key in the following and press enter: pio run --target upload

//run sixaxispair.exe to determine the address of the PS3 controller, the default address is 01:02:03:04:05:06.  The default will work as long as
// no other ps3 wireless controllers are nearby and turned on.  SixAxisPair will allow you to change the address if so desired.

//common pio commands: 
//  pio device list
//  pio run --target clean
//  pio run --target upload
//  pio run --target upload --upload-port com5
//
// MAKE CERTAIN YOUR TERMINAL WINDOW IS IN THE CORRECT DIRECTORY...ie C:\Users\perkins\Documents\PlatformIO\Projects\ACBR RGB One
//     if not, use the cd command via the PS prompt to change

#include <Arduino.h>            // Arduino Framework
#include <Adafruit_NeoPixel.h>
#include <Tone32.h>
#include <Ps3Controller.h>
#include <esp32-hal-ledc.h>
#include "Adafruit_VL53L0X.h"


#ifndef __ESP32__
#define __ESP32__
#endif

#define NUMPIXELS             4     // We have 4 LEDs on the board
#define NUM_PIXELS_ON_LB      6     // We have 6 LEDs on the front lightbar (FLB)
#define NUM_PIXELS_ON_RLB     6     // We have 6 LEDs on the rear lightbar (RLB)
#define NUM_PIXELS_ON_GELB    12    // 6 on each LB

#define PIN_FLB_SWITCH        14    // Flips the transistor switch on/off (high is on, closes the circuit)
#define PIN_PIXELS_RLB        15    // Wizzy Lightbar (rear lightbar) signal/DIN pin...this one will be available
#define PIN_PIXELS            18    // Our LEDs are hardwired to pin 18
#define PIN_BUZZER            19    // The buzzer is hardwired to pin 19
#define PIN_PIXELS_GELB       23    // Ground effect LB's
#define PIN_XSHUT_REAR_LOX    25    // shutdown pin for rear facing LOX
#define PIN_BT_CONNECTED_LED  26    // Blue LED indicated BT connection established
#define PIN_PIXELS_FLB        27    // Lightbar signal/DIN pin...this will control both front & rear LB
#define PIN_DEBUG_LED         32    // Turns on/off the white debug led 
#define PIN_XSHUT_FRONT_LOX   33    // shutdown pin for front facing LOX
#define PIN_DEMO_MODE         34    // when set high, causes the code to run in BW demo mode
#define PIN_PHOTORESISTOR     35    // Reads the photoresistor analog value

enum Color { UNKNOWN, RED, WHITE, BLUE };
Color _isRedWhiteOrBlue = RED;
Color _nextColor = UNKNOWN;

//arduino IDE allows this, but PlatformIO fails...both allow the uint8_t
//#define FRONT_FACING_LOX_I2C_ADDR 0x30; //the I2C address for the front facing lox
//#define REAR_FACING_LOX_I2C_ADDR  0x31; //the I2C address for the rear facing lox
uint8_t FRONT_FACING_LOX_I2C_ADDR = 0x30; //the I2C address for the front facing lox
uint8_t REAR_FACING_LOX_I2C_ADDR = 0x31; //the I2C address for the rear facing lox

/******   RESERVED PINS -- DO NOT USE   ******/
int _inOne = 13; //These are the input pins on our chip that we're
int _inTwo = 12; //connecting to our ESP32 through our circuit board!
int _inThree = 16;
int _inFour = 17;
int _rightX = 0;
int _rightY = 0;
int _leftX = 0;
int _leftY = 0;

int _redValue = 0;

int _color = 1;

//use SixAxisPairTool to set a custom mac address...the address below is the default that ships on the device
//char _ps3MacAddr[20] = { "01:02:03:04:05:06" };
//char _ps3MacAddr[20] = { "21:02:03:04:05:02"}; //2021 class
char _ps3MacAddr[20] = {"22:07:24:04:05:16"}; //2022 class

uint8_t _lbBrightness = 128;
const uint8_t MAX_LB_BRIGHTNESS = 255;
const int LOOPS_BETWEEN_BLINKS = 25;

bool _isFrontObstacleDetected = false;
bool _isRearObstacleDetected = false;
bool _isMovingForward = false;
bool _isMovingBackward = false;
bool _areHeadlightsManuallyOn = false;
bool _areHeadlightsOn = false;
bool _isStrobeOn = false;
bool _isGroundEffectOn = false;
bool _isRearLBOn = false;
bool _areBuiltInsOn = false;
bool _didCircleChange = false;
bool _didTriangleChange = false;
bool _didCrossChange = false;
bool _didSquareChange = false;
bool _isTestingLEDsOnly = false;
bool _useLiDar = false;
bool _isFrontLidarOn = false;
bool _isRearLidarOn = false;
bool _didL1Change = false;
bool _didR1Change = false;
bool _didL2Change = false;
bool _isPS3_L1_Pressed = false;
bool _isPS3_R1_Pressed = false;
bool _isPS3_L2_Pressed = false;
bool _showGroundEffect = false;
bool _keepFLBDemoStrobeOn = false;
int _loopsBetweenBlinks = 0;


// Set up some variables for the light level: a calibration value and and a raw light value
int _lightCal;
int _lightVal;

Adafruit_NeoPixel _builtInLEDs(NUMPIXELS, PIN_PIXELS, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel _frontLightbar(NUM_PIXELS_ON_LB, PIN_PIXELS_FLB, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel _rearLightbar(NUM_PIXELS_ON_RLB, PIN_PIXELS_RLB, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel _groundEffectLB(NUM_PIXELS_ON_GELB, PIN_PIXELS_GELB, NEO_GRB + NEO_KHZ800);

Adafruit_VL53L0X _frontLox = Adafruit_VL53L0X();
Adafruit_VL53L0X _rearLox = Adafruit_VL53L0X();
// this holds the lidar measurement 
VL53L0X_RangingMeasurementData_t _front_LOX_Measure;
VL53L0X_RangingMeasurementData_t _rear_LOX_Measure;

bool IsRunningInDemoMode();
void BlinkDebugLED(int BlinkXTimes);
void FlashFrontLightbar(bool FlashRandomly = false);                  //flashes the leds running across the lightbar
void FlashRearLightbar(bool FlashRandomly = false);                   //flashes the leds running across the lightbar
void FlashBuiltInLEDs();                                              //flashes the bottom leds blue if Bluetooth is connected, otherwise red
void FlashBuiltInLEDsForDebug(uint8_t R, uint8_t G, uint8_t B);       //flashes the bottom leds 
void OnNotify();
void OnConnect();
void SetupLidarSensors();
void ReadLidarSensors();
void BlinkBuiltInLEDs(uint8_t R = 0, uint8_t G = 255, uint8_t B = 0);
void TurnOnFrontLightbar(bool Manually = false);
void LightTheGE(uint8_t R, uint8_t G, uint8_t B);
void ChaserGE(uint8_t R, uint8_t G, uint8_t B, bool RandomTrailTaper = false);
void StrobeLightbar();
void StrobeLightbarDemoMode();
void LightTheFrontBar(uint8_t R = 255, uint8_t G = 255, uint8_t B = 255);  //sets the lightbar (all leds) to the passed in color
void LightTheRearBar(uint8_t R = 255, uint8_t G = 255, uint8_t B = 255);  //sets the rear lightbar (all leds) to the passed in color
void TurnBuiltInsOn();
void TurnBuiltInsOff();
/*if TurnAllOn is false then FlasRandomly is looked at, if that is false then the ground effect will strobe*/
void ToggleGroundEffect(uint8_t R = 255, uint8_t G = 255, uint8_t B = 255, bool TurnAllOn = false, bool FlashRandomly = false);
void ToggleGroundEffectDemoMode(bool TurnAllOn = false, bool FlashRandomly = false);
void setup_dbg();
void loop_dbg();
//void TurnOnHeadlights(bool Manually = false);

//NOTE: May want to change this logic so that it only reads the front sensor if the rover is moving forward or the user is attempting to move
//        forward.  Likewise, only read the rear sensor if the rover is in reverse or the user is attempting to move in reverse.
//- add a donut feature, tie it to one of the buttons on the controller.  When activated, stop all motors, then spin them up in opposite directions at 255
//- need to be able to speed the rover up by determining how far forward/aft the joystick is
//- code up left & right joystick movement (the x axis)

void setup_debug()
{
  // Setup the motors
  //ledcSetup(1, 30000, 8); //we set up PWM channel 1, frequency of 30,000 Hz, 8 bit resolution
  //ledcAttachPin(_inOne,1); //we're going to attach inOne to our new PWM channel
  //ledcSetup(2, 30000, 8); //we'll set up the rest of our PWM channels, just like before.
  //ledcAttachPin(_inTwo,2); //this time we'll need to set up 8 PWM channels!
  //ledcSetup(3, 30000, 8);
  //ledcAttachPin(_inThree,3);
  //ledcSetup(4, 30000, 8);
  //ledcAttachPin(_inFour,4);
}

void setup_LEDs()
{
  pinMode(PIN_PIXELS_RLB, OUTPUT);
  pinMode(PIN_FLB_SWITCH, OUTPUT);

  delay(250);

  _rearLightbar.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _rearLightbar.setBrightness(MAX_LB_BRIGHTNESS); // Full brightness

  delay(250);

  //FlashRearLightbar();
  
  _rearLightbar.clear();
  _rearLightbar.show();
  digitalWrite(PIN_FLB_SWITCH, HIGH);

  delay(250);
  digitalWrite(PIN_FLB_SWITCH, LOW);

  LightTheRearBar(0, 0, 255);
  _rearLightbar.show();
  digitalWrite(PIN_FLB_SWITCH, HIGH);

  _isTestingLEDsOnly = true;
}

void setup()
{
  pinMode(PIN_PIXELS_GELB, OUTPUT);
  digitalWrite(PIN_PIXELS_GELB, LOW);

  _groundEffectLB.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _groundEffectLB.setBrightness(MAX_LB_BRIGHTNESS); // Full brightness

  delay(250);

  LightTheGE(0, 0, 255);
  delay(1000);
}

void loop()
{
  uint8_t red = (uint8_t)0; //random(256);
  uint8_t white = (uint8_t)0; //random(256);
  uint8_t blue = (uint8_t)255; //random(256);
  
  switch (_isRedWhiteOrBlue)
  {
    case RED: 
    {
      red = (uint8_t)255;  
      white = 0;  
      blue = 0;  
      _nextColor = WHITE; 
      break;
    }
    case WHITE: 
    {
      red = (uint8_t)255;  
      white = (uint8_t)255;  
      blue = (uint8_t)255;  
      _nextColor = BLUE; 
      break;
    }
    default: 
    {
      red = 0;  
      white = 0;  
      blue = (uint8_t)255;  
      _nextColor = RED; 
      break;
    }
  }

  ChaserGE(red,white,blue);

  _isRedWhiteOrBlue = _nextColor;
}

//NOTE:  if the LiDar sensor is acting weird, verify the protective file (yellow or orange) has been removed from the face of the LiDar sensor
void setup1() 
{
  pinMode(PIN_DEMO_MODE, INPUT_PULLDOWN);

  pinMode(PIN_DEBUG_LED, OUTPUT);
  pinMode(PIN_BT_CONNECTED_LED, OUTPUT);
  digitalWrite(PIN_DEBUG_LED, HIGH);
  digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
  delay(500);

  _isRedWhiteOrBlue = RED;
  _nextColor = WHITE;

  //setupGE();

  // Setup the motors
  ledcSetup(1, 30000, 8); //we set up PWM channel 1, frequency of 30,000 Hz, 8 bit resolution
  ledcAttachPin(_inOne,1); //we're going to attach inOne to our new PWM channel
  ledcSetup(2, 30000, 8); //we'll set up the rest of our PWM channels, just like before.
  ledcAttachPin(_inTwo,2); //this time we'll need to set up 8 PWM channels!
  ledcSetup(3, 30000, 8);
  ledcAttachPin(_inThree,3);
  ledcSetup(4, 30000, 8);
  ledcAttachPin(_inFour,4);

  digitalWrite(PIN_BT_CONNECTED_LED, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_PHOTORESISTOR, INPUT);

  pinMode(PIN_XSHUT_FRONT_LOX, OUTPUT);
  pinMode(PIN_XSHUT_REAR_LOX, OUTPUT);

  pinMode(PIN_FLB_SWITCH, OUTPUT);
  pinMode(PIN_PIXELS_RLB, OUTPUT);
  digitalWrite(PIN_PIXELS_RLB, LOW);

  pinMode(PIN_PIXELS_GELB, OUTPUT);
  digitalWrite(PIN_PIXELS_GELB, LOW);
 
  BlinkDebugLED(1);

  digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
  delay(250);
  digitalWrite(PIN_BT_CONNECTED_LED, LOW);
  delay(1);

  _builtInLEDs.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _builtInLEDs.setBrightness(255); // Full brightness

  delay(250);

  _frontLightbar.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _frontLightbar.setBrightness(MAX_LB_BRIGHTNESS); // Full brightness

  delay(250);

  _rearLightbar.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _rearLightbar.setBrightness(MAX_LB_BRIGHTNESS); // Full brightness

  delay(250);

  _groundEffectLB.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _groundEffectLB.setBrightness(MAX_LB_BRIGHTNESS); // Full brightness

  delay(250);

  FlashFrontLightbar();
  delay(250);
  FlashRearLightbar();
  //StrobeLightbar();

  //flash the red, white & blue ground effect
  FlashBuiltInLEDsForDebug(255, 0, 0);
  delay(125);
  FlashBuiltInLEDsForDebug(255, 255, 255);
  delay(125);
  FlashBuiltInLEDsForDebug(0, 0, 255);
  delay(125);
  _builtInLEDs.clear();
  _builtInLEDs.show();
  //Turn on the rear bar...used for testing
  //_rearLightbar.clear();
  //LightTheRearBar();
  //_earLightbar.show();

  _lbBrightness = 128;
  _frontLightbar.setBrightness(_lbBrightness); 

  //Serial.begin(115200);
  
  Ps3.attach(OnNotify);
  Ps3.attachOnConnect(OnConnect);
  Ps3.begin(_ps3MacAddr);

  //FlashBuiltInLEDs();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) 
  _lightCal = 300;   // 725 is a good value for normal ambient light, 300 to simulate lights out (IE finger over the sensor)
#endif
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) 
  _lightCal = 300;   // 725 is a good value for normal ambient light, 300 to simulate lights out (IE finger over the sensor)
#endif
#if defined(__ESP32__)
  _lightCal = 1600;
#endif

  ToggleGroundEffect(255, 255, 255, true, false); //toggle ground effect on
  delay(500);
  ToggleGroundEffect(); //toggle ground effect off

  Serial.print("Using light calibration of: ");
  Serial.println(_lightCal, DEC);

  BlinkDebugLED(5);

  SetupLidarSensors();

  delay(1000);
  BlinkDebugLED(3);

  //digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
  //digitalWrite(PIN_BT_CONNECTED_LED, LOW);

  _loopsBetweenBlinks = (LOOPS_BETWEEN_BLINKS + 1);

  Serial.println("ESP32 Startup");
}

//this loop() is used for testing...might want to tie this into a button on the PS3
void loop_debug() 
{
  digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
  digitalWrite(PIN_DEBUG_LED, HIGH);
  delay(2000);

  digitalWrite(PIN_DEBUG_LED, LOW);
  delay(500);

  digitalWrite(PIN_BT_CONNECTED_LED, LOW);
  digitalWrite(PIN_DEBUG_LED, HIGH);
  delay(500);
}

void loop1()
{
  if ( _loopsBetweenBlinks > LOOPS_BETWEEN_BLINKS )
  {
    _loopsBetweenBlinks = 0;
    BlinkDebugLED(1);
  }
  else
  {
    _loopsBetweenBlinks++;
  }

  if ( !Ps3.isConnected() )
  {
    for ( int j = 0; j < 3; j++ )
    {
      digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
      delay(100);
      digitalWrite(PIN_BT_CONNECTED_LED, LOW);
      delay(100);
    }

    for ( int j = 0; j < 3; j++ )
    {
      //flash the leds blue, but in a circular pattern
      _builtInLEDs.clear();
      _builtInLEDs.show();
      delay(100);
      _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 255));
      _builtInLEDs.show();
      delay(100);
      _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 0));
      _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 255));
      _builtInLEDs.show();
      delay(100);
      _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 0));
      _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 255));
      _builtInLEDs.show();
      delay(100);
      _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 0));
      _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 255));
      _builtInLEDs.show();
      delay(100);
      _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 0));
      _builtInLEDs.show();
      delay(100);
    }

    //FlashBuiltInLEDs();
    Ps3.begin(_ps3MacAddr);
    delay(2000);
  }
  else
  {
    digitalWrite(PIN_BT_CONNECTED_LED, HIGH);

    //FlashBuiltInLEDsForDebug(255, 255, 255);
    delay(125);
    _builtInLEDs.clear();
    _builtInLEDs.show();

    ReadLidarSensors();

    delay(1);

    if ( _didCircleChange )
    {      
      TurnOnFrontLightbar(true);
      _didCircleChange = false;
    }
    // //if(Ps3.event.analog_changed.button.circle)
    // //if(Ps3.data.button.circle)
    // if(Ps3.event.analog_changed.button.circle)
    // {
    //   if ( Ps3.data.analog.button.circle > 0 )
    //   {
    //     TurnOnLightbar(true);
    //   }
    // }

    if(_didCrossChange)
    {
          _keepFLBDemoStrobeOn = !_keepFLBDemoStrobeOn;
    
        StrobeLightbar();
        _didCrossChange = false;
    }
    else if ( _keepFLBDemoStrobeOn )
    {
        StrobeLightbar();
    }

    if(_didSquareChange)
    {
      if ( !_isRearLBOn )
      {
        _isRearLBOn = true;
        _rearLightbar.clear();
        uint8_t _Red = random(1, 256);
        uint8_t _Green = random(1, 256);
        uint8_t _Blue = random(1, 256);
        LightTheRearBar(_Red, _Green, _Blue);
      }
      else
      {
        _isRearLBOn = false;
        _rearLightbar.clear();
        LightTheRearBar(0, 0, 0);
      }

      _rearLightbar.show();
      _didSquareChange = false;
    }

    if(_didTriangleChange)
    {
      if ( _didL2Change )
      {
        _showGroundEffect = !_showGroundEffect;

        if ( _showGroundEffect ) ToggleGroundEffect();
      }
      else
      {
        if ( _showGroundEffect ) ToggleGroundEffect();

        if ( _areBuiltInsOn ) TurnBuiltInsOff(); 
        else                  TurnBuiltInsOn();
      }

      _didL2Change = false;
      _didTriangleChange = false;
    }
    else
    {
      if ( _showGroundEffect ) ToggleGroundEffect(); 
    }

    if ( _didL1Change || _didR1Change )
    {
      _useLiDar = ( _isPS3_L1_Pressed && _isPS3_R1_Pressed ? !_useLiDar : _useLiDar );

      if ( _useLiDar ) 
      {
        FlashBuiltInLEDsForDebug(255, 255, 255);
        delay(200);
        FlashBuiltInLEDsForDebug(0, 0, 0);
      }
      else
      {
        FlashBuiltInLEDsForDebug(255, 0, 0);
        delay(200);
        FlashBuiltInLEDsForDebug(0, 0, 0);
      }
      delay(125);
      _builtInLEDs.clear();
      _builtInLEDs.show();
    }
  }

  TurnOnFrontLightbar();
}

void loopChaser()
{
  ////uint8_t _Red = random(256);
  ////uint8_t _Green = random(256);
  //uint8_t _Blue = random(256);

  //_Blue = random(0, _Blue);

  //_Blue = (_Blue <= 0 ? 256 : _Blue);

  //myLightbar.clear();
  //myLightbar.show();
  //delay(100);

  //LightTheBar(0, 0, _Blue);
  //myLightbar.show();
  //delay(1000);

  //myLightbar.clear();
  //myLightbar.show();
  //delay(100);

  uint8_t red = (uint8_t)0; //random(256);
  uint8_t white = (uint8_t)0; //random(256);
  uint8_t blue = (uint8_t)255; //random(256);
  
  switch (_isRedWhiteOrBlue)
  {
    case RED: 
    {
      red = (uint8_t)255;  
      white = 0;  
      blue = 0;  
      _nextColor = WHITE; 
      break;
    }
    case WHITE: 
    {
      red = (uint8_t)255;  
      white = (uint8_t)255;  
      blue = (uint8_t)255;  
      _nextColor = BLUE; 
      break;
    }
    default: 
    {
      red = 0;  
      white = 0;  
      blue = (uint8_t)255;  
      _nextColor = RED; 
      break;
    }
  }

  ChaserGE(red,white,blue);

  _isRedWhiteOrBlue = _nextColor;
}

void BlinkDebugLED(int BlinkXTimes)
{
  if ( BlinkXTimes > 0 )
  {
    for ( int j = 0; j < BlinkXTimes; j++ )
    {
        digitalWrite(PIN_DEBUG_LED, HIGH);    // turn the LED on (HIGH is the voltage level)
        delay(100);                           // wait
        digitalWrite(PIN_DEBUG_LED, LOW);     // turn the LED off by making the voltage LOW
        delay(100);                           // wait
    }
  }
  else
  {
        digitalWrite(PIN_DEBUG_LED, HIGH);    // turn the LED on (HIGH is the voltage level)
        delay(3000);                           // wait
        digitalWrite(PIN_DEBUG_LED, LOW);     // turn the LED off by making the voltage LOW
        delay(100);                           // wait
        digitalWrite(PIN_DEBUG_LED, HIGH);    // turn the LED on (HIGH is the voltage level)
        delay(3000);                           // wait
        digitalWrite(PIN_DEBUG_LED, LOW);     // turn the LED off by making the voltage LOW
        delay(100);                           // wait
  }
}

void BlinkBuiltInLEDs(uint8_t R, uint8_t G, uint8_t B)
{
    _builtInLEDs.clear();
    _builtInLEDs.show();
    delay(300);
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(R, G, B));   
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(R, G, B));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(R, G, B));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(R, G, B));
    _builtInLEDs.show();
    delay(300);
}

void TurnOnFrontLightbar(bool Manually)
{
  if ( _isStrobeOn ) return;

  LightTheFrontBar();

  //now determine if the lightbar actually lights up or not
  if ( Manually )
  {
    if ( _areHeadlightsManuallyOn )
    {
      //BlinkDebugLED(5);
      //digitalWrite(PIN_FLB_SWITCH, LOW);
      _frontLightbar.clear();
      _frontLightbar.show();
      //setting the pin low MUST be called here, setting low prior to the .show will result in low red LEDs shown when the LB should be off
      digitalWrite(PIN_FLB_SWITCH, LOW);
      _areHeadlightsManuallyOn = false;
      _areHeadlightsOn = false;
    }
    else
    {
      digitalWrite(PIN_FLB_SWITCH, HIGH);
      _frontLightbar.show();
      _areHeadlightsManuallyOn = true;
      _areHeadlightsOn = true;
    }
  }
  else
  {
    //Take a reading using analogRead() on sensor pin and store it in LightVal
    _lightVal = analogRead(PIN_PHOTORESISTOR);

    //Serial.println(LightVal, DEC);
    
    //if LightVal is less than our initial reading (LightCal) minus 50 it is dark and
    //turn pin HIGH. The (-50) part of the statement sets the sensitivity. The smaller
    //the number the more sensitive the circuit will be to variances in light.
    if (_lightVal < _lightCal)
    {
      _frontLightbar.show();
      digitalWrite(PIN_FLB_SWITCH, HIGH);      
      _areHeadlightsOn = true;
    }
    //else, it is bright, turn pin LOW
    else
    {
      if ( !_areHeadlightsManuallyOn )
      {
        _frontLightbar.clear();
        _frontLightbar.show();
        //setting the pin low MUST be called here, setting low prior to the .show will result in low red LEDs shown when the LB should be off
        digitalWrite(PIN_FLB_SWITCH, LOW);
        _areHeadlightsOn = false;
      }
    }
  }  
}

void LightTheFrontBar(uint8_t R, uint8_t G, uint8_t B)
{
  //setup the lightbar to show, but don't turn it on just yet
  for (int j = 0; j < NUM_PIXELS_ON_LB; j++)
  {
    _frontLightbar.setPixelColor(j, _frontLightbar.Color(R, G, B));
  }
}

void LightTheRearBar(uint8_t R, uint8_t G, uint8_t B)
{
  //setup the lightbar to show, but don't turn it on just yet
  for (int j = 0; j < NUM_PIXELS_ON_RLB; j++)
  {
    _rearLightbar.setPixelColor(j, _rearLightbar.Color(R, G, B));
  }
}

void ChaserGE(uint8_t R, uint8_t G, uint8_t B, bool RandomTrailTaper)
{
    uint8_t *colors{new uint8_t[NUM_PIXELS_ON_GELB]{}};

    //go from left to right
    for (int j = 0; j < NUM_PIXELS_ON_GELB; j++)
    {
      for (int p = 0; p < NUM_PIXELS_ON_GELB; p++)
      {
        long pixel = p;
        
        if ( RandomTrailTaper )
        {
          pixel = random(0, p);
        }
        
        uint8_t pixelColor = colors[pixel];
        //uint8_t pixelColor = colors[p];
        if ( pixelColor > 0 )
        {
          pixelColor = (pixelColor/2);
          colors[pixel] = pixelColor;
          //colors[p] = pixelColor;
          switch (_isRedWhiteOrBlue)
          {
            case RED:
            {
              _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(pixelColor, G, B));
              //_groundEffectLB.setPixelColor(p, _groundEffectLB.Color(pixelColor, G, B));
              break;
            }
            case WHITE:
            {
              _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(pixelColor, pixelColor, pixelColor));
              //_groundEffectLB.setPixelColor(p, _groundEffectLB.Color(pixelColor, pixelColor, pixelColor));
              break;
            }
            default:
            {
              _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(R, G, pixelColor));
              //_groundEffectLB.setPixelColor(p, _groundEffectLB.Color(R, G, pixelColor));
              break;
            }
          }
        }
      }

      _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(R, G, B));
      _groundEffectLB.show();
      switch (_isRedWhiteOrBlue)
      {
        case RED:
        {
          colors[j] = R;
          break;
        }
        case WHITE:
        {
          colors[j] = G;
          break;
        }
        default:
        {
          colors[j] = B;
          break;
        }
      }
      delay(100);
    }

    //go from right to left
    for (int j = NUM_PIXELS_ON_GELB; j >= 0; j--)
    {
      //if ( (j-1) >= 0)
      //{
        for (int p = 0; p < NUM_PIXELS_ON_GELB; p++)
        {
          long pixel = p;
          
          if ( RandomTrailTaper )
          {
            pixel = random(0, p);
          }
          
          uint8_t pixelColor = colors[pixel];
          //uint8_t pixelColor = colors[p];
          if ( pixelColor > 0 )
          {
            pixelColor = (pixelColor/2);
            colors[pixel] = pixelColor;
            //colors[p] = pixelColor;
            switch (_isRedWhiteOrBlue)
            {
              case RED:
              {
                _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(pixelColor, G, B));
                //_groundEffectLB.setPixelColor(p, _groundEffectLB.Color(pixelColor, G, B));
                break;
              }
              case WHITE:
              {
                _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(pixelColor, pixelColor, pixelColor));
                //_groundEffectLB.setPixelColor(p, _groundEffectLB.Color(pixelColor, pixelColor, pixelColor));
                break;
              }
              default:
              {
                _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(R, G, pixelColor));
                //_groundEffectLB.setPixelColor(p, _groundEffectLB.Color(R, G, pixelColor));
                break;
              }
            }
          }
        }

        //uint8_t pixelColor = 0;
        //uint8_t blue = (B/2);
        //myLightbar.setPixelColor((j-1), myLightbar.Color(R, G, blue));
      //}
      
      _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(R, G, B));
      _groundEffectLB.show();
      switch (_isRedWhiteOrBlue)
      {
        case RED:
        {
          colors[j] = R;
          break;
        }
        case WHITE:
        {
          colors[j] = G;
          break;
        }
        default:
        {
          colors[j] = B;
          break;
        }
      }
      delay(100);
    }
}

void LightTheGE(uint8_t R, uint8_t G, uint8_t B)
{
    //setup the lightbar to show, but don't turn it on just yet
    for (int j = 0; j < NUM_PIXELS_ON_GELB; j++)
    {
      _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(R, G, B));
    }
}
void StrobeLightbar()
{
    if ( IsRunningInDemoMode() )
    {
      StrobeLightbarDemoMode();
      return;
    }

    _isStrobeOn  = true;
    if ( _areHeadlightsOn )
    {
      _frontLightbar.clear();
      _frontLightbar.show();
      //setting the pin low MUST be called here, setting low prior to the .show will result in low red LEDs shown when the LB should be off
      digitalWrite(PIN_FLB_SWITCH, LOW);
    }

    digitalWrite(PIN_FLB_SWITCH, HIGH);
    delay(1);
    _frontLightbar.clear();
    _frontLightbar.show();
    delay(100);

    //long _Pixel = random(NUM_PIXELS_ON_LB);
    uint8_t _FadeTo = 255;

    //BlinkDebugLED(5);

    int _TimesToStrobe = 1;
    for ( int z = 0; z < _TimesToStrobe; z++ )
    {
      //going left to right
      for ( int j = 0; j < NUM_PIXELS_ON_LB; j++ )
      {
        if ( j > 0 )
        {
          // _FadeTo = (_FadeTo / 2);
          uint32_t _Color = _frontLightbar.getPixelColor(j);
          if ( _Color > 0 )
          {
            _FadeTo = (_Color / 2);
          }
          else
          {
            _FadeTo = 0;
          }
          _frontLightbar.setPixelColor((j - 1), _frontLightbar.Color(0, 0, _FadeTo));
        }

        if ( j < NUM_PIXELS_ON_LB)
        {
          _frontLightbar.setPixelColor(j, _frontLightbar.Color(0, 0, 255));
        }

        digitalWrite(PIN_FLB_SWITCH, HIGH);
        _frontLightbar.show();
        delay(100);
      }

      //BlinkDebugLED(5);

      //going right to left
      for ( int j = (NUM_PIXELS_ON_LB - 1); j >= 0 ; j-- )
      {
        if ( j < (NUM_PIXELS_ON_LB - 1) )
        {
          for ( int x = (NUM_PIXELS_ON_LB - 1); x > j; x-- )
          {
            // _FadeTo = (_FadeTo / 2);
            uint32_t _Color = _frontLightbar.getPixelColor((j + 1));
            if ( _Color > 0 )
            {
              _FadeTo = (_Color / 2);
            }
            else
            {
              _FadeTo = 0;
            }

            _frontLightbar.setPixelColor(x, _frontLightbar.Color(_FadeTo, 0, 0));
          }
        }

        if ( j >= 0 )
        {
          _frontLightbar.setPixelColor(j, _frontLightbar.Color(255, 0, 0));
        }

        digitalWrite(PIN_FLB_SWITCH, HIGH);
        _frontLightbar.show();
        delay(100);
      }
    }

    _frontLightbar.clear();
    digitalWrite(PIN_FLB_SWITCH, HIGH);
    _frontLightbar.show();
  
    digitalWrite(PIN_FLB_SWITCH, LOW);
    delay(1);

    if ( _areHeadlightsOn )
    {
      LightTheFrontBar();
      digitalWrite(PIN_FLB_SWITCH, HIGH);
      _frontLightbar.show();
    }

    //BlinkDebugLED(5);
    _isStrobeOn = false;
}

void StrobeLightbarDemoMode()
{
    _isStrobeOn  = true;
    if ( _areHeadlightsOn )
    {
      _frontLightbar.clear();
      _frontLightbar.show();
      //setting the pin low MUST be called here, setting low prior to the .show will result in low red LEDs shown when the LB should be off
      digitalWrite(PIN_FLB_SWITCH, LOW);
    }

    digitalWrite(PIN_FLB_SWITCH, HIGH);
    delay(1);
    _frontLightbar.clear();
    _frontLightbar.show();
    delay(100);

    //long _Pixel = random(NUM_PIXELS_ON_LB);
    uint8_t _FadeTo = 255;

    //BlinkDebugLED(5);

    int _TimesToStrobe = 1;
    for ( int z = 0; z < _TimesToStrobe; z++ )
    {
      //going left to right
      for ( int j = 0; j < NUM_PIXELS_ON_LB; j++ )
      {
        if ( j > 0 )
        {
          // _FadeTo = (_FadeTo / 2);
          uint32_t _Color = _frontLightbar.getPixelColor(j);
          if ( _Color > 0 )
          {
            _FadeTo = (_Color / 2);
          }
          else
          {
            _FadeTo = 0;
          }
          _frontLightbar.setPixelColor((j - 1), _frontLightbar.Color(0, 0, _FadeTo));
        }

        if ( j < NUM_PIXELS_ON_LB)
        {
          _frontLightbar.setPixelColor(j, _frontLightbar.Color(0, 0, 255));
        }

        digitalWrite(PIN_FLB_SWITCH, HIGH);
        _frontLightbar.show();
        delay(100);
      }

      //BlinkDebugLED(5);

      //going right to left
      for ( int j = (NUM_PIXELS_ON_LB - 1); j >= 0 ; j-- )
      {
        if ( j < (NUM_PIXELS_ON_LB - 1) )
        {
          for ( int x = (NUM_PIXELS_ON_LB - 1); x > j; x-- )
          {
            // _FadeTo = (_FadeTo / 2);
            uint32_t _Color = _frontLightbar.getPixelColor((j + 1));
            if ( _Color > 0 )
            {
              _FadeTo = (_Color / 2);
            }
            else
            {
              _FadeTo = 0;
            }

            _frontLightbar.setPixelColor(x, _frontLightbar.Color(_FadeTo, _FadeTo, 0));
          }
        }

        if ( j >= 0 )
        {
          _frontLightbar.setPixelColor(j, _frontLightbar.Color(255, 255, 0));
        }

        digitalWrite(PIN_FLB_SWITCH, HIGH);
        _frontLightbar.show();
        delay(100);
      }
    }

    _frontLightbar.clear();
    digitalWrite(PIN_FLB_SWITCH, HIGH);
    _frontLightbar.show();
  
    digitalWrite(PIN_FLB_SWITCH, LOW);
    delay(1);

    if ( _areHeadlightsOn )
    {
      LightTheFrontBar();
      digitalWrite(PIN_FLB_SWITCH, HIGH);
      _frontLightbar.show();
    }

    //BlinkDebugLED(5);
    _isStrobeOn = false;
}

/*
    - Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    - Keep sensor #1 awake by keeping XSHUT pin high
    - Put all other sensors into shutdown by pulling XSHUT pins low
    - Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. 
        Going with 0x30 to 0x3F is probably OK.
    - Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    - Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void SetupLidarSensors()
{
  if ( !_useLiDar ) return;

  //FlashBuiltInLEDsForDebug(255, 255, 255); //white

  // reset both front & rear lidar
  if ( !_isFrontLidarOn ) digitalWrite(PIN_XSHUT_FRONT_LOX, LOW);
  if ( !_isRearLidarOn ) digitalWrite(PIN_XSHUT_REAR_LOX, LOW);
  delay(50);

  // turn both front & rear lidar on
  if ( !_isFrontLidarOn ) digitalWrite(PIN_XSHUT_FRONT_LOX, HIGH);
  if ( !_isRearLidarOn ) digitalWrite(PIN_XSHUT_REAR_LOX, HIGH);
  delay(50);

  // keep the front on, turn off the rear
  if ( !_isRearLidarOn ) digitalWrite(PIN_XSHUT_REAR_LOX, LOW);

  //FlashBuiltInLEDsForDebug(255, 0, 0); //red

  // initing front
  if ( !_isFrontLidarOn )
  {
    if(!_frontLox.begin(FRONT_FACING_LOX_I2C_ADDR)) 
    {
      FlashBuiltInLEDsForDebug(255, 255, 0); //yellow
      Serial.println(F("Failed to boot first VL53L0X"));
      _isFrontLidarOn = false;
    }
    else
    {
      _isFrontLidarOn = true;
    }
    delay(50);
    }

  //FlashBuiltInLEDsForDebug(0, 0, 255); //blue

  // activating rear
  //digitalWrite(PIN_XSHUT_FRONT_LOX, LOW);
  if ( !_isRearLidarOn ) digitalWrite(PIN_XSHUT_REAR_LOX, HIGH);
  delay(50);

  //initing rear
  if ( !_isRearLidarOn ) 
  {
    if(!_rearLox.begin(REAR_FACING_LOX_I2C_ADDR)) 
    {
      FlashBuiltInLEDsForDebug(0, 255, 0); //green
      Serial.println(F("Failed to boot rear VL53L0X"));
      _isRearLidarOn = false;
    }
    else
    {
      _isRearLidarOn = true;
    }
  }
}

void ReadLidarSensors() 
{
  if ( !_useLiDar ) 
  {
    BlinkDebugLED(2);
    _isFrontObstacleDetected = false;
    _isRearObstacleDetected = false;
    return;
  }
  //else
  //{
  //  BlinkDebugLED(3);
  //}

  if ( !_isFrontLidarOn || !_isRearLidarOn ) SetupLidarSensors();

  if ( _isFrontLidarOn ) _frontLox.rangingTest(&_front_LOX_Measure, false); // pass in 'true' to get debug data printout!
  if ( _isRearLidarOn ) _rearLox.rangingTest(&_rear_LOX_Measure, false);   // pass in 'true' to get debug data printout!

  //Serial.print("Reading a measurement... ");
  //_FrontLox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if ( !_areBuiltInsOn )
  {
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(_redValue, 0, 0));   
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(_redValue, 0, 0));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(_redValue, 0, 0));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(_redValue, 0, 0));
    _builtInLEDs.show();
  }

  bool stopForward = false;
  bool stopBackward = false;

  //NOTE:  if the sensor is acting weird, verify the protective file (yellow or orange) has been removed from the face of the LiDar sensor
  if ( _isFrontLidarOn )
  {
    if (_front_LOX_Measure.RangeStatus != 4) 
    { // phase failures have incorrect data
      Serial.print("RedVal): "); Serial.println(_redValue);

      if (_front_LOX_Measure.RangeMilliMeter < 300 ) 
      {
        //if ( measure.RangeMilliMeter <= 127) //within 5"
        if (_front_LOX_Measure.RangeMilliMeter <= 250) //within 7 1/2", 200 is 7 3/4"
        {
          stopForward = true;
          _redValue = 255;
        }
        else if ( _redValue < 255)
        {
          _redValue+=5;
        }
      }
      else if(_front_LOX_Measure.RangeMilliMeter > 300 && _redValue > 0)  //300mm is 11.811"
      {
        _redValue-=5;
      }
    } 
    else 
    {
      Serial.println(" out of range ");
      _redValue = 0;
    }
  }
  else 
  {
    Serial.println(" front lox is off ");
    _redValue = 0;
  }

  //NOTE:  if the sensor is acting weird, verify the protective file (yellow or orange) has been removed from the face of the LiDar sensor
  if ( _isRearLidarOn )
  {
    if (_rear_LOX_Measure.RangeStatus != 4) 
    { // phase failures have incorrect data
      Serial.print("RedVal): "); Serial.println(_redValue);

      if (_rear_LOX_Measure.RangeMilliMeter < 300 ) 
      {
        //if ( measure.RangeMilliMeter <= 127) //within 5"
        if (_rear_LOX_Measure.RangeMilliMeter <= 250) //within 7 1/2", 200 is 7 3/4"
        {
          stopBackward = true;
          _redValue = 255;
        }
        else if ( _redValue < 255)
        {
          _redValue+=5;
        }
      }
      else if(_rear_LOX_Measure.RangeMilliMeter > 300 && _redValue > 0)  //300mm is 11.811"
      {
        _redValue-=5;
      }
    } 
    else 
    {
      Serial.println(" out of range ");
      _redValue = 0;
    }
  }
  else 
  {
    Serial.println(" rear lox is off ");
    _redValue = 0;
  }

  _isFrontObstacleDetected = stopForward;
  _isRearObstacleDetected = stopBackward;

  if ( (stopForward && _isMovingForward) || (stopBackward && _isMovingBackward) )
  {       
    //turn off forward (left side)
    ledcWrite(1, 0);
    //turn off reverse (left side)
    ledcWrite(2, 0);

    //turn off forward (right side)
    ledcWrite(3, 0);
    //turn off reverse (right side)
    ledcWrite(4, 0);
  }
}

bool IsRunningInDemoMode()
{
  //return false;
  // pins 34 & zero were used for testing...when pin 34 is jumpered to pin zero then 4095 is read
  return ( analogRead(PIN_DEMO_MODE) == 4095 );
}

void ToggleGroundEffect(uint8_t R, uint8_t G, uint8_t B, bool TurnAllOn, bool FlashRandomly)
{
  if ( IsRunningInDemoMode() )
  {
    // then set rgb color codes to to BW colors
    ToggleGroundEffectDemoMode(TurnAllOn, FlashRandomly);
    return;
  }

  _groundEffectLB.clear();
  _groundEffectLB.show();
  delay(100);

  if ( TurnAllOn )
  {
      for ( int j = 0; j < NUM_PIXELS_ON_GELB; j++ )
      {
        _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(R, G, B));
        _groundEffectLB.show();
        delay(100);
      }
  }
  else
  {
    if ( !FlashRandomly)
    {
      for ( int j = 0; j < NUM_PIXELS_ON_GELB; j++ )
      {
        if ( j > 0 )
        {
          _groundEffectLB.setPixelColor((j - 1), _groundEffectLB.Color(0, 0, 0));
        }

        _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(R, G, B));
        _groundEffectLB.show();
        delay(100);
      }

      for ( int j = (NUM_PIXELS_ON_GELB - 1); j >= 0 ; j-- )
      {
        if ( j < (NUM_PIXELS_ON_GELB - 1) )
        {
          _groundEffectLB.setPixelColor((j + 1), _groundEffectLB.Color(0, 0, 0));
        }

        _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(R, G, B));
        _groundEffectLB.show();
        delay(100);
      }

      _groundEffectLB.clear();
      _groundEffectLB.show();
    }
    else
    {
      long pixel = random(NUM_PIXELS_ON_GELB);

      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(R, G, B));
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));

      pixel = random(NUM_PIXELS_ON_GELB);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(R, G, B));
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(R, G, B));

      pixel = random(NUM_PIXELS_ON_GELB);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));

      pixel = random(NUM_PIXELS_ON_GELB);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(R, G, B));
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(R, G, B));

      pixel = random(NUM_PIXELS_ON_GELB);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.clear();
      _groundEffectLB.show();
    }
  }

  _isGroundEffectOn = !_isGroundEffectOn;
}

void ToggleGroundEffectDemoMode(bool TurnAllOn, bool FlashRandomly)
{
  _groundEffectLB.clear();
  _groundEffectLB.show();
  delay(100);

  if ( TurnAllOn )
  {
      for ( int j = 0; j < NUM_PIXELS_ON_GELB; j++ )
      {
        long rand = random(2);  // fetch a zero or one
        if ( rand > 0 ) _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(255, 255, 0));   // show yellow
        else            _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(0, 0, 255));     // show blue
        _groundEffectLB.show();
        delay(100);
      }
  }
  else
  {
    if ( !FlashRandomly)
    {
      // show blue
      for ( int j = 0; j < NUM_PIXELS_ON_GELB; j++ )
      {
        if ( j > 0 )
        {
          _groundEffectLB.setPixelColor((j - 1), _groundEffectLB.Color(0, 0, 0));
        }

        _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(0, 0, 255));
        _groundEffectLB.show();
        delay(100);
      }

      for ( int j = (NUM_PIXELS_ON_GELB - 1); j >= 0 ; j-- )
      {
        if ( j < (NUM_PIXELS_ON_GELB - 1) )
        {
          _groundEffectLB.setPixelColor((j + 1), _groundEffectLB.Color(0, 0, 0));
        }

        _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(0, 0, 255));
        _groundEffectLB.show();
        delay(100);
      }

      _groundEffectLB.clear();
      _groundEffectLB.show();

      // show yellow
      for ( int j = 0; j < NUM_PIXELS_ON_GELB; j++ )
      {
        if ( j > 0 )
        {
          _groundEffectLB.setPixelColor((j - 1), _groundEffectLB.Color(0, 0, 0));
        }

        _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(255, 255, 0));
        _groundEffectLB.show();
        delay(100);
      }

      for ( int j = (NUM_PIXELS_ON_GELB - 1); j >= 0 ; j-- )
      {
        if ( j < (NUM_PIXELS_ON_GELB - 1) )
        {
          _groundEffectLB.setPixelColor((j + 1), _groundEffectLB.Color(0, 0, 0));
        }

        _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(255, 255, 0));
        _groundEffectLB.show();
        delay(100);

      _groundEffectLB.clear();
      _groundEffectLB.show();

      }
    }
    else
    {
      long pixel = random(NUM_PIXELS_ON_GELB);
      long color = random(2); // zero or one

      if ( color > 0 ) _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(255, 255, 0));  // show yellow
      else             _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 255));    // show blue
      _groundEffectLB.show();
      delay(100);      
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));

      pixel = random(NUM_PIXELS_ON_GELB);
      color = random(2); // zero or one
      if ( color > 0 ) _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(255, 255, 0));  // show yellow
      else             _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 255));    // show blue
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));

      pixel = random(NUM_PIXELS_ON_GELB);
      color = random(2); // zero or one
      if ( color > 0 ) _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(255, 255, 0));  // show yellow
      else             _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 255));    // show blue
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));

      pixel = random(NUM_PIXELS_ON_GELB);
      color = random(2); // zero or one
      if ( color > 0 ) _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(255, 255, 0));  // show yellow
      else             _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 255));    // show blue
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 0));

      pixel = random(NUM_PIXELS_ON_GELB);
      color = random(2); // zero or one
      if ( color > 0 ) _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(255, 255, 0));  // show yellow
      else             _groundEffectLB.setPixelColor(pixel, _groundEffectLB.Color(0, 0, 255));    // show blue
      _groundEffectLB.show();
      delay(100);
      _groundEffectLB.clear();
      _groundEffectLB.show();
    }
  }

  _isGroundEffectOn = !_isGroundEffectOn;
}

void TurnBuiltInsOn()
{
  _builtInLEDs.clear();
  _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(30, 144, 255));   //dodger blue
  _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(255, 140, 0));   //dark orange
  _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(30, 144, 255));
  _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(255, 140, 0));
  _builtInLEDs.show();
  _areBuiltInsOn = true;
}

void TurnBuiltInsOff()
{
  _builtInLEDs.clear();
  _builtInLEDs.show();
  _areBuiltInsOn = false;
}  
  
void OnNotify()
{
    //BlinkDebugLED(1);

    _leftX = (Ps3.data.analog.stick.lx);
    _leftY = (Ps3.data.analog.stick.ly);
    _rightX = (Ps3.data.analog.stick.rx);
    _rightY = (Ps3.data.analog.stick.ry);

    if (Ps3.event.analog_changed.button.l1)
    {
      _didL1Change = true;
      _isPS3_L1_Pressed = ( Ps3.data.analog.button.l1 > 0 );
    }

    if (Ps3.event.analog_changed.button.l2)
    {
      _didL2Change = true;
      _isPS3_L2_Pressed = ( Ps3.data.analog.button.l2 > 0 );
    }

    if (Ps3.event.analog_changed.button.r1)
    {
      _didR1Change = true;
      _isPS3_R1_Pressed = ( Ps3.data.analog.button.r1 > 0 );
    }

    if(Ps3.event.analog_changed.button.circle)
    {
      if ( Ps3.data.analog.button.circle > 0 )
      {
        _didCircleChange = true;
      }
    }

    if(Ps3.event.analog_changed.button.triangle)
    {
      if ( Ps3.data.analog.button.triangle > 0 )
      {
        _didTriangleChange = true;
      }
    }

    if(Ps3.event.analog_changed.button.cross)
    {
      if ( Ps3.data.analog.button.cross > 0 )
      {
        _didCrossChange = true;
      }
    }

    if(Ps3.event.analog_changed.button.square)
    {
      if ( Ps3.data.analog.button.square > 0 )
      {
        _didSquareChange = true;
      }
    }

//     //Notes:
//     //
//     //PS3 controller:
//     //Stick:
//     //a negative value indicates the joystick is being pushed forward
//     //a positive value indicates the joystick is being pulled towards the user/aft
//     //a zero value indicates the joystick is in the neutral position
//     //
//     //Buttons: Ps3.event.analog_changed.button.*
//     //square, cross, circle, triangle, l1, l2, r1, r2, up, down, left, right
//     //  All of these fields/buttons are declared as uint8_t, which means that they can have a value between 0 and 255. If we look into this file, we can check that these fields will have the difference between the previous analog value of the button and the current one. This means that the value will be greater than 0 if the analog value of the button changes.
//     //  It’s important to take in consideration that this value will be equal to zero while the button is pressed but in the same position, since the previous position will be equal to the current one.
//     //  For example, if we click the R2 button to the maximum and hold it pressed, the corresponding field will be zero. It will only be different from zero when the button is going from not pressed to pressed, and then from pressed to not pressed (or between intermediate states).
//     //  Note also that even if we give a single click from not pressed to fully pressed, multiple intermediate positions may be detected. Consequently, multiple events will be triggered.
//     //  so
//     //  this renders the event that has changed: Ps3.event.analog_changed.button.circle
//     //  and this renders the value indicating the change: Ps3.data.analog.button.circle
//     //    we simply access the button to obtain the analog value. 0 corresponds to not pressed and 255 corresponds to fully pressed.
//     //    IE: 
//     //      if(Ps3.event.analog_changed.button.square)
//     //      {
//     //        Serial.print("Square New value: ");
//     //        Serial.println(Ps3.data.analog.button.square);
//     //      }
//     //
//     //Motors:
//     //the motors run in tandem...1 & 2 work the left side, 3 & 4 the right
//     //1 & 3 are forwards
//     //2 & 4 are reverse
//     //a PWM value of: 0 = stop, 255 = full speed, 127 is half speed



//     //the following code is functional for left to right joystick movements (the X axis), but does not take into account what the Y axis is doing
//     //if ( leftX < -5 ) //the left joystick is being pulled to the left
//     //{
//     //  //turn on forward (left side)
//     //  ledcWrite(1, 0);
//     //  //turn on reverse (left side)
//     //  ledcWrite(2, (abs(leftX) + 127));
//     //}
//     //else if (leftX > 5) //the left joystick is being pushed to the right
//     //{
//     //  //turn on forward (left side)
//     //  ledcWrite(1, (abs(leftX) + 127));
//     //  //turn off reverse (left side)
//     //  ledcWrite(2, 0);
//     //}
//     //else
//     //{
//     //  //turn off forward (left side)
//     //  ledcWrite(1, 0);
//     //  //turn off reverse (left side)
//     //  ledcWrite(2, 0);
//     //}
// 	  //
//     //if ( rightX < -5 ) //the right joystick is being pushed to the left
//     //{
//     //  //turn on forward (right side)
//     //  ledcWrite(3, (abs(rightX) + 127));
//     //  //turn off reverse (right side)
//     //  ledcWrite(4, 0);
//     //}
//     //else if (rightX > 5) //the right joystick is being pulled to the right
//     //{
//     //  //turn on forward (right side)
//     //  ledcWrite(3, 0);
//     //  //turn on reverse (right side)
//     //  ledcWrite(4, (abs(rightX) + 127));
//     //}
//     //else
//     //{
//     //  //turn off forward (right side)
//     //  ledcWrite(3, 0);
//     //  //turn off reverse (right side)
//     //  ledcWrite(4, 0);
//     //}


    if ( _leftY < -5 ) //the joystick is being pushed forward
    {
      if ( !_isFrontObstacleDetected )
      {
        //turn on forward (left side)
        ledcWrite(1, (abs(_leftY) + 127));
        //turn off reverse (left side)
        ledcWrite(2, 0);
        _isMovingForward = true;
        _isMovingBackward = false;
      }
    }
    else if ( _leftY > 5 && !_isRearObstacleDetected ) //the joystick is being pulled aft
    {
      //turn on forward (left side)
      ledcWrite(1, 0);
      //turn on reverse (left side)
      ledcWrite(2, (abs(_leftY) + 127));
      _isMovingForward = false;
      _isMovingBackward = true;
    }
    else
    {
      //turn off forward (left side)
      ledcWrite(1, 0);
      //turn off reverse (left side)
      ledcWrite(2, 0);
      _isMovingForward = false;
      _isMovingBackward = false;
    }
	
    if ( _rightY < -5 ) //the joystick is being pushed forward
    {
      if ( !_isFrontObstacleDetected )
      {
        //turn on forward (right side)
        ledcWrite(3, (abs(_rightY) + 127));
        //turn off reverse (right side)
        ledcWrite(4, 0);
        _isMovingForward = true;
        _isMovingBackward = false;
      }
    }
    else if (_rightY > 5 && !_isRearObstacleDetected)  // the joystick is being pulled aft
    {
      //turn on forward (right side)
      ledcWrite(3, 0);
      //turn on reverse (right side)
      ledcWrite(4, (abs(_rightY) + 127));
      _isMovingForward = false;
      _isMovingBackward = true;
    }
    else
    {
      //turn off forward (right side)
      ledcWrite(3, 0);
      //turn off reverse (right side)
      ledcWrite(4, 0);
      _isMovingForward = false;
      _isMovingBackward = false;
    }


    ////all four wheels oppossing direction...basically it will spin in a circle...donuts!!!
    //if ( _rightY < -5 )
    //{
    //  //forward
    //  ledcWrite(3, (abs(_rightY) + 127));
    //}
    //else if (_rightY > 5)
    //{
    //  ledcWrite(3, 0);
    //}
}

void OnConnect()
{
    digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.show();
    delay(2000);
    _builtInLEDs.clear();
    _builtInLEDs.show();
    digitalWrite(PIN_BT_CONNECTED_LED, LOW);
}

void FlashFrontLightbar(bool FlashRandomly)
{
    digitalWrite(PIN_FLB_SWITCH, HIGH);
    _frontLightbar.clear();
    _frontLightbar.show();
    delay(100);

    if ( !FlashRandomly)
    {
      for ( int j = 0; j < NUM_PIXELS_ON_LB; j++ )
      {
        if ( j > 0 )
        {
          _frontLightbar.setPixelColor((j - 1), _frontLightbar.Color(0, 0, 0));
        }

        _frontLightbar.setPixelColor(j, _frontLightbar.Color(255, 255, 255));
        _frontLightbar.show();
        delay(100);
      }

      for ( int j = (NUM_PIXELS_ON_LB - 1); j >= 0 ; j-- )
      {
        if ( j < (NUM_PIXELS_ON_LB - 1) )
        {
          _frontLightbar.setPixelColor((j + 1), _frontLightbar.Color(0, 0, 0));
        }

        _frontLightbar.setPixelColor(j, _frontLightbar.Color(255, 255, 255));
        _frontLightbar.show();
        delay(100);
      }

      _frontLightbar.clear();
      _frontLightbar.show();
    }
    else
    {
      long _Pixel = random(NUM_PIXELS_ON_LB);

      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(255, 0, 0));
      _frontLightbar.show();
      delay(100);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(0, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_LB);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(255, 0, 0));
      _frontLightbar.show();
      delay(100);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(255, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_LB);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(0, 0, 0));
      _frontLightbar.show();
      delay(100);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(0, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_LB);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(255, 0, 0));
      _frontLightbar.show();
      delay(100);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(255, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_LB);
      _frontLightbar.setPixelColor(_Pixel, _frontLightbar.Color(0, 0, 0));
      _frontLightbar.show();
      delay(100);
      _frontLightbar.clear();
      _frontLightbar.show();
    }

    digitalWrite(PIN_FLB_SWITCH, LOW);
}

void FlashRearLightbar(bool FlashRandomly)
{
    _rearLightbar.clear();
    _rearLightbar.show();
    delay(100);

    if ( !FlashRandomly)
    {
      for ( int j = 0; j < NUM_PIXELS_ON_RLB; j++ )
      {
        if ( j > 0 )
        {
          _rearLightbar.setPixelColor((j - 1), _rearLightbar.Color(0, 0, 0));
        }

        _rearLightbar.setPixelColor(j, _rearLightbar.Color(255, 255, 255));
        _rearLightbar.show();
        delay(100);
      }

      for ( int j = (NUM_PIXELS_ON_RLB - 1); j >= 0 ; j-- )
      {
        if ( j < (NUM_PIXELS_ON_RLB - 1) )
        {
          _rearLightbar.setPixelColor((j + 1), _rearLightbar.Color(0, 0, 0));
        }

        _rearLightbar.setPixelColor(j, _rearLightbar.Color(255, 255, 255));
        _rearLightbar.show();
        delay(100);
      }

      _rearLightbar.clear();
      _rearLightbar.show();
    }
    else
    {
      long _Pixel = random(NUM_PIXELS_ON_RLB);

      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(255, 0, 0));
      _rearLightbar.show();
      delay(100);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(0, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_RLB);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(255, 0, 0));
      _rearLightbar.show();
      delay(100);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(255, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_RLB);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(0, 0, 0));
      _rearLightbar.show();
      delay(100);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(0, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_RLB);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(255, 0, 0));
      _rearLightbar.show();
      delay(100);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(255, 0, 0));

      _Pixel = random(NUM_PIXELS_ON_RLB);
      _rearLightbar.setPixelColor(_Pixel, _rearLightbar.Color(0, 0, 0));
      _rearLightbar.show();
      delay(100);
      _rearLightbar.clear();
      _rearLightbar.show();
    }
}

void FlashBuiltInLEDs()
{
  if ( Ps3.isConnected() )
  {
    digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(0, 0, 255));
    _builtInLEDs.show();
    delay(500);
    _builtInLEDs.clear();
    _builtInLEDs.show();
    digitalWrite(PIN_BT_CONNECTED_LED, LOW);
  }
  else
  {
    _builtInLEDs.clear();
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.clear();
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.clear();
    _builtInLEDs.show();
    delay(200);
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(255, 0, 0));
    _builtInLEDs.show();
    delay(500);
    _builtInLEDs.clear();
    _builtInLEDs.show();
  }
}

void FlashBuiltInLEDsForDebug(uint8_t R, uint8_t G, uint8_t B)
{
  
    _builtInLEDs.clear();
    _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(R, G, B));
    _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(R, G, B));
    _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(R, G, B));
    _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(R, G, B));
    _builtInLEDs.show();
    delay(200);
    // _builtInLEDs.clear();
    // _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.show();
    // delay(200);
    // _builtInLEDs.clear();
    // _builtInLEDs.setPixelColor(0, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.setPixelColor(1, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.setPixelColor(2, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.setPixelColor(3, _builtInLEDs.Color(R, G, B));
    // _builtInLEDs.show();
    // delay(200);
    // _builtInLEDs.clear();
    // _builtInLEDs.show();
}
