#pragma region Code Hx / Change Log
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
  07.28.23 DJP - Finished testing code changes for Wizzy Rover Camp 2023
  08.03.23 DJP - Added PS4 support
  08.05.23 DJP - Changed event time from 150ms to 250ms to resolve a button pressed timing issue
  08.05.23 DJP - Rover gets laxed/sluggish in response to PS4 controller requests, adding a high gain antenna does resolve this issue
*/
#pragma endregion Code Hx / Change Log

//confirm the com is correct in platform.ini file then open a new terminal window, key in the following and press enter: pio run --target upload

//run sixaxispair.exe to determine the address of the PS3 controller, the default address is 01:02:03:04:05:06.  The default will work as long as
// no other ps3 wireless controllers are nearby and turned on.  SixAxisPair will allow you to change the address if so desired.

//common pio commands: 
//  pio device list
//  pio run --target clean
//  pio run --target upload
//  pio run --target upload --upload-port com5
//
// MAKE CERTAIN YOUR TERMINAL WINDOW IS IN THE CORRECT DIRECTORY ..\PlatformIO\Projects\Rover-2023\Wizzy Rover
//     if not, use the cd command via the PS prompt to change

#include <Arduino.h>            // Arduino Framework
#include <Adafruit_NeoPixel.h>
//#include <Tone32.h>

#include <ps4.h>
#include <PS4Controller.h>
#include <ps4_int.h>

//#include <esp32-hal-ledc.h>
#include "Adafruit_VL53L0X.h"
#include <tuple>

#ifndef __ESP32__
#define __ESP32__
#endif

#define NUM_PIXELS_ON_DB      4     // We have 4 LEDs on the driver board (DB) (with the LEDs facing you, index 3 is upper left, index 2 is lower left, index 1 is upper right, and index 0 is lower right)
#define NUM_PIXELS_ON_FLB     6     // We have 6 LEDs on the front lightbar (FLB)
#define NUM_PIXELS_ON_RLB     6     // We have 6 LEDs on the rear lightbar (RLB)
#define NUM_PIXELS_ON_GELB    12    // We have 6 LEDs on each ground effect lightbar (GELB)

//The PIN_MOTOR* are reserved pins; they are on our chip that we're connecting to our ESP32 through our circuit board!
#define PIN_MOTOR2            12    // RESERVED for motor 2
#define PIN_MOTOR1            13    // RESERVED for motor 1
#define PIN_FLB_SWITCH        14    // Flips the transistor switch on/off (high is on, closes the circuit)
#define PIN_AVAIL_1           15    // Not currently used
#define PIN_MOTOR3            16    // RESERVED for motor 3
#define PIN_MOTOR4            17    // RESERVED for motor 4
#define PIN_PIXELS_DB         18    // Driver board LEDs are hardwired to pin 18
#define PIN_BUZZER            19    // The buzzer is hardwired to pin 19
#define PIN_AVAIL_2           20    // Not currently used
#define PIN_SDA_LOX           21    // LiDar data pin
#define PIN_SCL_LOX           22    // LiDar clock pin
#define PIN_PIXELS_GELB       23    // Ground effect LB's
#define PIN_UNKNOWN           24    // Not defined on board
#define PIN_XSHUT_REAR_LOX    25    // shutdown pin for rear facing LOX
#define PIN_BT_CONNECTED_LED  26    // Blue LED indicated BT connection established
#define PIN_PIXELS_FLB        27    // Lightbar signal/DIN pin...this will control both front & rear LB
#define PIN_DEBUG_LED         32    // Turns on/off the white debug led 
#define PIN_XSHUT_FRONT_LOX   33    // shutdown pin for front facing LOX
#define PIN_DEMO_MODE         34    // when set high, causes the code to run in BW demo mode
#define PIN_PHOTORESISTOR     35    // Reads the photoresistor analog value

enum Lightbar { FRONT, REAR, GROUND_EFFECT, FRONT_AND_REAR, BUILT_IN };
enum Color { UNKNOWN, RED, WHITE, BLUE, GREEN, YELLOW, ORANGE, LIGHTBLUE };
// Exercise 1: Add a custom color to the enum above

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

int _frontRedValue = 0;
int _rearRedValue = 0;

int _color = 1;

//use SixAxisPairTool to set a custom mac address...the address below is the default that ships on the device
//char _ps3MacAddr[20] = { "01:02:03:04:05:06" };
//char _ps3MacAddr[20] = { "21:02:03:04:05:02"}; //2021 class
//char _ps3MacAddr[20] = { "23:07:31:04:05:01"}; //2023 class
char _ps4MacAddr[20] = {"23:08:03:04:05:19"};

uint8_t _lbBrightness = 128;
const uint8_t MAX_LB_BRIGHTNESS = 255;
const int LOOPS_BETWEEN_BLINKS = 5;

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
bool _didR2Change = false;
bool _isPS4_L1_Pressed = false;
bool _isPS4_R1_Pressed = false;
bool _isPS4_R2_Pressed = false;
bool _isPS4_L2_Pressed = false;
bool _showGroundEffect = false;
bool _keepFLBDemoStrobeOn = false;
int _loopsBetweenBlinks = 0;
bool _showPS4Connection = true;
bool _movementRequested = false;
bool _wasPSButtonPressed = false;

// Set up some variables for the light level: a calibration value and and a raw light value
int _lightCal;
int _lightVal;

Adafruit_NeoPixel _builtInLEDs(NUM_PIXELS_ON_DB, PIN_PIXELS_DB, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel _frontLightbar((NUM_PIXELS_ON_FLB + NUM_PIXELS_ON_RLB), PIN_PIXELS_FLB, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel _groundEffectLB(NUM_PIXELS_ON_GELB, PIN_PIXELS_GELB, NEO_GRB + NEO_KHZ800);

Adafruit_VL53L0X _frontLox = Adafruit_VL53L0X();
Adafruit_VL53L0X _rearLox = Adafruit_VL53L0X();
// this holds the lidar measurement 
VL53L0X_RangingMeasurementData_t _front_LOX_Measure;
VL53L0X_RangingMeasurementData_t _rear_LOX_Measure;

// LED related methods
void SetupLightbars();
void BlinkDebugLED(int BlinkXTimes);
void Chaser(uint8_t R, uint8_t G, uint8_t B, Lightbar LB, bool RandomTrailTaper = false);
void Chaser(Color color, Lightbar LB, bool RandomTrailTaper = false);
void ToggleLightbar(Lightbar LB, bool TurnOn = true, uint8_t R = 255, uint8_t G = 255, uint8_t B = 255);
void ToggleLightbar(Lightbar LB,  Color color, bool TurnOn = true);
void ToggleLBDueToLight();
void FlashLightbar(Lightbar LB, int numFlashes = 1, uint8_t R = 255, uint8_t G = 255, uint8_t B = 255);
bool IsRunningInDemoMode();

// LiDar sensor related methods
void SetupLidarSensors();
void ReadLidarSensors();

void SetupMotors();
void SetupPins();

void setup()
{
  Serial.begin(115200);
  
  SetupPins();
  SetupLightbars();
  SetupMotors();

  ToggleLightbar(GROUND_EFFECT, YELLOW, true);

  digitalWrite(PIN_DEBUG_LED, HIGH);
  
  // for debug
  Chaser(BLUE, FRONT_AND_REAR);
  Chaser(BLUE, GROUND_EFFECT);
  Chaser(BLUE, BUILT_IN);

  InitializePS4();

  SetupLidarSensors();

  delay(1000);

  _loopsBetweenBlinks = (LOOPS_BETWEEN_BLINKS + 1);

  digitalWrite(PIN_DEBUG_LED, LOW);  
}

void loop()
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

  if ( !PS4.isConnected() )
  {
    Serial.println("No PS4 controller connected...");
    _showPS4Connection = true;
    
    for ( int j = 0; j < 3; j++ )
    {
      digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
      delay(100);
      digitalWrite(PIN_BT_CONNECTED_LED, LOW);
      delay(100);
    }

    Chaser(BLUE, BUILT_IN);

    // if( PS4.isConnected())
    // {
    //   FlashBuiltInLEDs(3, 0, 0, 255);
    // }
    // else
    // { 
    //   FlashBuiltInLEDs(3, 255, 0, 0);
    // }
    // PS4.begin(_ps4MacAddr);
    // delay(2000);
  }
  else
  {
    if ( _showPS4Connection ) { Serial.println("PS4 CONNECTED..."); _showPS4Connection = false; }

    digitalWrite(PIN_BT_CONNECTED_LED, HIGH);

    //FlashLightbar(BUILT_IN);
    delay(125);

    ReadLidarSensors();

    delay(100);

    if ( _wasPSButtonPressed )
    {
      int batteryLvl = PS4.Battery();

      //(???) I believe the battery scale is 0 - 9 as the battery lvl always seems to be single digits
      
      Serial.printf("Battery Level: %d\n", PS4.Battery());

      //(???) change this to low, medium & good
      if ( batteryLvl <= 2 )
      {
        Serial.println("Battery Level is low, flashing controller red");
        FlashController(255, 0, 0);
      }
      else
      {
        Serial.println("Battery Level is good, flashing controller green");
        FlashController(0, 255, 0);
      }
      
      _wasPSButtonPressed = false;
    }
    else
    {
        FlashController(0, 0, 255);
    }

    if ( _didCircleChange )
    {
      //TurnOnFrontLightbar(true);
      ToggleLightbar(FRONT, true);
      //delay here just long enough to allow the user to press and release the buttons...if the user wants to sit on 
      //the buttons, well that's different code.  Anything less than 250 millis is not long enough
      delay(250);
      _didCircleChange = false;
    }

    if(_didCrossChange)
    {
        _keepFLBDemoStrobeOn = !_keepFLBDemoStrobeOn;

        Chaser(WHITE, FRONT);

        //delay here just long enough to allow the user to press and release the buttons...if the user wants to sit on 
        //the buttons, well that's different code.  Anything less than 250 millis is not long enough
        delay(250);
        _didCrossChange = false;
    }
    else if ( _keepFLBDemoStrobeOn && IsRunningInDemoMode() )
    {
        Chaser(WHITE, FRONT);
    }

    if(_didSquareChange)
    {
      if ( !_isRearLBOn )
      {
        _isRearLBOn = true;
        uint8_t red = random(1, 256);
        uint8_t green = random(1, 256);
        uint8_t blue = random(1, 256);
        //delay here just long enough to allow the user to press and release the buttons...if the user wants to sit on 
        //the buttons, well that's different code.  Anything less than 250 millis is not long enough
        ToggleLightbar(REAR, red, green, blue);
        delay(250);
      }
      else
      {
        _isRearLBOn = false;
        ToggleLightbar(REAR, false);
      }

      _didSquareChange = false;
    }

//    Serial.print("Triangle Changed: "); Serial.println(_didTriangleChange);
//    Serial.print("L2 Changed: "); Serial.println(_didL2Change);
//    Serial.print("ShowGroundEffect: "); Serial.println(_showGroundEffect);
    if(_didTriangleChange)
    {
      if ( _didL2Change )
      {
        _showGroundEffect = !_showGroundEffect;

//        Serial.print("Turning Ground Effect: "); (_showGroundEffect ? Serial.println("ON") : Serial.println("OFF") );

        ToggleLightbar(GROUND_EFFECT, _showGroundEffect);

//        Serial.println("Exiting if condition...");
      }
      else
      {
        if ( _areBuiltInsOn ) ToggleLightbar(BUILT_IN); 
        else                  ToggleLightbar(BUILT_IN, false);
      }

      //delay here just long enough to allow the user to press and release the buttons...if the user wants to sit on 
      //the buttons, well that's different code.  Anything less than 250 millis is not long enough
      delay(250);
      _didL2Change = false;
      _didTriangleChange = false;
    }
    else
    {
      //if ( _showGroundEffect ) ToggleLightbar(GROUND_EFFECT);
    }

    if ( _didL1Change || _didR1Change )
    {
      _useLiDar = ( _isPS4_L1_Pressed && _isPS4_R1_Pressed ? !_useLiDar : _useLiDar );

      if ( _useLiDar ) 
      {
        FlashLightbar(BUILT_IN);
      }
      else
      {
        FlashLightbar(BUILT_IN, 1, 255, 0, 0);
      }
      
      //delay here just long enough to allow the user to press and release the buttons...if the user wants to sit on 
      //the buttons, well that's different code.  Anything less than 250 millis is not long enough
      delay(250);
      _builtInLEDs.clear();
      _builtInLEDs.show();
    }
  }

  _didL1Change = _didL2Change = _didR1Change = _didR2Change = false;
  _didTriangleChange = _didCircleChange = _didCrossChange = _didSquareChange = false;
  
  ToggleLBDueToLight();

//  Serial.println("End of loop...");
}

#pragma region SetUp Helper Methods
void SetupPins()
{
  pinMode(PIN_DEMO_MODE, INPUT_PULLDOWN); //pin 34

  pinMode(PIN_DEBUG_LED, OUTPUT);         //pin 32
  pinMode(PIN_BT_CONNECTED_LED, OUTPUT);  //pin 26

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_PHOTORESISTOR, INPUT);      //pin 35

  digitalWrite(PIN_DEBUG_LED, HIGH);
  digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
  delay(500);
  
  pinMode(PIN_PIXELS_GELB, OUTPUT);       //pin 23
  digitalWrite(PIN_PIXELS_GELB, LOW);

  //this is the switch, high allows the front LB to turn on
  pinMode(PIN_FLB_SWITCH, OUTPUT);        //pin 14

  pinMode(PIN_PIXELS_FLB, OUTPUT);        //pin 27
  digitalWrite(PIN_PIXELS_FLB, LOW);
}
void SetupMotors()
{
  // Setup the motors
  ledcSetup(1, 30000, 8); //we set up PWM channel 1, frequency of 30,000 Hz, 8 bit resolution
  ledcAttachPin(_inOne,1); //we're going to attach inOne to our new PWM channel
  ledcSetup(2, 30000, 8); //we'll set up the rest of our PWM channels, just like before.
  ledcAttachPin(_inTwo,2); //this time we'll need to set up 8 PWM channels!
  ledcSetup(3, 30000, 8);
  ledcAttachPin(_inThree,3);
  ledcSetup(4, 30000, 8);
  ledcAttachPin(_inFour,4);

  delay(250);
}
void SetupLightbars()
{
  // set up lightbars and built in LEDs
  _builtInLEDs.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _builtInLEDs.setBrightness(MAX_LB_BRIGHTNESS); 
  delay(250);

  _frontLightbar.begin();
  _frontLightbar.setBrightness(MAX_LB_BRIGHTNESS);
  delay(250);

  _groundEffectLB.begin();
  _groundEffectLB.setBrightness(MAX_LB_BRIGHTNESS);
  delay(250);

  // set colors for RWB Chaser sequence
  _isRedWhiteOrBlue = RED;
  _nextColor = WHITE;

  // set brightness lower when we want to conserve battery
  _lbBrightness = 128;
  _frontLightbar.setBrightness(_lbBrightness); 

  //light sensor calibration
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) 
  _lightCal = 300;   // 725 is a good value for normal ambient light, 300 to simulate lights out (IE finger over the sensor)
#endif
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) 
  _lightCal = 300;   // 725 is a good value for normal ambient light, 300 to simulate lights out (IE finger over the sensor)
#endif
#if defined(__ESP32__)
  _lightCal = 1600;
#endif
}
void InitializePS4()
{
  Serial.print("Initializing PS4 to ");
  Serial.println(_ps4MacAddr);

  PS4.attach(OnNotify);
  PS4.attachOnConnect(OnConnect);

  if ( !PS4.begin(_ps4MacAddr) )
  {
    Serial.println("PS4 Failed to Initialize!!!");
  }
  else
  {
    Serial.println("PS4 Initialized...");
  }
}
#pragma endregion Setup Helper Methods

#pragma region LEDs
void ToggleLBDueToLight()
{    
    //Take a reading using analogRead() on sensor pin and store it in LightVal
    _lightVal = analogRead(PIN_PHOTORESISTOR);

    //Serial.println(LightVal, DEC);
    
    //if LightVal is less than our initial reading (LightCal) minus 50 it is dark and
    //turn pin HIGH. The (-50) part of the statement sets the sensitivity. The smaller
    //the number the more sensitive the circuit will be to variances in light.
    if (_lightVal < _lightCal)
    {
      ToggleLightbar(FRONT);
      _areHeadlightsOn = true;
    }
    //else, it is bright, turn pin LOW
    else
    {
      if ( !_areHeadlightsManuallyOn )
      {
        ToggleLightbar(FRONT, false);
        //setting the pin low MUST be called here after the show, setting low prior to the .show will result in low red LEDs shown when the LB should be off
        digitalWrite(PIN_FLB_SWITCH, LOW);
        _areHeadlightsOn = false;
      }
    }
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
  delay(250);
}

// Helper method to take our color enum and create an RGB value tuple
std::tuple<uint8_t, uint8_t, uint8_t> ToRGB8(Color color){
  switch (color)
  {
    case RED:
      return std::tuple<uint8_t, uint8_t, uint8_t>(255, 0, 0);
    case WHITE:
      return std::tuple<uint8_t, uint8_t, uint8_t>(255, 255, 255);
    case BLUE:
      return std::tuple<uint8_t, uint8_t, uint8_t>(0, 0, 255);
    case GREEN:
      return std::tuple<uint8_t, uint8_t, uint8_t>(0, 255, 0);
    case LIGHTBLUE:
      return std::tuple<uint8_t, uint8_t, uint8_t>(50, 147, 168);
    case YELLOW:
      return std::tuple<uint8_t, uint8_t, uint8_t>(245, 242, 31);
    case ORANGE:
      return std::tuple<uint8_t, uint8_t, uint8_t>(255, 77, 0);
    default: 
      return std::tuple<uint8_t, uint8_t, uint8_t>(255, 255, 255);
  }
}

void Chaser(Color color, Lightbar LB, bool RandomTrailTaper)
{
  std::tuple<uint8_t, uint8_t, uint8_t> rgb = ToRGB8(color);
  Chaser(std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb), LB, RandomTrailTaper);
}

void Chaser(uint8_t R, uint8_t G, uint8_t B, Lightbar LB, bool RandomTrailTaper)
{
#pragma region Code To Consider

    //the following code was from StrobeLightbar()...it set the switch vs. turning the lightbar on directly...that needs to be coded here
    // _frontLightbar.clear();
    // digitalWrite(PIN_FLB_SWITCH, HIGH);
    // _frontLightbar.show();
  
    // digitalWrite(PIN_FLB_SWITCH, LOW);
    // delay(1);

    // if ( _areHeadlightsOn )
    // {
    //   LightTheFrontBar();
    //   digitalWrite(PIN_FLB_SWITCH, HIGH);
    //   _frontLightbar.show();
    // }

    // //BlinkDebugLED(5);
    // _isStrobeOn = false;
#pragma endregion Code To Consider

    int firstPixel = 0;
    int numberOfPixels;
    Adafruit_NeoPixel* bar = &_groundEffectLB;

    switch (LB)
    {
      case FRONT:
        {
          bar = &_frontLightbar;
          numberOfPixels = NUM_PIXELS_ON_FLB;
          break;
        }
      case REAR:
        {
          bar = &_frontLightbar;
          firstPixel = NUM_PIXELS_ON_FLB;
          numberOfPixels = (NUM_PIXELS_ON_FLB + NUM_PIXELS_ON_RLB);
          break;
        }
      case GROUND_EFFECT:
        {
          bar = &_groundEffectLB;
          numberOfPixels = NUM_PIXELS_ON_GELB;
          break;
        }
      case FRONT_AND_REAR:
        {
          bar = &_frontLightbar;
          numberOfPixels = (NUM_PIXELS_ON_FLB + NUM_PIXELS_ON_RLB);
          break;
        }
      case BUILT_IN:
      {
        bar = &_builtInLEDs;
        numberOfPixels = NUM_PIXELS_ON_DB;
        break;
      }
      default:
        {
          bar = &_frontLightbar;
          numberOfPixels = 6;
          firstPixel = 0;
          break;
        }
    }

    uint8_t *colorR{new uint8_t[numberOfPixels]};
    uint8_t *colorG{new uint8_t[numberOfPixels]};
    uint8_t *colorB{new uint8_t[numberOfPixels]};

    memset(colorR, 0, numberOfPixels);
    memset(colorG, 0, numberOfPixels);
    memset(colorB, 0, numberOfPixels);

    long pixel = 0;
    uint8_t pixelColorR = 0;
    uint8_t pixelColorG = 0;
    uint8_t pixelColorB = 0;

    //go from left to right
    for (int j = firstPixel; j < numberOfPixels; j++)
    {
      for (int p = 0; p < numberOfPixels; p++)
      {       
        pixel = p;

        if ( RandomTrailTaper )
        {
          pixel = random(0, p);
        }
        
        pixelColorR = colorR[pixel];
        pixelColorG = colorG[pixel];
        pixelColorB = colorB[pixel];

        if ( pixelColorR > 0 || pixelColorG > 0 || pixelColorB > 0 )
        {
          pixelColorR = (pixelColorR > 0 ? ( pixelColorR / 2 ) : 0);
          pixelColorG = (pixelColorG > 0 ? ( pixelColorG / 2 ) : 0);
          pixelColorB = (pixelColorB > 0 ? ( pixelColorB / 2 ) : 0);

          colorR[pixel] = pixelColorR;
          colorG[pixel] = pixelColorG;
          colorB[pixel] = pixelColorB;

          bar->setPixelColor(pixel, bar->Color(pixelColorR, pixelColorG, pixelColorB));
        }
      }

      //set the leading pixel/led
      bar->setPixelColor(j, bar->Color(R, G, B));
      bar->show();
      colorR[j] = R;
      colorG[j] = G;
      colorB[j] = B;
      delay(100);
    }

    pixel = 0;
    pixelColorR = 0;
    pixelColorG = 0;
    pixelColorB = 0;

    //go from right to left
    for (int j = (numberOfPixels-1); j >= firstPixel; j--)
    {
      for (int p = 0; p < numberOfPixels; p++)
      {
        pixel = p;
        
        if ( RandomTrailTaper )
        {
          pixel = random(0, p);
        }
        
        pixelColorR = colorR[pixel];
        pixelColorG = colorG[pixel];
        pixelColorB = colorB[pixel];
        if ( pixelColorR > 0 || pixelColorG > 0 || pixelColorB > 0 )
        {
          pixelColorR = (pixelColorR > 0 ? ( pixelColorR / 2 ) : 0);
          pixelColorG = (pixelColorG > 0 ? ( pixelColorG / 2 ) : 0);
          pixelColorB = (pixelColorB > 0 ? ( pixelColorB / 2 ) : 0);

          colorR[pixel] = pixelColorR;
          colorG[pixel] = pixelColorG;
          colorB[pixel] = pixelColorB;

          bar->setPixelColor(pixel, bar->Color(pixelColorR, pixelColorG, pixelColorB));
        }
      }

      //set the leading pixel/led
      bar->setPixelColor(j, bar->Color(R, G, B));
      bar->show();

      colorR[j] = R;
      colorG[j] = G;
      colorB[j] = B;

      delay(100);
    }

    bool areTherePixelsLeftToBeFaded = false;
    pixel = 0;
    pixelColorR = 0;
    pixelColorG = 0;
    pixelColorB = 0;

    //fade out the trailing tail
    for (int j = firstPixel; j < numberOfPixels; j++)
    {
      for (int p = 0; p < numberOfPixels; p++)
      {
        pixel = p;
        
        if ( RandomTrailTaper )
        {
          pixel = random(0, p);
        }
        
        pixelColorR = colorR[pixel];
        pixelColorG = colorG[pixel];
        pixelColorB = colorB[pixel];
        if ( pixelColorR > 0 || pixelColorG > 0 || pixelColorB > 0 )
        {
          areTherePixelsLeftToBeFaded = true;
          pixelColorR = (pixelColorR > 0 ? ( pixelColorR / 2 ) : 0);
          pixelColorG = (pixelColorG > 0 ? ( pixelColorG / 2 ) : 0);
          pixelColorB = (pixelColorB > 0 ? ( pixelColorB / 2 ) : 0);

          colorR[pixel] = pixelColorR;
          colorG[pixel] = pixelColorG;
          colorB[pixel] = pixelColorB;

          bar->setPixelColor(pixel, bar->Color(pixelColorR, pixelColorG, pixelColorB));          
        }
      }

      //set the leading pixel/led
      bar->show();

      delay(100);

      if (!areTherePixelsLeftToBeFaded){break;}

      areTherePixelsLeftToBeFaded = false;
    }

    delete [] colorR;
    delete [] colorG;
    delete [] colorB;

    bar->clear();
    bar->show();
}

void ToggleLightbar(Lightbar LB, Color color, bool TurnOn)
{
  std::tuple<uint8_t, uint8_t, uint8_t> rgb = ToRGB8(color);
  ToggleLightbar(LB, TurnOn, std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb));
}

void ToggleLightbar(Lightbar LB, bool TurnOn, uint8_t R, uint8_t G, uint8_t B)
{
    // default color is WHITE
    Adafruit_NeoPixel *bar;
    int firstPixel = 0;
    int numberOfPixels = 0;
    bool turnOffLBSw = false;
    bool clearBar = true;

    // turn the desired lightbar on or off
    switch (LB)
      {
        case FRONT:
          {
            clearBar = false;
            bar = &_frontLightbar;
            numberOfPixels = NUM_PIXELS_ON_FLB;
            if ( TurnOn ) { digitalWrite(PIN_FLB_SWITCH, HIGH); } 
            else { turnOffLBSw = true; }
            break;
          }
        case REAR:
          {
            // Rear and Front LBs are chained
            clearBar = false;
            bar = &_frontLightbar;
            firstPixel = NUM_PIXELS_ON_FLB;
            numberOfPixels = NUM_PIXELS_ON_FLB + NUM_PIXELS_ON_RLB;
            break;
          }
        case GROUND_EFFECT:
          {
            bar = &_groundEffectLB;
            numberOfPixels = NUM_PIXELS_ON_GELB;
            break;
          }
        case FRONT_AND_REAR:
          {
            bar = &_frontLightbar;
            numberOfPixels = (NUM_PIXELS_ON_FLB + NUM_PIXELS_ON_RLB);
            if ( TurnOn ) { digitalWrite(PIN_FLB_SWITCH, HIGH); } 
            else { turnOffLBSw = true; }
            break;
          }
        case BUILT_IN:
        {
          bar = &_builtInLEDs;
          numberOfPixels = NUM_PIXELS_ON_DB;
          break;
        }
        default:
          {
            bar = &_frontLightbar;
            numberOfPixels = 6;
            if ( TurnOn ) { digitalWrite(PIN_FLB_SWITCH, HIGH); } 
            else { turnOffLBSw = true; }
            break;
          }
      }

    if ( clearBar ) bar->clear();
    
    if (TurnOn) 
    {
        for(int i=firstPixel; i < numberOfPixels; i++)
        {
          bar->setPixelColor(i, bar->Color(R, G, B));
        }
    }
    else
    {
      //don't clear the bar (most likely this is a chained bar, that need to work independantly as well as one), but turn one bar off without changing the other bar
      if ( !clearBar )
      {
        for(int i=firstPixel; i < numberOfPixels; i++)
        {
          bar->setPixelColor(i, bar->Color(0, 0, 0));
        }
      }
    }
    
    bar->show();

    //setting the pin low MUST be called here after the show, setting low prior to the .show will result in low red LEDs shown when the LB should be off
    if ( turnOffLBSw ) digitalWrite(PIN_FLB_SWITCH, LOW);
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

void FlashLightbar(Lightbar LB, int numFlashes, uint8_t R, uint8_t G, uint8_t B)
{
 FlashLightbar(LB, numFlashes, Adafruit_NeoPixel::Color(R, G, B));
}
void FlashLightbar(Lightbar LB, int numFlashes, Color color)
{
  std::tuple<uint8_t, uint8_t, uint8_t> rgb = ToRGB8(color);
  FlashLightbar(LB, numFlashes, std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb));
}

void FlashLightbar(Lightbar LB, int numFlashes, uint32_t color)
{
  // turn desired LB on and then back off to flash it
  for(int i=0; i< numFlashes; i++)
  {
    ToggleLightbar(LB, true, color);
    ToggleLightbar(LB, false);
  }
}

#pragma endregion LEDs

bool IsRunningInDemoMode()
{
  //return false;
  // pins 34 & zero were used for testing...when pin 34 is jumpered to pin zero then 4095 is read
  return ( analogRead(PIN_DEMO_MODE) == 4095 );
}

#pragma region LiDar
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
  pinMode(PIN_SDA_LOX, OUTPUT);           //pin 21    // LiDar data pin
  pinMode(PIN_SCL_LOX, OUTPUT);           //pin 22    // LiDar clock pin

  pinMode(PIN_XSHUT_FRONT_LOX, OUTPUT);   //pin 33
  pinMode(PIN_XSHUT_REAR_LOX, OUTPUT);    //pin 25

  Serial.print("using LiDar: ");
  Serial.println(_useLiDar);

  if ( !_useLiDar ) return;

  Serial.println("Resetting LiDar sensors...");

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

  Serial.println("LiDar sensors are ready...");

  Serial.print("Call begin on front LOX? ");
  Serial.println(_isFrontLidarOn);

  // initing front
  if ( !_isFrontLidarOn )
  {
    if(!_frontLox.begin(FRONT_FACING_LOX_I2C_ADDR, true)) 
    {
      FlashLightbar(BUILT_IN, 1, 255, 255, 0); //yellow
      Serial.println(F("Failed to boot first VL53L0X"));
      _isFrontLidarOn = false;
    }
    else
    {
      Serial.println("Front LiDar is ready...");
      _isFrontLidarOn = true;
    }
    delay(50);
    }

  // activating rear
  //digitalWrite(PIN_XSHUT_FRONT_LOX, LOW);
  if ( !_isRearLidarOn ) digitalWrite(PIN_XSHUT_REAR_LOX, HIGH);
  delay(50);

  Serial.print("Call begin on rear LOX? ");
  Serial.println(_isRearLidarOn);

  //initing rear
  if ( !_isRearLidarOn ) 
  {
    if(!_rearLox.begin(REAR_FACING_LOX_I2C_ADDR)) 
    {
      FlashLightbar(BUILT_IN, 1, 0, 255, 0); //green
      Serial.println(F("Failed to boot rear VL53L0X"));
      _isRearLidarOn = false;
    }
    else
    {
      Serial.println("Rear LiDar is ready...");
      _isRearLidarOn = true;
    }
  }
}

void ReadLidarSensors_Test()
{
  Serial.print("Is LiDar ready (Front:Rear)? ");
  Serial.print(_isFrontLidarOn);
  Serial.print(":");
  Serial.println(_isRearLidarOn);

  if ( _movementRequested ) {Serial.println("Movement since last read...");}
  else                      {Serial.println("No movement since last read...");}
  
  if ( !_isFrontLidarOn || !_isRearLidarOn ) { SetupLidarSensors(); }
  else if ( _movementRequested ) { SetupLidarSensors(); _movementRequested = false; }

  if ( _isFrontLidarOn ) digitalWrite(PIN_XSHUT_FRONT_LOX, LOW);
  if ( _isRearLidarOn ) digitalWrite(PIN_XSHUT_REAR_LOX, HIGH);

  //QUESTION: can these happen back to back or there should be some latching?
  //if ( _isFrontLidarOn ) _frontLox.rangingTest(&_front_LOX_Measure, false); // pass in 'true' to get debug data printout!
  if ( _isRearLidarOn ) _rearLox.rangingTest(&_rear_LOX_Measure, true);   // pass in 'true' to get debug data printout!

  //Serial.print("Reading a measurement... ");

  //if either are on, turn on the red leds
  int redValue = (_frontRedValue + _rearRedValue) > 255 ? 255 : (_frontRedValue + _rearRedValue);

  ToggleLightbar(BUILT_IN, true, redValue, 0, 0);

  bool stopBackward = false;
  int rangeMillis = 0;

  //NOTE:  if the sensor is acting weird, verify the protective file (yellow or orange) has been removed from the face of the LiDar sensor
  if ( _isRearLidarOn )
  {
    Serial.print("Reading Rear LiDar....");
    if (_rear_LOX_Measure.RangeStatus != 4) // phase failures have incorrect data
    { 
      Serial.print("Rear RedVal: "); Serial.println(_rearRedValue);
      rangeMillis = _rear_LOX_Measure.RangeMilliMeter;
      Serial.print("Rear range is (mm): "); Serial.println(rangeMillis);

      if (_rear_LOX_Measure.RangeMilliMeter < 300 ) 
      {
        //if ( measure.RangeMilliMeter <= 127) //within 5"
        if (_rear_LOX_Measure.RangeMilliMeter <= 250) //within 7 1/2", 200 is 7 3/4"
        {
          stopBackward = true;
          _rearRedValue = 255;
        }
        else if ( _rearRedValue < 255)
        {
          _rearRedValue+=5;
        }
      }
      else if(_rear_LOX_Measure.RangeMilliMeter > 300 && _rearRedValue > 0)  //300mm is 11.811"
      {
        //_rearRedValue-=5;
        Serial.println("Nothing close to rear LiDar ");
        _rearRedValue = 0;
      }
    } 
    else 
    {
      Serial.println("rear out of range ");
      _rearRedValue = 0;
    }
  }
  else 
  {
    Serial.println("rear lox is off ");
    _rearRedValue = 0;
  }

  _isRearObstacleDetected = stopBackward;

  Serial.print("Rear obstacle detected? "); Serial.println((_isRearObstacleDetected ? "TRUE" : "FALSE"));

//  if ( (stopBackward && _isMovingBackward) )
//  {       
//    //turn off forward (left side)
//    ledcWrite(1, 0);
//    //turn off reverse (left side)
//    ledcWrite(2, 0);
//
//    //turn off forward (right side)
//    ledcWrite(3, 0);
//    //turn off reverse (right side)
//    ledcWrite(4, 0);
//  }
}

void ReadLidarSensors() 
{
  if ( !_useLiDar ) 
  {
    //BlinkDebugLED(2);
    _isFrontObstacleDetected = false;
    _isRearObstacleDetected = false;
    return;
  }

  Serial.print("Is LiDar ready (Front:Rear)? ");
  Serial.print(_isFrontLidarOn);
  Serial.print(":");
  Serial.println(_isRearLidarOn);

  if ( _movementRequested ) {Serial.println("Movement since last read...");}
  else                      {Serial.println("No movement since last read...");}
  
  if ( !_isFrontLidarOn || !_isRearLidarOn ) { SetupLidarSensors(); }
  else if ( _movementRequested ) { SetupLidarSensors(); _movementRequested = false; }

  if ( _isFrontLidarOn ) _frontLox.rangingTest(&_front_LOX_Measure, false); // pass in 'true' to get debug data printout!
  if ( _isRearLidarOn ) _rearLox.rangingTest(&_rear_LOX_Measure, true);   // pass in 'true' to get debug data printout!

  //Serial.print("Reading a measurement... ");
  //_FrontLox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if ( !_areBuiltInsOn )
  {
    //if either are on, turn on the red leds
    int redValue = (_frontRedValue + _rearRedValue) > 255 ? 255 : (_frontRedValue + _rearRedValue);

    ToggleLightbar(BUILT_IN, true, redValue, 0, 0);
  }

  bool stopForward = false;
  bool stopBackward = false;
  int rangeMillis = 0;

  //NOTE:  if the sensor is acting weird, verify the protective file (yellow or orange) has been removed from the face of the LiDar sensor
  if ( _isFrontLidarOn )
  {
    //Serial.print("Reading Front LiDar....");
    if (_front_LOX_Measure.RangeStatus != 4) 
    { // phase failures have incorrect data
      //Serial.print("Front RedVal: "); Serial.println(_frontRedValue);
      rangeMillis = _front_LOX_Measure.RangeMilliMeter;
      //Serial.print("Front range is (mm): "); Serial.println(rangeMillis);

      if (_front_LOX_Measure.RangeMilliMeter < 300 ) 
      {
        //if ( measure.RangeMilliMeter <= 127) //within 5"
        if (_front_LOX_Measure.RangeMilliMeter <= 250) //within 7 1/2", 200 is 7 3/4"
        {
          stopForward = true;
          _frontRedValue = 255;
        }
        else if ( _frontRedValue < 255)
        {
          _frontRedValue+=5;
        }
      }
      else if(_front_LOX_Measure.RangeMilliMeter > 300 && _frontRedValue > 0)  //300mm is 11.811"
      {
        //_frontRedValue-=5;
        //Serial.println("Nothing close to front LiDar ");
        _frontRedValue = 0;
      }
    } 
    else 
    {
      //Serial.println("front out of range ");
      _frontRedValue = 0;
    }
  }
  else 
  {
    //Serial.println("front lox is off ");
    _frontRedValue = 0;
  }

  rangeMillis = 0;
  
  //NOTE:  if the sensor is acting weird, verify the protective file (yellow or orange) has been removed from the face of the LiDar sensor
  if ( _isRearLidarOn )
  {
    Serial.print("Reading Rear LiDar....");
    if (_rear_LOX_Measure.RangeStatus != 4) 
    { // phase failures have incorrect data
      Serial.print("Rear RedVal: "); Serial.println(_rearRedValue);
      rangeMillis = _rear_LOX_Measure.RangeMilliMeter;
      Serial.print("Rear range is (mm): "); Serial.println(rangeMillis);

      if (_rear_LOX_Measure.RangeMilliMeter < 300 ) 
      {
        //if ( measure.RangeMilliMeter <= 127) //within 5"
        if (_rear_LOX_Measure.RangeMilliMeter <= 250) //within 7 1/2", 200 is 7 3/4"
        {
          stopBackward = true;
          _rearRedValue = 255;
        }
        else if ( _rearRedValue < 255)
        {
          _rearRedValue+=5;
        }
      }
      else if(_rear_LOX_Measure.RangeMilliMeter > 300 && _rearRedValue > 0)  //300mm is 11.811"
      {
        //_rearRedValue-=5;
        Serial.println("Nothing close to rear LiDar ");
        _rearRedValue = 0;
      }
    } 
    else 
    {
      Serial.println("rear out of range ");
      _rearRedValue = 0;
    }
  }
  else 
  {
    Serial.println("rear lox is off ");
    _rearRedValue = 0;
  }

  _isFrontObstacleDetected = stopForward;
  _isRearObstacleDetected = stopBackward;

  Serial.print("Front obstacle detected? "); Serial.println((_isFrontObstacleDetected ? "TRUE" : "FALSE"));
  Serial.print("Rear obstacle detected? "); Serial.println((_isRearObstacleDetected ? "TRUE" : "FALSE"));

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

#pragma endregion LiDar

#pragma region PS4
int _lastR1Value = 0;
int _lastR2Value = 0;
int _lastL1Value = 0;
int _lastL2Value = 0;

void OnNotify()
{
  //BlinkDebugLED(1);

  _leftX = PS4.LStickX();
  _leftY = PS4.LStickY();
  _rightX = PS4.RStickX();
  _rightY = PS4.RStickY();

  if ( PS4.L1() != _lastL1Value )
  {    
      _didL1Change = true;
      _isPS4_L1_Pressed = ( PS4.L1() > 0 );
      //_isPS4_L1_Pressed = true;
      _lastL1Value = PS4.L1();
  }

  if ( PS4.L2Value() != _lastL2Value )
  {    
      _didL2Change = true;
      _isPS4_L2_Pressed = ( PS4.L2Value() > 0 );
  }

  if ( PS4.R1() != _lastR1Value )
  {    
      _didR1Change = true;
      _isPS4_R1_Pressed = ( PS4.R1() > 0 );
      _lastR1Value = PS4.R1();
  }

  if ( PS4.R2Value() != _lastR2Value )
  {    
      _didR2Change = true;
      _isPS4_R2_Pressed = ( PS4.R2Value() > 0 );
  }

//(???) need to had logic here to determine if the button state has changed
  if( PS4.Circle() )
  {
    if ( PS4.Circle() > 0 )
    {
      _didCircleChange = true;
    }
  }

  if( PS4.Triangle() )
  {
    if ( PS4.Triangle() > 0 )
    {
      _didTriangleChange = true;
    }
  }

  if( PS4.Cross() )
  {
    if ( PS4.Cross() > 0 )
    {
      _didCrossChange = true;
    }
  }

  if( PS4.Square() )
  {
    if ( PS4.Square() > 0 )
    {
      _didSquareChange = true;
    }
  }

  if ( PS4.PSButton() )
  {
    _wasPSButtonPressed = true;
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
//    //
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

  if ( _leftY > 5 ) //the joystick is being pushed forward
  {
    Serial.println("going forward...");
    if ( !_isFrontObstacleDetected )
    {
      //turn on forward (left side)
      ledcWrite(1, (abs(_leftY) + 127));
      //turn off reverse (left side)
      ledcWrite(2, 0);
      _isMovingForward = true;
      _isMovingBackward = false;
      _movementRequested = true;
    }
  }
  else if ( _leftY < -5 && !_isRearObstacleDetected ) //the joystick is being pulled aft
  {
    Serial.println("going backward...");

    //turn on forward (left side)
    ledcWrite(1, 0);
    //turn on reverse (left side)
    ledcWrite(2, (abs(_leftY) + 127));
    _isMovingForward = false;
    _isMovingBackward = true;
    _movementRequested = true;
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

  if ( _rightY > 5 ) //the joystick is being pushed forward
  {
    if ( !_isFrontObstacleDetected )
    {
      //turn on forward (right side)
      ledcWrite(3, (abs(_rightY) + 127));
      //turn off reverse (right side)
      ledcWrite(4, 0);
      _isMovingForward = true;
      _isMovingBackward = false;
      _movementRequested = true;
    }
  }
  else if (_rightY < -5 && !_isRearObstacleDetected)  // the joystick is being pulled aft
  {
    //turn on forward (right side)
    ledcWrite(3, 0);
    //turn on reverse (right side)
    ledcWrite(4, (abs(_rightY) + 127));
    _isMovingForward = false;
    _isMovingBackward = true;
    _movementRequested = true;
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
    FlashLightbar(BUILT_IN, 3, 0, 0, 255);
    digitalWrite(PIN_BT_CONNECTED_LED, LOW);

    // Sets the color of the controller's front light
    // Params: Red, Green,and Blue
    // See here for details: https://www.w3schools.com/colors/colors_rgb.asp
    PS4.setLed(0, 0, 255);
    //nextRainbowColor();

    // Sets how fast the controller's front light flashes
    // Params: How long the light is on in ms, how long the light is off in ms
    // Range: 0->2550 ms, Set to 0, 0 for the light to remain on
    //PS4.setFlashRate(PS4.LStickY() * 10, PS4.RStickY() * 10);
    PS4.setFlashRate(250, 250);

    // Sets the rumble of the controllers
    // Params: Weak rumble intensity, Strong rumble intensity
    // Range: 0->255
    //PS4.setRumble(PS4.L2Value(), PS4.R2Value());
    PS4.setRumble(0, 255);

    // Sends data set in the above three instructions to the controller
    PS4.sendToController();

    // Don't send data to the controller immediately, will cause buffer overflow
    Serial.println("Connected...");
    
    //PS4.setRumble(100, 100);
    delay(10);
}

void FlashController(int R, int G, int B)
{
    // Sets the color of the controller's front light
    // Params: Red, Green,and Blue
    // See here for details: https://www.w3schools.com/colors/colors_rgb.asp
    PS4.setLed(R, G, B);

    // Sets how fast the controller's front light flashes
    // Params: How long the light is on in ms, how long the light is off in ms
    // Range: 0->2550 ms, Set to 0, 0 for the light to remain on
    //PS4.setFlashRate(PS4.LStickY() * 10, PS4.RStickY() * 10);
    PS4.setFlashRate(250, 250);

    PS4.setRumble(0, 0);

    PS4.sendToController();
    delay(10);
}

// Calculates the next value in a rainbow sequence
//void nextRainbowColor() {
//  if (r > 0 && b == 0) {
//    r--;
//    g++;
//  }
//  if (g > 0 && r == 0) {
//    g--;
//    b++;
//  }
//  if (b > 0 && g == 0) {
//    r++;
//    b--;
//  }
//}
#pragma endregion PS4