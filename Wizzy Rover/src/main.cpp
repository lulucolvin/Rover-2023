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
#define NUM_PIXELS_ON_FLB     6     // We have 6 LEDs on the front lightbar (FLB)
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

enum Lightbar { FRONT, REAR, GROUND_EFFECT, FRONT_AND_REAR };
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
Adafruit_NeoPixel _frontLightbar(NUM_PIXELS_ON_FLB, PIN_PIXELS_FLB, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel _rearLightbar(NUM_PIXELS_ON_RLB, PIN_PIXELS_RLB, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel _groundEffectLB(NUM_PIXELS_ON_GELB, PIN_PIXELS_GELB, NEO_GRB + NEO_KHZ800);

Adafruit_VL53L0X _frontLox = Adafruit_VL53L0X();
Adafruit_VL53L0X _rearLox = Adafruit_VL53L0X();
// this holds the lidar measurement 
VL53L0X_RangingMeasurementData_t _front_LOX_Measure;
VL53L0X_RangingMeasurementData_t _rear_LOX_Measure;

void BlinkDebugLED(int BlinkXTimes);
void LightTheGE(uint8_t R, uint8_t G, uint8_t B);
void SetupGE();
void loopGE();
void Chaser(uint8_t R, uint8_t G, uint8_t B, Lightbar LB, bool RandomTrailTaper = false);

void SetupGE()
{
  //pinMode(PIN_PIXELS_GELB, OUTPUT);
  //digitalWrite(PIN_PIXELS_GELB, LOW);

  _groundEffectLB.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  _groundEffectLB.setBrightness(MAX_LB_BRIGHTNESS); // Full brightness

  delay(250);

  LightTheGE(0, 0, 255);
  delay(1000);
}

void setup()
{
  pinMode(PIN_DEMO_MODE, INPUT_PULLDOWN); //pin 34

  pinMode(PIN_DEBUG_LED, OUTPUT);         //pin 32
  pinMode(PIN_BT_CONNECTED_LED, OUTPUT);  //pin 26

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_PHOTORESISTOR, INPUT);      //pin 35

  pinMode(PIN_XSHUT_FRONT_LOX, OUTPUT);   //pin 33
  pinMode(PIN_XSHUT_REAR_LOX, OUTPUT);    //pin 25

  pinMode(PIN_PIXELS_GELB, OUTPUT);       //pin 23
  digitalWrite(PIN_PIXELS_GELB, LOW);

  //this is the switch, high allows the front LB to turn on
  pinMode(PIN_FLB_SWITCH, OUTPUT);        //pin 14

  pinMode(PIN_PIXELS_FLB, OUTPUT);        //pin 27
  digitalWrite(PIN_PIXELS_FLB, LOW);
  pinMode(PIN_PIXELS_RLB, OUTPUT);        //pin 15
  digitalWrite(PIN_PIXELS_RLB, LOW);

  digitalWrite(PIN_DEBUG_LED, HIGH);
  digitalWrite(PIN_BT_CONNECTED_LED, HIGH);
  delay(500);

  digitalWrite(PIN_PIXELS_GELB, LOW);

  _isRedWhiteOrBlue = RED;
  _nextColor = WHITE;

  BlinkDebugLED(1);
  delay(1000);
  SetupGE();
}

void loopGE()
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

  Chaser(red,white,blue, GROUND_EFFECT, false);

  _isRedWhiteOrBlue = _nextColor;
}

void loop()
{
    loopGE();

    BlinkDebugLED(10);
    delay(500);
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

void Chaser(uint8_t R, uint8_t G, uint8_t B, Lightbar LB, bool RandomTrailTaper)
{  
    int numberOfPixels;
    Adafruit_NeoPixel* bar = &_groundEffectLB;

    RandomTrailTaper = false;

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
          bar = &_rearLightbar;
          numberOfPixels = NUM_PIXELS_ON_RLB;
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
      default:
        {
          bar = &_frontLightbar;
          numberOfPixels = 6;
          break;
        }
    }

    //BlinkDebugLED(2);

    uint8_t *colors{new uint8_t[numberOfPixels]};

    //colors = new uint8_t[numberOfPixels];
    memset(colors, 0, numberOfPixels);

    long pixel = 0;
    uint8_t pixelColor = 0;

    //go from left to right
    for (int j = 0; j < numberOfPixels; j++)
    {
      for (int p = 0; p < numberOfPixels; p++)
      {       
        pixel = p;

        if ( RandomTrailTaper )
        {
          pixel = random(0, p);
        }
        
        pixelColor = colors[pixel];

        if ( pixelColor > 0 )
        {
          pixelColor = (pixelColor/2);
          colors[pixel] = pixelColor;
          switch (_isRedWhiteOrBlue)
          {
            case RED:
            {
              bar->setPixelColor(pixel, bar->Color(pixelColor, G, B));
			        bar->show();
              break;
            }
            case WHITE:
            {
              bar->setPixelColor(pixel, bar->Color(pixelColor, pixelColor, pixelColor));
			        bar->show();
              break;
            }
            default:
            {
              bar->setPixelColor(pixel, bar->Color(R, G, pixelColor));
 			        bar->show();
              break;
            }
          }
        }
      }

	    //set the leading pixel/led
      bar->setPixelColor(j, bar->Color(R, G, B));
      bar->show();
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
    for (int j = numberOfPixels; j >= 0; j--)
    {
        for (int p = 0; p < numberOfPixels; p++)
        {
          pixel = p;
          
          if ( RandomTrailTaper )
          {
            pixel = random(0, p);
          }
          
          pixelColor = colors[pixel];
          if ( pixelColor > 0 )
          {
            pixelColor = (pixelColor/2);
            colors[pixel] = pixelColor;
            switch (_isRedWhiteOrBlue)
            {
              case RED:
              {
                bar->setPixelColor(pixel, bar->Color(pixelColor, G, B));
				        bar->show();
                break;
              }
              case WHITE:
              {
                bar->setPixelColor(pixel, bar->Color(pixelColor, pixelColor, pixelColor));
				        bar->show();
                break;
              }
              default:
              {
                bar->setPixelColor(pixel, bar->Color(R, G, pixelColor));
				        bar->show();
                break;
              }
            }
          }
        }

	    //set the leading pixel/led
      bar->setPixelColor(j, bar->Color(R, G, B));
      bar->show();

      delay(100);
    }

    bool areTherePixelsLeftToBeFaded = false;

    //fade out the trailing tail
    for (int j = 0; j < numberOfPixels; j++)
    {
        for (int p = 0; p < numberOfPixels; p++)
        {
          pixel = p;
          
          if ( RandomTrailTaper )
          {
            pixel = random(0, p);
          }
          
          pixelColor = colors[pixel];
          if ( pixelColor > 0 )
          {
            areTherePixelsLeftToBeFaded = true;
            pixelColor = (pixelColor/2);
            colors[pixel] = pixelColor;
            switch (_isRedWhiteOrBlue)
            {
              case RED:
              {
                bar->setPixelColor(pixel, bar->Color(pixelColor, G, B));
				        bar->show();
                break;
              }
              case WHITE:
              {
                bar->setPixelColor(pixel, bar->Color(pixelColor, pixelColor, pixelColor));
				        bar->show();
                break;
              }
              default:
              {
                bar->setPixelColor(pixel, bar->Color(R, G, pixelColor));
				        bar->show();
                break;
              }
            }
          }
        }

	    //set the leading pixel/led
      bar->setPixelColor(j, bar->Color(R, G, B));
      bar->show();

      delay(100);

      if (!areTherePixelsLeftToBeFaded){break;}

      areTherePixelsLeftToBeFaded = false;
    }

    delete [] colors;
}

void LightTheGE(uint8_t R, uint8_t G, uint8_t B)
{
    //setup the lightbar to show, but don't turn it on just yet
    for (int j = 0; j < NUM_PIXELS_ON_GELB; j++)
    {
      _groundEffectLB.setPixelColor(j, _groundEffectLB.Color(R, G, B));
    }
}

