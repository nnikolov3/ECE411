#include <FastLED.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69s
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high


#define LED_PIN     5
#define NUM_LEDS    12
#define BRIGHTNESS  255
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB


//Smoothing globals

  const int numReadings = 10;
  
  int readingsx[numReadings];      // the readings from the analog input
  int readingsy[numReadings];      // the readings from the analog input
  //int readingsz[numReadings];      // the readings from the analog input
  
  int readIndex = 0;              // the index of the current reading
  
  float totalax = 0;                  // the running total
  float totalay = 0;                  // the running total
 // float totalaz = 0;                  // the running total
  
  float averageax = 0;                // the average
  float averageay = 0;                // the average
 // float averageaz = 0;                // the average


  float neg_averageax =0;
  float neg_averageay =0;



int LedsPosX = 0;
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

enum state_enum{IDLE1,MODE11,MODE12,MODE0};
int CHARGING = 2; //double check

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;


int16_t ax, ay, az;
int16_t gx, gy, gz;
state_enum currentState = IDLE1;


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

/*
void idle(); //Cube on charge stand, lights off
void mode11(); //Cube is free from charge stand, lights are on
void mode12(); //cube is free form charge stand, lights are off
void mode0();  //Cube on charge stand, lights are on

void MeasureGyro(bool gyro_on);
void OutputLight(bool lights_on);
*/
bool CUBE_FLIPPED = false;
bool blinkState = false;

void setup() {
   
   Serial.begin(38400);
    pinMode(CHARGING,INPUT);
    
    delay( 3000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );
    
    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;


  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)


    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
   
    
    accelgyro.setXAccelOffset(46);
    accelgyro.setYAccelOffset(-981);
    accelgyro.setZAccelOffset(1342);
    accelgyro.setXGyroOffset(51);
    accelgyro.setYGyroOffset(-12);
    accelgyro.setZGyroOffset(-5);
  /*
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output


}

void loop() {


switch(currentState)

{
  case IDLE1: if(digitalRead(CHARGING))
  { idle(); 
  Serial.print("S: IDLE1");
  }
  
  else 
  {currentState = MODE11; mode11(); 
  Serial.print("S: MODE11");
  };
  
  break;

  case MODE11: if(!CUBE_FLIPPED && !digitalRead(CHARGING)) 
  {mode11(); Serial.print("S: Disconnected from Charger, lights on \n");}
  
  else if(CUBE_FLIPPED)
  {CUBE_FLIPPED = false; modeChange(); currentState = MODE12;  mode12(); Serial.print("S: MODE12");}
  
  else if(digitalRead(CHARGING))
  {currentState = MODE0; mode0();Serial.print("S: Charging, lights on \n");};
  break;
  
  case MODE12: if(!CUBE_FLIPPED  && !digitalRead(CHARGING))
  {mode12();Serial.print("S: Disconnected from Charger, lights off \n");}
  
  else if(CUBE_FLIPPED)
  {CUBE_FLIPPED = false; modeChange(); currentState = MODE11; mode11();Serial.print("S: Disconnected from charger, lighhts on (2) \n");}
  
  else if(digitalRead(CHARGING))
  {currentState = IDLE1; idle();Serial.print("S: Charging, lights off\n");}
  break;
  

  case MODE0: if(digitalRead(CHARGING))
  {mode0();Serial.print("Charging, lights on (2) \n");}
  else
  {currentState = MODE11;mode11();Serial.print("S: Disconnected from charger, lighhts on (3)\n");}
  break;

  }


    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);


        /*// First, clear the existing led values
        FastLED.clear();
        for(int led = 0; led < 9; led++) { 
            //leds[led] = CRGB::Red; 
            leds[led].setRGB(0,ColorMapToAX,ColorMapToAY);
        }
        FastLED.show();
    */

  

   
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
       // Serial.print(ay); Serial.print("\t");
       // Serial.print(az); Serial.print("\t");
       // Serial.print(gx); Serial.print("\t");
       // Serial.print(gy); Serial.print("\t");
       // Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

}

void idle()
{
  MeasureGyro(false);
  OutputLight(false);
    return;
}

void mode11()
{
  MeasureGyro(true);
  OutputLight(true);
  return;
}

void mode12()
{
  MeasureGyro(true);
  OutputLight(false);
  return;
}


void mode0()
{
  MeasureGyro(false);
  OutputLight(true);
  return;
}


void MeasureGyro(bool gyro_on)
{



  
  if(gyro_on)
  {
    // read raw accel/gyro measurements from device
    
    totalax = totalax - readingsx[readIndex];
    totalay = totalay - readingsy[readIndex];
    //totalaz = totalaz - readingsz[readIndex];
    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    readingsx[readIndex] = ax;
    readingsy[readIndex] = ay;
   // readingsz[readIndex] = az;
    
    totalax = totalax + readingsx[readIndex];
    totalay = totalay + readingsy[readIndex];
    //totalaz = totalaz + readingsz[readIndex];
    
    readIndex=readIndex+1;

    if(readIndex >= numReadings) { readIndex = 0;}
    
    averageax = totalax/numReadings;
    averageay = totalay/numReadings;
   // averageaz = totalaz/numReadings;
    
    Serial.print("Average: ");
    Serial.print(averageax);
    

    

    if(az < -15000)
    {
      CUBE_FLIPPED = true;
    }
  }
  else
  {
    return;
  }
  
  
  
  return;
}







  
void OutputLight(bool light_on)
{


//Modify how strongly the lights responds to movement
const float Sensitivity = 3;
  
 if (light_on)
  
      {
        
          ChangePalettePeriodically();
    
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
        
        
        
        /*
        
            if (averageax < 0)
              {
                neg_averageax = -averageax;
                averageax = 0;
              }
        
             if (averageay < 0)
             {
              neg_averageay = -averageay;
              averageay = 0;
             }
  
  int LedsPosY  = map(averageay, 0, 16200, 0, 255);
  int LedsPosX = map(averageax, 0, 16200, 0, 255);
  int LedsNegY = map(neg_averageay, 0, 16200, 0, 255);
  int LedsNegX  = map(neg_averageax, 0, 16200, 0, 255);

  LedsPosY = LedsPosY*Sensitivity;
  LedsPosX = LedsPosX*Sensitivity;
  LedsNegY = LedsNegY*Sensitivity;
  LedsNegX = LedsNegX*Sensitivity;
  
//limit output to prevent overflow brightness
  if (LedsPosY > 255)
    LedsPosY = 255;
    
    if (LedsPosX > 255)
    LedsPosX = 255;
    
    if (LedsNegY > 255)
    LedsNegY = 255;
    
    if (LedsNegX > 255)
    LedsNegX = 255;

        Serial.print("\t");
        Serial.print(LedsPosY); Serial.print("\t");
        Serial.print(LedsPosX); Serial.print("\t");
        Serial.print(LedsNegY); Serial.print("\t");
        Serial.print(LedsNegX); Serial.print("\t");

      int ledintensity[12] = {0};


      ledintensity[0] = .5*LedsPosX + .5*LedsPosY;
      ledintensity[1] = .5*LedsPosX + .35*LedsPosY;
      ledintensity[2] = .5*LedsPosX + .35*LedsNegY;
      ledintensity[3] = .5*LedsPosX + .5*LedsNegY;
      ledintensity[4] = .5*LedsNegY + .35*LedsPosY;
      ledintensity[5] = .5*LedsNegY + .35*LedsNegX;
      ledintensity[6] = .5*LedsNegY + .5*LedsNegX;
      ledintensity[7] = .5*LedsNegX + .35*LedsNegY;
      ledintensity[8] = .5*LedsNegX + .35*LedsPosY;
      ledintensity[9] = .5*LedsNegX + .5*LedsPosY;
      ledintensity[10] = .5*LedsPosY + .35*LedsNegX;
      ledintensity[11] = .5*LedsPosY + .35*LedsPosX;

      
for (int i = 0; i < 12; i++)
      {
        leds[i] = CRGB::White;
        leds[i] -= CRGB(0, ledintensity[i], ledintensity[i]);
        leds[i] += CRGB(ledintensity[i], 0, 0);
        }
        FastLED.show();*/
 }


 
 
  else{
    for (int i=0; i < 12; i++)
    {
      leds[i] = CRGB::Black;
      
    }
    FastLED.show();
  }
  
  return;

}

void modeChange()
{
  ax = 0;
  ay = 0;
  az = 0;

    for (int i = 0; i < 12; i++)
  {
    leds[i] = CRGB::Black;
    FastLED.show();
  }
        FastLED.show();
  for (int i = 0; i < 12; i++)
  {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(200);
    leds[i] = CRGB::Black;
    FastLED.show();
  }
  


}

// This example shows several ways to set up and use 'palettes' of colors
// with FastLED.
//
// These compact palettes provide an easy way to re-colorize your
// animation on the fly, quickly, easily, and with low overhead.
//
// USING palettes is MUCH simpler in practice than in theory, so first just
// run this sketch, and watch the pretty lights as you then read through
// the code.  Although this sketch has eight (or more) different color schemes,
// the entire sketch compiles down to about 6.5K on AVR.
//
// FastLED provides a few pre-configured color palettes, and makes it
// extremely easy to make up your own color schemes with palettes.
//
// Some notes on the more abstract 'theory and practice' of
// FastLED compact palettes are at the bottom of this file.




void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}


// There are several different palettes of colors demonstrated here.
//
// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
//
// Additionally, you can manually define your own color palettes, or you can write
// code that creates color palettes on the fly.  All are shown here.

void ChangePalettePeriodically()
{
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;
    
    if( lastSecond != secondHand) {
        lastSecond = secondHand;
        if( secondHand ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
        if( secondHand == 10)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
        if( secondHand == 15)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
        if( secondHand == 20)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
        if( secondHand == 25)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
        if( secondHand == 30)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
        if( secondHand == 35)  { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
        if( secondHand == 40)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 45)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
        if( secondHand == 55)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
    }
}

// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette()
{
    for( int i = 0; i < 16; i++) {
        currentPalette[i] = CHSV( random8(), 255, random8());
    }
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette()
{
    // 'black out' all 16 palette entries...
    fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    
}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB black  = CRGB::Black;
    
    currentPalette = CRGBPalette16(
                                   green,  green,  black,  black,
                                   purple, purple, black,  black,
                                   green,  green,  black,  black,
                                   purple, purple, black,  black );
}


// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};



// Additional notes on FastLED compact palettes:
//
// Normally, in computer graphics, the palette (or "color lookup table")
// has 256 entries, each containing a specific 24-bit RGB color.  You can then
// index into the color palette using a simple 8-bit (one byte) value.
// A 256-entry color palette takes up 768 bytes of RAM, which on Arduino
// is quite possibly "too many" bytes.
//
// FastLED does offer traditional 256-element palettes, for setups that
// can afford the 768-byte cost in RAM.
//
// However, FastLED also offers a compact alternative.  FastLED offers
// palettes that store 16 distinct entries, but can be accessed AS IF
// they actually have 256 entries; this is accomplished by interpolating
// between the 16 explicit entries to create fifteen intermediate palette
// entries between each pair.
//
// So for example, if you set the first two explicit entries of a compact 
// palette to Green (0,255,0) and Blue (0,0,255), and then retrieved 
// the first sixteen entries from the virtual palette (of 256), you'd get
// Green, followed by a smooth gradient from green-to-blue, and then Blue.
