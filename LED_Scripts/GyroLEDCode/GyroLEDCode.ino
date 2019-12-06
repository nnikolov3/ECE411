#include <FastLED.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

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

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN     5
#define NUM_LEDS    12
#define BRIGHTNESS  255
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define INTERRUPT_PIN 3
#define CHARGING 2



//Smoothing globals
  const float Sensitivity = 2;
  const int numReadings = 10;
  int readIndex = 0;              // the index of the current reading
  
  int readingsx[numReadings];      // the readings from the analog input
  int readingsy[numReadings];      // the readings from the analog input
  //int readingsz[numReadings];      // the readings from the analog input
  


int axcounter = 0;
int axxcounter = 0;
int aycounter = 0;
int ayycounter = 0;

  // MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion r;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 

//Modify how strongly the lights responds to movement
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

enum state_enum{IDLE1,MODE11,MODE12,MODE0};


CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

int16_t ax, ay, az;
int16_t gx, gy, gz;

state_enum currentState = IDLE1;


bool CUBE_FLIPPED = false;
bool blinkState = false;
bool LEFT_ON = false;
bool RIGHT_ON = false;
bool LEFT_ON2 = false;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
   
   Serial.begin(115200);
    pinMode(CHARGING,INPUT);
    pinMode(INTERRUPT_PIN, INPUT);
    delay( 3000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );
    
    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties


  Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
        

    while (Serial.available() && Serial.read()); // empty buffer
    
    devStatus = accelgyro.dmpInitialize();
    // initialize device


    // use the code below to change accel/gyro offset values
    Serial.println("Updating internal sensor offsets...");
    /*accelgyro.setXAccelOffset(46);  //with niko
    accelgyro.setYAccelOffset(-981);
    accelgyro.setZAccelOffset(1342);
    accelgyro.setXGyroOffset(51);
    accelgyro.setYGyroOffset(-12);
    accelgyro.setZGyroOffset(-5);*/
    
    accelgyro.setXAccelOffset(300);
    accelgyro.setYAccelOffset(-1048);
    accelgyro.setZAccelOffset(1368);
    accelgyro.setXGyroOffset(52);
    accelgyro.setYGyroOffset(-11);
    accelgyro.setZGyroOffset(-5);

    
 // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        accelgyro.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        accelgyro.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = accelgyro.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = accelgyro.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {

   
      if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = accelgyro.getFIFOCount();
          
///////////////////////////////////////////////////////////////////////////////
//                  STATE MACHINE 
///////////////////////////////////////////////////////////////////////////////
switch(currentState)

{
  case IDLE1: if(deboCharging())
  { idle(); 
  Serial.print("S: IDLE1");
  }
  
  else 
  {currentState = MODE11; mode11(); 
  Serial.print("S: MODE11");
  };
  break;
  case MODE11: if(!CUBE_FLIPPED && !deboCharging()) 
  {mode11(); Serial.print("S: Disconnected from Charger, lights on \n");}
  
  else if(CUBE_FLIPPED)
  {CUBE_FLIPPED = false; modeChange(); currentState = MODE12;  mode12(); Serial.print("S: MODE12");}
  
  else if(deboCharging())
  {currentState = MODE0; mode0();Serial.print("S: Charging, lights on \n");};
  break;
  
  case MODE12: if(!CUBE_FLIPPED  && !deboCharging())
  {mode12();Serial.print("S: Disconnected from Charger, lights off \n");}
  
  else if(CUBE_FLIPPED)
  {CUBE_FLIPPED = false; modeChange(); currentState = MODE11; mode11();Serial.print("S: Disconnected from charger, lighhts on (2) \n");}
  
  else if(deboCharging())
  {currentState = IDLE1; idle();Serial.print("S: Charging, lights off\n");}
  break;
  case MODE0: if(deboCharging())
  {mode0();Serial.print("Charging, lights on (2) \n");}
  else
  {currentState = MODE11;mode11();Serial.print("S: Disconnected from charger, lighhts on (3)\n");}
  break;
  } // end switch state;
        }
    }  

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = accelgyro.getIntStatus();

    // get current FIFO count
    fifoCount = accelgyro.getFIFOCount();
   if(fifoCount < packetSize){
          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        accelgyro.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
    while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
      accelgyro.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }
            accelgyro.dmpGetQuaternion(&r, fifoBuffer);
            accelgyro.dmpGetGravity(&gravity, &r);
            accelgyro.dmpGetYawPitchRoll(ypr, &r, &gravity); 
            }
}  //end of loop()


int deboCharging()
{
  if (digitalRead(CHARGING))
      {
          delay(10);
          if (digitalRead(CHARGING))
            return HIGH;
      }
   else return LOW;
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
      Serial.print("\t");
        Serial.print(ypr[0]*(180/M_PI)); Serial.print("\t");
        Serial.print(ypr[1]*(180/M_PI)); Serial.print("\t");
        Serial.print(ypr[2]*(180/M_PI)); Serial.print("\t");

        if (ypr[1] > 1.4)
        {
              axcounter = axcounter + 1;
        }
        else
          axcounter = 0;
         if (axcounter > 30)
            {
              modeChange();
              
              if (LEFT_ON&&!LEFT_ON2)
              {
                LEFT_ON2 = true;
                Serial.print("LEFT_ON2 = true");}
               else
                  {LEFT_ON2 = false;
                  Serial.print("LEFT_ON2 = false");}
                  
              LEFT_ON = true;
              RIGHT_ON = false;
              axcounter = 0;
              axxcounter = 0;
              ayycounter = 0;
              aycounter = 0;

            }
            

        if (ypr[1] < -1.4)
        {
              axxcounter = axxcounter + 1;
        }
        else
          axxcounter = 0;
         if (axxcounter > 30)
            {
              modeChange();
              LEFT_ON = false;
              LEFT_ON2 = false;
              RIGHT_ON = true;
              axcounter = 0;
              axxcounter = 0;
              ayycounter = 0;
              aycounter = 0;
            }
            

        if (ypr[2] > 1.4)
        {
              aycounter = aycounter + 1;
        }
        else
          aycounter = 0;
         if (aycounter > 30)
            {
              modeChange();
              LEFT_ON = false;
              LEFT_ON2 = false;
              RIGHT_ON = false;
              axcounter = 0;
              axxcounter = 0;
              ayycounter = 0;
              aycounter = 0;
            }


      if (ypr[2] < -1.4)
        {
              ayycounter = ayycounter + 1;
        }
        else
          ayycounter = 0;
         if (ayycounter > 30)
            {
              modeChange();
              LEFT_ON = true;
              RIGHT_ON = true;
               LEFT_ON2 = false;
              axcounter = 0;
              axxcounter = 0;
              ayycounter = 0;
              aycounter = 0;
            }
        }
        
    else
   {
     return;
   }


        /* Serial.print("axcounter:    ");
         Serial.print(axcounter);
         Serial.print("\t");
         Serial.print("aycounter:    ");
         Serial.print(aycounter);
        Serial.print("\t");*/

   
  return;
}



void OutputLight(bool light_on)
{
    if (light_on)
    
        {
          if (LEFT_ON&&RIGHT_ON)
          {
            WarmGlowLedsON();
          }
          else if (LEFT_ON)
          {
             RotationLedsON();
          }
          
          else if (RIGHT_ON)
          {
            OffLedsON();
          }

          else
          {
          DefaultLedsON();
          }
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
    for (int i = 0; i < 12; i++)
  {
    leds[i] = CRGB::Black;
  }
        FastLED.show();
  for (int i = 0; i < 12; i++)
  {
    leds[i] = CRGB::Green;
   FastLED.show();
    delay(100);
    leds[i] = CRGB::Black;
   FastLED.show();   
  }

  return;
}


void DefaultLedsON()
{
  float n_ypr[3] = {0};
            // get rid of negative numbers to make mapping work
            if (ypr[1] < 0)
              {
                n_ypr[1] = -ypr[1];
                ypr[1] = 0;
              }
              
             if (ypr[2] < 0)
             {
              n_ypr[2] = -ypr[2];
              ypr[2] = 0;
             }

        
//map each side to color intensity
  int LedsPosY  = map(ypr[2]*(180/M_PI), 0, 70, 0, 255);
  int LedsPosX = map(ypr[1]*(180/M_PI), 0, 70, 0, 255);
  int LedsNegY = map(n_ypr[2]*(180/M_PI), 0, 70, 0, 255);
  int LedsNegX  = map(n_ypr[1]*(180/M_PI), 0, 70, 0, 255);

        /*Serial.print("\t");
        Serial.print(LedsPosY); Serial.print("\t");
        Serial.print(LedsPosX); Serial.print("\t");
        Serial.print(LedsNegY); Serial.print("\t");
        Serial.print( LedsNegX); Serial.print("\t");*/
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



      int ledintensity[12] = {0};

//map sides to colors, and in some colors on the corners aswell
      ledintensity[0] = LedsPosX + LedsPosY;
      ledintensity[1] = LedsPosX + .1*LedsPosY;
      ledintensity[2] = LedsPosX + .1*LedsNegY;
      ledintensity[3] = LedsPosX + LedsNegY;
      ledintensity[4] = LedsNegY + .1*LedsPosX;
      ledintensity[5] = LedsNegY + .1*LedsNegX;
      ledintensity[6] = LedsNegY + LedsNegX;
      ledintensity[7] = LedsNegX + .1*LedsNegY;
      ledintensity[8] = LedsNegX + .1*LedsPosY;
      ledintensity[9] = LedsNegX + LedsPosY;
      ledintensity[10] = LedsPosY + .1*LedsNegX;
      ledintensity[11] = LedsPosY + .1*LedsPosX;



for (int i = 0; i < 12; i++)
      {
        if (ledintensity[i] > 255)
          ledintensity[i] = 255;
        leds[i] = CRGB::Black;
        leds[i] += CRGB(0,0,10);
        leds[i] += CRGB(ledintensity[i], 0, 0);
        
        if (i == 1 || i == 2)
        {
          leds[i] -= CRGB(0, 0, ledintensity[i]);
        }
        else if (i == 4 || i == 5)
        {
        leds[i] -= CRGB(ledintensity[i], 0, ledintensity[i]);
        leds[i] += CRGB(0, 0, ledintensity[i]);
        }
        else if (i == 7 || i == 8)
        {
          leds[i] -= CRGB(ledintensity[i], 0, ledintensity[i]);
          leds[i] += CRGB(0, ledintensity[i], 0);
        }
        else if (i == 10 || i == 11)
        {
          leds[i] -= CRGB(0, 0, ledintensity[i]);
          leds[i] += CRGB(0, ledintensity[i], 0);
        }
        }
        FastLED.show();
        return;
}

void RotationLedsON()
{
     if ((ypr[0] * 180/M_PI) < 0)
        ypr[0] = 360 + (ypr[0] * (180/M_PI));
      else
        ypr[0] = ypr[0] * (180/M_PI);

     if (!LEFT_ON2)
     {
        for (int i=0; i < 12; i++)
            {
              leds[i] = CRGB::Black;
              leds[i] = ledpick(i);
            }
     }
     else if (LEFT_ON2)
     {
          for (int i=0; i < 12; i++)
            {
              leds[i] = CRGB::Black;
              leds[i] = ledpick_two(i);
           }
     }
            FastLED.show();
}


/*
//led light rotation from FASTLED library
void RaindowsLedsON()
{
          ChangePalettePeriodically();
    
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; // motion speed 
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
   FastLED.delay(1000 / UPDATES_PER_SECOND);
        
        return;
  
}*/


void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}


void OffLedsON()
{
  
  for (int i = 0; i < 12; i++)
  {
    leds[i] = CRGB::Black;
  }
}

void WarmGlowLedsON()
{
   for (int i = 0; i < 12; i++)
  {
    leds[i] = CRGB(255,255,100);
    FastLED.show();
  }
}


CRGB ledpick(int i)
{
  int j = 11 - i;
  int i_ypr = ypr[0];
  if (ypr[0] > (j) * 30 && ypr[0] < (j) * 30+30)
  {
    return CRGB::Green;
  }
  else if (ypr[0] > (j+1) * 30 && ypr[0] < (j+1) * 30+30)
  {
    return CRGB::Red;
  }
  
  else if (ypr[0] > (j-1) * 30 && ypr[0] < (j-1) * 30+30)
  {
    return CRGB::Blue; 
  }
  
  else 
    return CRGB::Black;
}




//roation logic
CRGB ledpick_two(int i)
{
  int j = 11 - i;
  int i_ypr = ypr[0];
  int granR;
  int granG;
  int granB;
  if (ypr[0] > (j) * 30 && ypr[0] < (j) * 30+30)
  {
    
    granR = map(i_ypr%30, 0, 30, 255, 128);
    granG = map(i_ypr%30, 0, 30, 0, 128);
    return CRGB( granR, granG, 0);   
  }

else if (ypr[0] > (j+1) * 30 && ypr[0] < (j+1) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 128, 0);
    granG = map(i_ypr%30, 0, 30, 128, 255);
    return CRGB( granR, granG, 0);   
  }

else if (ypr[0] > (j+2) * 30 && ypr[0] < (j+2) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 255, 128);
    granB = map(i_ypr%30, 0, 30, 0, 128);
    return CRGB( 0, granG, granB);   
  }

else if (ypr[0] > (j+3) * 30 && ypr[0] < (j+3) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 128, 0);
    granB = map(i_ypr%30, 0, 30, 128, 255);
    return CRGB( 0, granG, granB);   
  }
  
else if (ypr[0] > (j+4) * 30 && ypr[0] < (j+4) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 0, 128);
    granB = map(i_ypr%30, 0, 30, 255, 128);
    return CRGB( granR, 0, granB);
  }

else if (ypr[0] > (j+5) * 30 && ypr[0] < (j+5) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 128, 255);
    granB = map(i_ypr%30, 0, 30, 128, 0);
    return CRGB( granR, 0, granB);   
  }
else  if (ypr[0] > (j+6) * 30 && ypr[0] < (j+6) * 30+30)
  {
    
    granR = map(i_ypr%30, 0, 30, 255, 128);
    granG = map(i_ypr%30, 0, 30, 0, 128);
    return CRGB( granR, granG, 0);   
  }
else if (ypr[0] > (j+7) * 30 && ypr[0] < (j+7) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 128, 0);
    granG = map(i_ypr%30, 0, 30, 128, 255);
    return CRGB( granR, granG, 0);   
  }

else if (ypr[0] > (j+8) * 30 && ypr[0] < (j+8) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 255, 128);
    granB = map(i_ypr%30, 0, 30, 0, 128);
    return CRGB( 0, granG, granB);   
  }

else if (ypr[0] > (j+9) * 30 && ypr[0] < (j+9) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 128, 0);
    granB = map(i_ypr%30, 0, 30, 128, 255);
    return CRGB( 0, granG, granB);   
  }
  
else if (ypr[0] > (j+10) * 30 && ypr[0] < (j+10) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 0, 128);
    granB = map(i_ypr%30, 0, 30, 255, 128);
    return CRGB( granR, 0, granB);
  }

else if (ypr[0] > (j+11) * 30 && ypr[0] < (j+11) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 128, 255);
    granB = map(i_ypr%30, 0, 30, 128, 0);
    return CRGB( granR,0, granB);   
  }

else if (ypr[0] > (j-1) * 30 && ypr[0] < (j-1) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 128, 255);
    granB = map(i_ypr%30, 0, 30, 128, 0);
    return CRGB( granR, 0, granB);   
  }

else if (ypr[0] > (j-2) * 30 && ypr[0] < (j-2) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 0, 128);
    granB = map(i_ypr%30, 0, 30, 255, 128);
    return CRGB( granR, 0, granB);   
  }

else if (ypr[0] > (j-3) * 30 && ypr[0] < (j-3) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 128, 0);
    granB = map(i_ypr%30, 0, 30, 128, 255);
    return CRGB( 0, granG, granB);   
  }
  
else if (ypr[0] > (j-4) * 30 && ypr[0] < (j-4) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 255, 128);
    granB = map(i_ypr%30, 0, 30, 0, 128);
    return CRGB( 0, granG, granB);
  }

else if (ypr[0] > (j-5) * 30 && ypr[0] < (j-5) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 128, 0);
    granG = map(i_ypr%30, 0, 30, 128, 255);
    return CRGB( granR, granG, 0);   
  }
  
else  if (ypr[0] > (j-6) * 30 && ypr[0] < (j-6) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 255, 128);
    granG = map(i_ypr%30, 0, 30, 0, 128);
    return CRGB( granR, granG, 0);   
  }
else if (ypr[0] > (j-7) * 30 && ypr[0] < (j-7) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 128, 255);
    granB = map(i_ypr%30, 0, 30, 128, 0);
    return CRGB( granR, 0, granB);   
  }

else if (ypr[0] > (j-8) * 30 && ypr[0] < (j-8) * 30+30)
  {
    granR = map(i_ypr%30, 0, 30, 0, 128);
    granB = map(i_ypr%30, 0, 30, 255, 128);
    return CRGB( granR, 0, granB);   
  }

else if (ypr[0] > (j-9) * 30 && ypr[0] < (j-9) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 128, 0);
    granB = map(i_ypr%30, 0, 30, 128, 255);
    return CRGB( 0, granG, granB);   
  }
  
else if (ypr[0] > (j-10) * 30 && ypr[0] < (j-10) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 255, 128);
    granB = map(i_ypr%30, 0, 30, 0, 128);
    return CRGB( 0, granG, granB);
  }

else if (ypr[0] > (j-11) * 30 && ypr[0] < (j-11) * 30+30)
  {
    granG = map(i_ypr%30, 0, 30, 128, 255);
    granR = map(i_ypr%30, 0, 30, 128, 0);
    return CRGB( granR, granG, 0);   
 }
  else
    return CRGB::Black;
}
