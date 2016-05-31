//#define SIMULATION

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define BUTTON1_PIN   4
#define BUZZER_PIN    11
#define LED           13

#define smoothness 0.35f

#define numberOfSamplesForCalibration  5
#define NUMBER_OF_ALARMS 6

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


#define ONGROUND     1
#define ARMED        2
#define ONALTITUDE   4
#define FREEFALL     8
#define UNDERCANOPY  16
#define LANDED       32

int state = ONGROUND;
bool onAltitude = false;

float GroundLevelPressure = 1023.35;
int prevTime = 0;
float prevAltitude = 0.0f;


void freefallAlarm1();
void freefallAlarm2();
void freefallAlarm3();
void canopyAlarm1();
void canopyAlarm2();
void canopyAlarm3();


typedef void (*alarmPtr)();

typedef struct Alarm
{
  bool triggered;
  float Speed;
  float Altitude;
  alarmPtr alarm;
}_Alarm;

Alarm Alarms[NUMBER_OF_ALARMS]
{
  {false, 30, 1525, &freefallAlarm1 },
  {false, 30, 1100, &freefallAlarm2 },
  {false, 30,  700, &freefallAlarm3 },
  {false,  0,  300, &canopyAlarm1 },
  {false,  0,  200, &canopyAlarm2  },
  {false,  0,  100, &canopyAlarm3  },
};

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  #ifdef SIMULATION
  Serial.begin(9600);
  Serial.println("Pressure Sensor Test"); Serial.println("");
  #endif


  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    digitalWrite(13, HIGH);
    while(1);
  }

  float pressure = 0;
  float avgPressure = 0;
  for(int i = 0; i < numberOfSamplesForCalibration; i++)
  {
    bmp.getPressure(&pressure);  
    avgPressure = avgPressure+(pressure / 100.0F);
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    tone(BUZZER_PIN, 3000);      
    delay(250);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    noTone(BUZZER_PIN);
    delay(200);       
  }
  GroundLevelPressure = avgPressure / numberOfSamplesForCalibration;
//  Serial.println("Ground level pressure"); Serial.println(GroundLevelPressure);
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
  int currTime = millis();
  //Calculate deltaTime, since the previous instance
  int dt = currTime - prevTime;
  #ifdef SIMULATION
  float altitude = getSimulatedAltitude();
  #else
  float altitude = getAltitude();
  #endif
  //Calculate current speed based on prevAltitude.
  //Negative speeds means going up, positive speeds means falling down
  float currSpeed = (prevAltitude - altitude) / ((float)(dt) / 1000.0f);

//  Serial.print("Altitude ");
//  Serial.print(altitude);
//  Serial.print("Speed");
//  Serial.print(currSpeed);
//  Serial.println();
  

  
  prevAltitude = altitude;  //Used to calculate speed
  prevTime = currTime;      //Used to calculate dt

  if (state == ONGROUND && altitude > 300)
  {
    state = ARMED;
    armedAlarm();
  }
  if (state == ARMED && !onAltitude && altitude > 3500 )
  {
    atAltitudeAlarm();
    onAltitude = true;
  }
  else if (state == ARMED && currSpeed > 30)
  {
    state = FREEFALL;
    beep(1000, 3, 50, 300);
  }
  else if (state == FREEFALL && currSpeed < 20)
  {
    state = UNDERCANOPY;
  }
  else if (state == UNDERCANOPY && altitude < 8)
  {
    state = LANDED;
  }


 
  if (state == ONGROUND )
  {
    //It is safe to sleep for 8 secs here until we reach 300m
    //SavePower();
  }
  else if (state == ARMED)
  {
    //It is safe to sleep for 4 secs here until we reach altitude
    delay(1000); // this is important not to accidentally enable FREEFALL MODE
  }
  
  else if (state == FREEFALL || state == UNDERCANOPY )
  {
    for(int i = 0; i < NUMBER_OF_ALARMS; i++)
    {
      if (!Alarms[i].triggered && currSpeed > Alarms[i].Speed && altitude < Alarms[i].Altitude)  
      {
          Alarms[i].triggered = true;
          Alarms[i].alarm();
          break;
      }
    }
  }
  else if (state == LANDED)
  {
    //It is safe to put the device on sleep forever until next reset
    beep(3000, 1, 20, 200);
  }
 
  delay(1000); 
}

void beep(int freq, int count, int on, int off)
{
  for(int i = 0; i < count; i++)
  {
    tone(BUZZER_PIN, freq);      
    delay(on);              // wait for a second
    noTone(BUZZER_PIN);
    delay(off);       
  }
}


void armedAlarm()
{
  beep(3000, 5, 200, 100);
}

void freefallAlarm1()
{
  beep(2000, 5, 200, 100);
}
void freefallAlarm2()
{
  for (int i = 0; i < 4; i++)
  {
    beep(2000, 1, 100, 100);
    beep(3000, 2, 500, 50);
  }
}
void freefallAlarm3()
{
    beep(3000, 20, 50, 50);
}

void canopyAlarm1()
{
    beep(3000, 3, 200, 100);
}
void canopyAlarm2()
{
    beep(3000, 6, 100, 50);
  
}
void canopyAlarm3()
{
    beep(3000, 12, 50, 25);
}

void atAltitudeAlarm()
{
  beep(3000, 5, 200, 100);
}


// watchdog interrupt
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
}  // end of WDT_vect

void SavePower()
{
   // disable ADC
  ADCSRA = 0;  

  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval 
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  noInterrupts ();           // timed sequence follows
  sleep_enable();
 
  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();  
  
  // cancel sleep as a precaution
  sleep_disable();
}


float getAltitude()
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    //Read the current air pressure from the sensor and filter it using a running average filter
    float currentPressure = event.pressure;//(event.pressure * (1.0f - smoothness)) + (currentPressure  *  smoothness);
    //Calculate the altitude based on groundLEvelPRessure
    float altitude  = bmp.pressureToAltitude(GroundLevelPressure, currentPressure);
    return altitude;
    
  }
  else
  {
    //If something is wrong, turn on the built in LED
    digitalWrite(13, HIGH);
    while(true); //Stay here, and stop program
  }

  }

void lowBatteryWarning () 
{
  digitalWrite (LED, HIGH);  
  delay (1);       // mS        
  digitalWrite (LED, LOW);    
  delay (999);             
}

#ifdef SIMULATION
float simSpeed;

float getSimulatedAltitude()
{
   int t = millis();

   float dt = t - prevTime;  
   float s = simSpeed * dt / 1000;
   float altitude  = prevAltitude - s;
  
   if (state == ONGROUND && altitude < 300)
   {
     simSpeed = -40;
   }
//   else if (state == ARMED && altitude > 350)
//   {
//    altitude = 2200;
//    prevAltitude = 2300;
//    simSpeed = 50;
//   }
   else if ((state == ARMED || state == ONALTITUDE) && altitude > 4000)
   {
    prevAltitude = 4300;
    simSpeed = 50;
   }
   if (state == FREEFALL && altitude < 1100)
   {
     simSpeed = 10;
   }
   if (state == UNDERCANOPY && altitude < 5)
   {
     simSpeed = 0;
   }


   Serial.print("Altitude ");
   Serial.print(altitude);
   Serial.print("SimSpeed");
   Serial.print(simSpeed);
   Serial.println();
   
     delay(500);
  
   return altitude;
}
#endif
