#include <PID_v1.h>
#include <max6675.h>

/*  Pin Connections
 *  LAMP - FAN RElays
 *  TEMP SENSOR
 */
#define RELAY1 2 //Lamp relay
#define RELAY2 3 //Fan relay
#define CLK 4
#define CS 5
#define DBIT 6

/*  Profile Definitions
 *  These definitions select heating and cooling profiles
 *  Also control stability duration
 */
//#define SLOW_HEATING
#define AGGRESSIVE_HEATING
//#define SLOW_COOLING
#define AGGRESSIVE_COOLING
//#define DEBUG_MODE
#define RESULT_MODE

unsigned long STABILITY_TIME = 240000; //4 min
/* PID Parameters */
double Setpoint, Input, Output;
double Kp=1950, Ki=25, Kd=4;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 2500;
unsigned long windowStartTime;

/* Thermocouple Instance */
MAX6675 thermocouple (CLK, CS, DBIT);

/* State Variable */
enum state { Heating, Steady, Cooling, Off };
enum state oven_state;
int kick_threshold = 0;  //  degrees
unsigned long stability_timer;
bool recorded_time = 0;

void setup() {
  pinMode(CLK, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(DBIT, INPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  
  digitalWrite(CS, HIGH);
  digitalWrite(CLK, LOW);
  
  Serial.begin(9600);

  #ifdef DEBUG_MODE
    Serial.println("PROGRAM STARTED");
  
    #ifdef SLOW_COOLING
      Serial.print("SLOW COOLING - ");
    #endif

    #ifdef AGGRESSIVE_COOLING
      Serial.print("AGGRESSIVE COOLING - ");
    #endif

    Serial.print("STEADY TIME: ");
    Serial.print(STABILITY_TIME);

    #ifdef SLOW_HEATING
      Serial.println("SLOW HEATING");
    #endif

    #ifdef AGGRESSIVE_HEATING
      Serial.println("AGGRESSIVE HEATING");
    #endif

    Serial.println("**************************");
  
  #endif

  windowStartTime = millis();
  //initialize the variables we're linked to
  Setpoint = 50; //50 o-C
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  oven_state = Heating;   //Startup State
}

void loop() {
  switch(oven_state)
  {
    case (Heating):
      #ifdef DEBUG_MODE
        Serial.println("Entered Heating Case");
      #endif

      #ifdef SLOW_HEATING
        digitalWrite(RELAY1,LOW);// lamp on
        digitalWrite(RELAY2,LOW); // fan on
        Input= sensor2reading()*0.264; //Get sensor reading
        
        if ( (Setpoint - Input ) <= kick_threshold){  //Checks if we are close to the threshold
          oven_state = Steady;
        }
        break;
      #endif

      #ifdef AGGRESSIVE_HEATING
        digitalWrite(RELAY2,HIGH);// fan off
        digitalWrite(RELAY1,LOW);// lamp on

        Input= sensor2reading()*0.264; //Get sensor reading
        
        if ((Setpoint - Input ) <= kick_threshold){  //Checks if we are close to the threshold
          oven_state = Steady;
        }
        break;
      #endif

    case (Steady):
      #ifdef DEBUG_MODE
        Serial.println("Entered Steady Case");
      #endif

      Input= sensor2reading()*0.264; //Get sensor reading

      if (!recorded_time)   //This block should only be executed once in the whole program
      {
         stability_timer = millis(); //Record the time when the steady state started
         recorded_time = 1;
      }
      
      myPID.Compute();
      
      if ((millis() - windowStartTime) > WindowSize)
      { //time to shift the Relay Window
      windowStartTime += WindowSize;
      }
      if (Output < (millis() - windowStartTime)) {
        digitalWrite(RELAY1,HIGH); // fan on
        digitalWrite(RELAY2,LOW);
      }
      else if (Output >= (millis() - windowStartTime) ) {
        digitalWrite(RELAY1,LOW); // lamp on 
        digitalWrite(RELAY2,HIGH);
      }

      if ( (millis() - stability_timer) >= STABILITY_TIME ) { //Check that STABILITY_TIME minutes were elapsed
        oven_state = Cooling; 
      }
      break;

    case (Cooling):
      #ifdef DEBUG_MODE
        Serial.println("Entered Cooling Case");
      #endif

      #ifdef SLOW_COOLING
        digitalWrite(RELAY1,HIGH);// lamp off
        digitalWrite(RELAY2,HIGH); // fan off
        Input= sensor2reading()*0.264; //Get sensor reading
        
        if ( Input <= 36 ){  //Checks if we are close to the threshold
          oven_state = Off;
        }
        break;
      #endif

      #ifdef AGGRESSIVE_COOLING
        digitalWrite(RELAY1,HIGH);// lamp off
        digitalWrite(RELAY2,LOW); // fan on
        Input= sensor2reading()*0.264; //Get sensor reading
        
        if ( Input <= 36  ){  //Checks if we are close to the threshold
          oven_state = Off;
        }
        break;
      #endif

    case (Off):
      #ifdef DEBUG_MODE
        Serial.println("Entered OFF Case");
      #endif
      digitalWrite(RELAY1,HIGH);// lamp off
      digitalWrite(RELAY2,HIGH); // fan off
      break;
      
    default:
      break;
  }
  #ifdef RESULT_MODE
    Input= sensor2reading()*0.264;
    Serial.println(Input);
  #endif
}

/* Getting Sensor Raw Data and Conversion to Reading in C */
double sensor2reading() {
  //Serial.println("Thermocouple is working: ");
  
  int value = 0;
  digitalWrite(CS, LOW);
  delay(2);
  digitalWrite(CS, HIGH);
  delay(220);
  
  /* Read the chip and return the raw temperature value */
  /* Bring CS pin low to allow us to read the data from
    the conversion process */
  digitalWrite(CS, LOW);
  
  /* Cycle the clock for dummy bit 15 */
  digitalWrite(CLK, HIGH);
  delay(1);
  digitalWrite(CLK, LOW);
  
  /* Read bits 14-3 from MAX6675 for the Temp. Loop for each bit reading
    the value and storing the final value in 'temp'*/
  for (int l = 14; l >= 0; l--) {
    digitalWrite(CLK, HIGH);
    value += digitalRead(DBIT) << l;
    digitalWrite(CLK, LOW);
  }
  
  // check bit D2 if HIGH no sensor
  if ((value & 0x04) == 0x04) return -1;
  // shift right three places
  return value >> 3;
}
