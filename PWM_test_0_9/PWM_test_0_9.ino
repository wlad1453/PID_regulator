  /*
 * 
 *
 * 
 *ROM = 28 6D 89 75 D0 1 3C 46  - black waterproof sensor
 */

#include <OneWire.h> 
#include "DallasTemperature.h"

#define SENS_PIN 4
#define PWM_PIN 11
#define RELAY_PIN 7
#define HEATING 0
#define COOLING 1

#define SET_T 45.0
#define HYSTER 2.0
#define STEP 2.0             

OneWire oneWire(SENS_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress tSensor = { 0x28, 0x6D, 0x89, 0x75, 0xD0, 0x1, 0x3C, 0x46 };

struct REGULATOR {
    float setTemp = SET_T;         // set value (degrees C)
    float hysteresis = HYSTER;
    float temp;                   // imput sygnal (temperature)
    
    float K = 50;                  // Algorithm factor    / gain factor
    float _dt_S = STEP;           // differentiation step (Seconds)
    bool _direction = HEATING;    // Working direction (heating or cooling)
    bool switchOn = false;         // Relay status (Control signal)

} Heater;

struct FILLTER {
    float Value1;         // set value (degrees C)
    float Value2;
    float Value3;                   // imput sygnal (temperature)
    
    float Average(float T) {
      Value3 = Value2;
      Value2 = Value1;
      Value1 = T;
      
      return ( Value1 + Value2 + Value3 )/ 3;
    }

    void setUp(float T) {
      Value1 = Value2 = Value3 = T;
    }
   
} averT;

void setup()
{
  averT.setUp(Heater.setTemp);
  
  Serial.begin(9600);

  pinMode( PWM_PIN, OUTPUT );
  pinMode( RELAY_PIN, OUTPUT );
  
  sensors.begin();
  sensors.setResolution(12);
  
  plot(Heater, 1);
  
  // Serial.println("Temperature    , Heating,   SetT,   T-Hys,   T+Hys");  // This could be trnsfered into plot() function
   
}


void loop()
{ 
  sensors.requestTemperatures(); // Send the command to get temperatures
  Heater.temp = averT.Average(sensors.getTempC(tSensor));    // Temp after heat exchange 

  Heater.switchOn = relayControl(Heater);
  digitalWrite( RELAY_PIN, Heater.switchOn );

  plot(Heater, 0);
  
  analogWrite( PWM_PIN, int(Heater.temp / 40 * 255) );      // For further analog regulating purposes
 
  /******* Plain algorithm *******
  if ( temp <= ( setTemp - hysteresis / 2 ) ) switchOn = true;
  else if ( temp >= ( setTemp + hysteresis / 2 ) ) switchOn = false;
  digitalWrite( RELAY_PIN, switchOn );
  */

  
  /*
  Serial.print( Heater.temp ); 
  Serial.print (",  ");
  // Serial.print( ( 8 + int(Heater.switchOn) ) * Heater.setTemp / 10 );        // int(temp / 40 * 255)
  Serial.print( ( 8 + Heater.switchOn ) * Heater.setTemp / 10 );
  Serial.print (",  ");
  Serial.print( Heater.setTemp ); 
  Serial.print (",  ");
  Serial.print( Heater.setTemp - Heater.hysteresis / 2 ); 
  Serial.print (",  ");
  Serial.println( Heater.setTemp + Heater.hysteresis / 2  ); 

  Inp = Serial.readString();
  if ( Inp.length() > 0 && Inp.toFloat() ) Heater.K = Inp.toFloat(); // If there is an input AND this input contains meaningful number
  
  */
  
  delay( Heater._dt_S * 1000 );

}

void plot(REGULATOR &H, bool capture) {

  if (capture) {
    Serial.println("Temperature    , Heating,   SetT,   T-Hys,   T+Hys");
  } else {    
    String Inp = "";
    int num = 0;
        
    Serial.print( H.temp ); 
    Serial.print (",  ");
    Serial.print( ( 10 + H.switchOn ) * H.setTemp / 12 );
    Serial.print (",  ");
    Serial.print( H.setTemp ); 
    Serial.print (",  ");
    Serial.print( H.setTemp - H.hysteresis / 2 ); 
    Serial.print (",  ");
    Serial.println( H.setTemp + H.hysteresis / 2  ); 
  
    Inp = Serial.readString();
    if ( Inp.length() > 0 && Inp.toFloat() ) H.K = Inp.toFloat(); // If there is an input AND this input contains meaningful number
  } 
}

boolean relayControl(REGULATOR &H) {
  float signal, rate;
  static float prevValue;
  boolean output;
  
  if (H.K > 0) {
    
    // float rate = (H.temp - prevValue) / H._dt_S;    

    float rate = ( H.temp - prevValue ) / H._dt_S;        // производная от величины (величина/секунду)
    prevValue = H.temp;
    signal = H.temp + rate * H.K;
  } else {
    signal = H.temp;
  }
  /*
  int8_t F = (sign(signal - setpoint - hysteresis / 2) + sign(signal - setpoint + hysteresis / 2)) / 2;
  if (F == 1) output = _direction;
  else if (F == -1) output = !_direction; */
  
  if     ( (( signal - H.setTemp - H.hysteresis / 2) > 0 ) && ((signal - H.setTemp + H.hysteresis / 2) > 0) ) output =  H._direction;
  else if( (( signal - H.setTemp - H.hysteresis / 2) < 0 ) && ((signal - H.setTemp + H.hysteresis / 2) < 0) ) output = !H._direction;
  
  return output;
}
