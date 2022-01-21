 /*
 * 
 * https://alexgyver.ru/lessons/pid/
 * 
 * ROM = 28 6D 89 75 D0 1 3C 46  - black waterproof sensor
 */

#include <OneWire.h> 
#include "DallasTemperature.h"

#define SENS_PIN 4
#define PWM_PIN 3

#define HEATING 0
#define COOLING 1

#define SET_T 45.0
#define HYSTER 2.0
#define STEP 2.0             

OneWire oneWire(SENS_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress tSensor = { 0x28, 0x6D, 0x89, 0x75, 0xD0, 0x1, 0x3C, 0x46 };

struct REGULATOR {
    float setTemp = SET_T;          // set value (degrees C)
    float temp;                     // imput sygnal (temperature)

    float Kp = 150;                 // Proportional factor / gain factor 1:100, 2:70, 3:150
    float Ki = 2;                   // Integral factor  1:5 (76S), 2:0.5 (190S), 3:2.0
    float Kd = 2000;                // Differential factor 1:500, 2:1250, 3:2500 4:1250
    float _dt_S = STEP;             // differentiation step (Seconds)
    bool  _direction = HEATING;     // Working direction (heating or cooling)
    bool  switchOn = false;         // Relay status (Control signal)
    int   PWM_level = 0;

} Heater;

struct FILTER {
  float k = 0.1;                        // коэффициент фильтрации, 0.0-1.0
  static const int NUM_READ = 10;       // количество усреднений для средних арифм. фильтров
  
  private:
      float valArray[NUM_READ];
  
  public:  
    void setUp (float value) {
      for (int i = 0; i < NUM_READ; i++) valArray[i] = value;
    }
   
    
    float runningAverage(float newVal) {  // *** running average method ***
      static byte idx = 0;                
            
      valArray[idx] = newVal;                                       // each time in the new cell of the "ring" register                                          
      if (++idx >= NUM_READ) idx = 0;                               // "ring" register increment and reset
      
      float average = 0;                  
      for (int i = 0; i < NUM_READ; i++) average += valArray[i];    // summ of all members
      return (float)average / NUM_READ;   // average calculation
    } // *** running Average ***

    float runningAverageOptim(float newVal) {    // *** optimal running average ***
      static int t = 0;
      static int vals[NUM_READ];
      static int average = 0;
      if (++t >= NUM_READ) t = 0; // ring register reset
      average -= vals[t];         // old value substruction from the current cell 
      average += newVal;          // new value addition
      vals[t] = newVal;           // saving the value into the array
      return ((float)average / NUM_READ);
    } // *** optimal running Average ***
    
    float expRunningAverage(float newVal) {     // *** exponential running average ***
      static float filVal = 0;
      filVal += (newVal - filVal) * k;
      return filVal;
    }      // *** exponential running average ***
} filterT; // object of the structure FILTER 

void setup()
{
  pinMode( PWM_PIN, OUTPUT );
  // TCCR2B = (TCCR2B & 0xF8) | 1; // 
  TCCR2B = 0b00000001;  // x1
  TCCR2A = 0b00000011;  // 
  
  Serial.begin(57600);
  sensors.begin();
  sensors.setResolution(12);
  

  sensors.requestTemperatures();                                        // First reading of the  temperature values
  Heater.temp = sensors.getTempC(tSensor);   

  filterT.setUp(Heater.temp);                                          // SetUp procedure  
  
  plot(Heater, 1);                // Captures of charts should be printed first
     
} // end setup()


void loop()
{ 
  static unsigned long tPoint;

  if ( millis() - tPoint >= Heater._dt_S * 1000 ) {                       
    tPoint = millis();
  
    sensors.requestTemperatures();                                        // Command to read temperature values
    Heater.temp = filterT.runningAverage( sensors.getTempC(tSensor) );   // Getting measurement results and carriing out the digital filtering  
                                                                          
    PWM_control( Heater, PWM_PIN );                   // Calculation of PID parameters, detemination of the pwm control level 
                                                      // and sending it to the pwm generator at PWM_PIN                                                        
    plot( Heater, 0);                           
    readCoeff( Heater, 1 );
  }
} // end loop()

void PWM_control( REGULATOR &H, int pwmPin ) {
  float delta(0);
  static float prevDelta(0);
  static int lastPWM;
  static float Icomp(0.0);
  float Pcomp (0.0), Dcomp(0.0);
  int targetPWM, currentPWM;
  
  delta = H.setTemp - H.temp;
  /* P-component
   * Pcomp = delta * Kp;
   *  
   * I-comonent
   * Icomp = delta * _dt_S;
   * Icomp *= Ki;
   * 
   * D-component
   * Dcomp = ( lastDelta - delta ) / _dt_S;
   * Dcomp *= Kd;
   * 
   * kP = 0.6 * kP1               // kP = 150                                 100
   * kI = kP / T * 2 * dt         // t = 76S, kI = 150 / 76 * 2 * 2 = 8       2   (200S)
   * kD = kP * T / 8 / dt         // kD = 150 * 76 / 8 / 2 = 750              1250(200S)
   */
  
  Pcomp = constrain( delta * H.Kp, - 510.0, + 510.0);                       // Proportial component
  Icomp = constrain( Icomp + delta * H._dt_S * H.Ki, -510.0, 510.0); // iComponent( Heater, delta );          // Integral component
  Dcomp = (delta - prevDelta) / H._dt_S * H.Kd;                             // Differential componennt
  prevDelta = delta;
  
  Dcomp = filterT.runningAverageOptim(Dcomp);

  Serial.print( 40.0 + float(Pcomp) / 50.0 );   Serial.print (",  ");
  Serial.print( 40.0 + constrain( float(Icomp), -510.0, +510.0 ) / 50.0 );   Serial.print (",  ");
  Serial.print( 40.0 + constrain( float(Dcomp), -510.0, +510.0 ) / 50.0 );   Serial.print (",  ");
    
  targetPWM = constrain( (int)( Pcomp + Icomp + Dcomp + 0.5 ), 0, 255 );    

  currentPWM = targetPWM;
  lastPWM = currentPWM;
  H.PWM_level = currentPWM;
  
  analogWrite( pwmPin, currentPWM );  
} // end PWMcontrol()

void plot( REGULATOR &H, bool capture ) {
  String data = "";

  if (capture) {
    Serial.println("P_comp, I_comp, D_comp, PWMcomm, Temp., SetT, ZeroPWM");
  } else {        
    data = String( 40.0 + float(H.PWM_level) / 50.0 ) + ", " + String(H.temp) + ", " + String(H.setTemp) + ", " + String(40) + "\n";
    
    Serial.print( data );  
  } 
} // end plot()

void readCoeff( REGULATOR &H, uint8_t coeffNum ) {

    String Inp = "";
  
    Inp = Serial.readString();
    if ( Inp.length() > 0 && Inp.toFloat() ) H.Kp = Inp.toFloat(); // If there is an input AND this input contains meaningful number
}
