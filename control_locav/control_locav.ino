/**Low Cost Underwater Vehicle - AGUS GROUP RESEARCH of UNMANNED SYSTEM
 * Made by : Naufalino Fadel Hutomo & Muhammad Hanif
 * Controller for pitch motion using linear actuator as buoyancy system and elevator control surface.
 * Inpput from APM channel output. APM -> Arduino -> Actuator
 */

#include <PID_v1.h>

#define chElPin 2 ////Pin elevator from ch 2 rx. Min 1000 Max 2000 Center 1500
#define chModePin 3 //Pin mode from ch 5 rx. Min 1000 Max 2000 Center 1500
#define elevPin 5 // pin to elevator surface control, ch out
#define LA_PWM 6 //linear actuator pwm signal L, ch out
#define RPWM 9 //linear actuator pwm signal R
#define L_EN 7 //linear actuator enable L
#define R_EN 8 //linear actuator enable R

#define trigPin 2 //trigger pin for ultrasonic
#define echoPin 3 //trigger pin for ultrasonic

#define elTol 100 // tolerance for ch2 out pwm stable
//bit flags
#define EL_FLAG 1
#define MODE_FLAG 2
volatile uint8_t bUpdateFlagsShared; //hold the update flags bit

 
/**CHANGE - Depend on Calibration / initial setup in GCS */
const uint16_t limMode[3] = {1490, 1620, 1749}; //GANTI dengan batas pwm mode untuk berbagai mode [float, drown, cruise, manual]. < lim bakal buoy
const double LA_bal = 5; //length of LA to balance the system
const double LA_min = 0; //minimum length LA
const double LA_max = 15; //maximum length LA
const uint16_t sigMin = 984;
const uint16_t sigMax = 2012;
uint16_t sigCen = 1400;

/** PID CONSTANTS**/
const double Kp = 47, Ki = 0, Kd = 10;


volatile uint16_t pwmEl;
volatile uint32_t prev_timeEl;
volatile uint16_t elevator;

volatile uint16_t pwmMode;
volatile uint32_t prev_timeMode;
volatile char mode = 'm'; //m = manual, drown, cruise, float
volatile char prev_mode = 'm';

double LA_set = LA_bal;
double LA_dist;
double LA_out;
uint8_t LA_pwm;

PID myPID (&LA_dist, &LA_out, &LA_set,Kp,Ki,Kd, DIRECT );

long duration;

void setup() {
  // when pin D2 goes high, call the rising function
  attachInterrupt(digitalPinToInterrupt(chElPin), isrEl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(chModePin), isrMode, CHANGE);

  //pin Mode
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(LA_PWM, OUTPUT);
  //pinMode(elevPin, OUTPUT);
  pinMode (trigPin, OUTPUT);
  pinMode (trigPin, INPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  Serial.begin(9600);

  delay(1000);
  sigCen = pwmEl; //calibration
  Serial.print("calibrated");
  Serial.print("sigCen = ");
  Serial.print(sigCen);
  delay(2000);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}
 
void loop() {
  // local variable
  static uint16_t elIn;
  static uint16_t modeIn;
  static uint8_t bUpdateFlags;
  
  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    // only copy when the flags tell us we can.
   
    if(bUpdateFlags & EL_FLAG)
    {
      elIn = pwmEl;
    }
   
    if(bUpdateFlags & MODE_FLAG)
    {
      modeIn = pwmMode;
    }
    
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
   
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    
  }
  
  currentMode(); //decide the current mode
  
  if (mode =='c')
  {
    cruise();
  }else{
    buoyMode(mode);
  }
    Serial.print("  mode: ");
    Serial.print(mode);
    Serial.print("\t PWM Elev : ");
    Serial.print(pwmEl);
    Serial.print("\t LA_dist : ");
    Serial.print(LA_dist);
    Serial.print("  LA_out ");
    Serial.print(LA_out);
    Serial.print("  LA_set ");
    Serial.print(LA_set);
    Serial.print("\t R_EN : ");
    Serial.print(digitalRead(R_EN));
    Serial.print("\t L_EN : ");
    Serial.println(digitalRead(L_EN));
 
}


//ISR for elevator input
void isrEl() {
  if (digitalRead(chElPin)==HIGH){
    prev_timeEl = micros(); 
  }else{
    pwmEl = (uint16_t) (micros()-prev_timeEl);  
    bUpdateFlagsShared |= EL_FLAG;
  }
}
/* 
void fallingEl() {
  attachInterrupt(chElPin, risingEl, RISING);
  pwmEl = micros()-prev_timeEl;
  Serial.println(pwmEl);
}
*/

//ISR for elevator input
void isrMode() {
  if(digitalRead(chModePin)==HIGH){
    prev_timeMode = micros();  
  }else{
    pwmMode = (uint16_t)(micros()-prev_timeMode);  
    bUpdateFlagsShared |= MODE_FLAG;
  }
}

 /*
void fallingMode() {
  attachInterrupt(chModePin, risingMode, RISING);
  pwmMode = micros()-prev_timeMode;
  Serial.println(pwmMode);
}*/

double setLA ()  //compute set point from PWM signal
{
  double ret;
  
  if ( (pwmEl < sigCen+elTol) and (pwmEl > sigCen-elTol) )
  {
    ret = LA_bal;
  }else if (pwmEl >= (sigCen+elTol) )
  {
    ret = LA_bal + ( (double) (pwmEl - sigCen))/ (double)(sigMax-sigCen) * (LA_max-LA_bal); 
  }else if (pwmEl <= (sigCen - elTol) )
  {
    ret = LA_bal - ((double) (sigCen - pwmEl))/ (double)(sigCen - sigMin) * (LA_bal- LA_min);
  }

  return ret;
}

void cruise() //service routine while cruise
{
    //turn off linear actuator
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(LA_PWM,0);
    if(prev_mode != 'c'){
      prev_mode = 'c';  
    }
    
}

void currentMode (){
  //decide what current mode from RC CH5 Input 
  
  if (pwmMode < limMode[0]){   //change movement mode to buoy or cruise
      mode = 'f'; //float mode
    }else if( pwmMode < limMode[1]){
      mode = 'd'; //drown mode
    } else if(pwmMode < limMode[2]){
      mode = 'c';  //cruise
      digitalWrite(elevPin, digitalRead(chElPin)); //bypass the signal
    } else{
      mode = 'm'; //manual
    }  
}

uint8_t calcLA ()//calculate necessary PWM signal to move LA
{
  double dif = abs(LA_set - LA_dist);
  uint8_t ret;
  
  if (dif > 5){
    ret = 255; 
  } else{
    ret = (uint8_t) (255/ LA_bal * dif); //51 = 255/5
  }
  Serial.print("calcLA ");
  Serial.print(ret);
  return ret;
}

void manualMode(double f){    // looping routine for manual mode
  //Compute Set point
    LA_set = f;
    //LA_set = 25;
    
    /*//Measure distance with ultrasonic sensor
    digitalWrite(trigPin,LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    LA_dist = (duration/2) / 29.1;
  */
    LA_dist = LA_bal;
    
    // PID Computation
    myPID.Compute();
    LA_pwm = calcLA();

    
    //PWM Output 
    if(LA_dist >= LA_set ) {
        //digitalWrite(R_EN, LOW);
        //digitalWrite(L_EN, HIGH);
        analogWrite(LA_PWM,LA_pwm);
        analogWrite(RPWM,0);
    }
    else {
        //digitalWrite(R_EN, HIGH);
       // digitalWrite(L_EN, LOW);
        //analogWrite(LA_PWM,LA_out);
        analogWrite(LA_PWM,0);
        analogWrite(RPWM,LA_pwm);
    }
}

void drownMode(){   //looping routin for drowning
  LA_pwm = 255;
  analogWrite(LA_PWM,LA_pwm);
  analogWrite(RPWM,0);
  delay (10000);  //drowning for 10 seconds
   if(prev_mode != 'd'){
      prev_mode = 'd';  
   }
}


void floatMode(){   //looping routin for floating
  LA_pwm = 255;
  analogWrite(LA_PWM,0);
  analogWrite(RPWM,LA_pwm);
  delay (10000);  //floating for 10 seconds
  if(prev_mode != 'f'){
      prev_mode = 'f';  
  }
}

void buoyMode (char m) //routine mode while buoy/ not cruise
{
    digitalWrite(elevPin, LOW); //turn off elevator
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    switch (m){
      case 'm' :     
         manualMode((double) (setLA()));
          if(prev_mode != 'm'){
            prev_mode = 'm';  
          }              
                break;
      case 'd' :  
        if (prev_mode == 'd'){
            manualMode(LA_bal);
         }else{
           drownMode();
         }
         break;
      case 'f':
        if (prev_mode == 'f'){
            manualMode(LA_bal);
         }else{
           floatMode();
         }
         break;
       default: manualMode((double) (setLA() ) );
    }

}
