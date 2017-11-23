/**Low Cost Underwater Vehicle - AGUS GROUP RESEARCH of UNMANNED SYSTEM
 * Made by : Naufalino Fadel Hutomo & Muhammad Hanif
 * Controller for pitch motion using linear actuator as buoyancy system and elevator control surface.
 * Inpput from APM channel output. APM -> Arduino -> Actuator
 */

#include <PID_v1.h>

#define chElPin 2 ////Pin elevator from ch 2 rx. Min 1000 Max 2000 Center 1500
#define chModePin 3 //Pin mode from ch 5 rx. Min 1000 Max 2000 Center 1500
#define elevPin 5 // pin to elevator surface control
#define LA_PWM 6 //linear actuator pwm signal L
//#define RPWM 9 //linear actuator pwm signal R
#define L_EN 7 //linear actuator enable L
#define R_EN 8 //linear actuator enable R

#define trigPin 10 //trigger pin for ultrasonic
#define echoPin 11 //trigger pin for ultrasonic


#define limMode 1200 //GANTI dengan batas pwm mode untuk buoy. < lim bakal buoy
 
/**CHANGE - Depend on Calibration / initial setup in GCS */
const int sigMin = 1000;
const int sigMax = 2000;
const int sigCen = 1500;
const double LA_bal = 5; //length of LA to balance the system
const double LA_min = 0; //minimum length LA
const double LA_max = 15; //maximum length LA

/** PID CONSTANTS**/
const double Kp = 3, Ki = 0, Kd = 10;

volatile int pwmEl = 0;
volatile int prev_timeEl = 0;
volatile int elevator;

volatile int pwmMode = 0;
volatile int prev_timeMode = 0;
volatile char mode = 'c'; //buoy mode and cruise mode

double LA_set = LA_bal;
double LA_dist;
double LA_out;
PID myPID (&LA_dist, &LA_out, &LA_set,Kp,Ki,Kd, DIRECT );

long duration;

void setup() {
  // when pin D2 goes high, call the rising function
  attachInterrupt(chElPin, risingEl, RISING);
  attachInterrupt(chModePin, risingMode, RISING);

  //pin Mode
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(LA_PWM, OUTPUT);
  pinMode(elevPin, OUTPUT);
  pinMode (trigPin, OUTPUT);
  pinMode (trigPin, INPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}
 
void loop() {
  if (mode == 'b')
  {
    digitalWrite(elevPin, LOW); //turn off elevator
    //Compute Set point
    LA_set = setLA();
    
    //Measure distance with ultrasonic sensor
    digitalWrite(trigPin,LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    LA_dist = (duration/2) / 29.1;

    // PID Computation
    myPID.Compute();

    //PWM Output 
    if(LA_dist <= LA_set ) {
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, HIGH);
        analogWrite(LA_PWM,LA_out);
        //analogWrite(RPWM,0);
    }
    else {
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, LOW);
        analogWrite(LA_PWM,LA_out);
        //analogWrite(LPWM,0);
        //analogWrite(RPWM,LA_out);
    }
  }else
  if (mode =='c')
  {
    //turn off linear actator
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(LA_PWM,0);
    
    digitalWrite(elevPin, digitalRead(elevPin)); //bypass the signal
  }
  
}


//ISR for elevator input
void risingEl() {
  attachInterrupt(chElPin, fallingEl, FALLING);
  prev_timeEl = micros();
}
 
void fallingEl() {
  attachInterrupt(chElPin, risingEl, RISING);
  pwmEl = micros()-prev_timeEl;
  Serial.println(pwmEl);
}

//ISR for elevator input
void risingMode() {
  attachInterrupt(chModePin, fallingMode, FALLING);
  prev_timeMode = micros();
}
 
void fallingMode() {
  attachInterrupt(chModePin, risingMode, RISING);
  pwmMode = micros()-prev_timeMode;
  Serial.println(pwmMode);
  if (pwmMode < limMode)
  {
    mode = 'b'; //buoy mode
  }else
  {
    mode = 'c'; //cruise mode
  } 
}

double setLA ()  //compute set point from PWM signal
{
  double ret;

  if ( (pwmEl < sigCen+50) and (pwmEl > sigCen-50) )
  {
    ret = LA_bal;
  }else if (pwmEl > (sigCen+50) )
  {
    ret = LA_bal + (pwmEl - sigCen)/(sigMax-sigCen) * (LA_max-LA_bal); 
  }else if (pwmEl < (sigCen - 50) )
  {
    ret = LA_bal - (sigCen - pwmEl)/(sigCen - sigMin) * (LA_bal- LA_min);
  }

  return ret;
}

