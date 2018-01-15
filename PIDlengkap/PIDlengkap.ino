/*========================================= Tugas Besar SKD ============================================ */
/*===========================Linear Actuator for Buoyancy Engine Motor Position Control =================*/

#include <PID_v1.h>
#define chElPin 2 ////Pin elevator from ch 2 rx. Min 1000 Max 2000 Center 1500
#define elevPin 8 // pin to elevator surface control, ch out
#define LPWM A0 //linear actuator pwm signal L, ch out
#define RPWM 13 //linear actuator pwm signal R
#define L_EN A1 //linear actuator enable L
//#define R_EN 8 //linear actuator enable R
//#define B_PWM A2 //BilgePump pwm signal

#define trigPin 12 //trigger pin for ultrasonic
#define echoPin 11 //trigger pin for ultrasonic

/*========================================Deklarasi Variabel=============================================*/

//Variabel & Konstanta PID
double Input, Output;
double Setpoint = 50;
double Kp=47, Ki=0, Kd=10;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


double duration, distance;

/*========================================Program Utama============================================*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(L_EN, OUTPUT);
  //pinMode(R_EN, OUTPUT);
  pinMode(LPWM, OUTPUT);
  //pinMode(elevPin, OUTPUT);
  //pinMode (trigPin, OUTPUT);
  //pinMode (trigPin, INPUT);

  digitalWrite(L_EN,HIGH);
  
  myPID.SetMode(AUTOMATIC); //turn the PID on
}


void loop() {

  //ULTRASONIC SENSOR
  /*
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  
  //PID Compute
  Input = distance;
  myPID.Compute();
  
  //PWM Output 
  if(distance <= Setpoint ) {
      analogWrite(LPWM,Output);
      analogWrite(RPWM,0);
  }
  else {
      analogWrite(LPWM,0);
      analogWrite(RPWM,Output);
  }
   */

   analogWrite(LPWM,255);
      analogWrite(RPWM,0);
    delay(2000);
    Serial.print("mundur");
  analogWrite(LPWM,0);
      analogWrite(RPWM,255);
  delay(2000);
  //analogWrite(LPWM,LPWM_value);
  //analogWrite(RPWM,RPWM_value);


  //DISPLAY SERIAL MONITOR

  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.print("\t Output = ");
  Serial.print(Output);
  Serial.print("\t Setpoint = ");
  Serial.println(Setpoint);
}

