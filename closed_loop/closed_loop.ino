// Adapted from http://blog.industrialshields.com/en/how-to-read-encoder-speed-through-an-interrupt-with-an-arduino-based-plcs/

#include <PID_v1.h>

//Set up Pins
#define phaseB 2 //Digital pin for encoder phase B
#define phaseA  3 //Digital pin for encoder phase A
#define enA 9 //Motor A enable pin 9
#define dirA1 10 //Motor direction 1 pin 4
#define dirA2 11 //Motor direction 2 pin 5
#define pot A0 //Potentiometer pin A0

//Initialize Variables
int valuePhaseB = 0; //initialize phase B
double potValue = 0;
volatile long pulses = 0; //initialize number of pulses detected
unsigned long timeO = 0; //initialize initial time
unsigned long timeF = 0; //initialize final time
double encoderSpeed = 0; // initialize encoder speed
double interval = 1000000; //choose sampling interval length for calculating and reporting rpm
double pwmOutput = 0;
double setpt = 0;

//Specify the links and initial tuning parameters
double Kp = 0.2, Ki=0.1, Kd = 0;
PID myPID(&encoderSpeed, &pwmOutput, &setpt, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600); 
  pinMode(phaseA, INPUT);   // configure the encorder pins as inputs
  pinMode(phaseB, INPUT);
  pinMode(enA, OUTPUT); //configure Motor pin as output
  pinMode(dirA1, OUTPUT); //configure Motor direction 1 pin as output
  pinMode(dirA2, OUTPUT); //configure Motor direction 2 pin as output
  attachInterrupt(digitalPinToInterrupt(phaseA), detectPulses, RISING);    // enable external interrupt with a rising edge of PhaseA
  timeO = micros();   // Initialize initial time
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
  
}

void loop() {
    //Set Motor speed according to potentiometer
    potValue = analogRead(pot); // Read potentiometer value
    //setpt = map(potValue, 320, 620, -500, 500);
    setpt = map(potValue, 400, 480, -500, 500);
    setpt = constrain(setpt,-500,500);


    //Measure RPM
    timeF = micros(); //Getting the final time  
    if ((float)(timeF-timeO) > interval) { //when final time exceeds interval, calculate rpm
    encoderSpeed = ((float)pulses/360.0) / ((float)(timeF - timeO) / (60000000.0)); 
    Serial.print("Speed: "); 
    Serial.print(encoderSpeed);     // Printing speed value
    Serial.println(" rev/min");
    Serial.print("Setpt: ");
    Serial.println(setpt);
    Serial.print("Outpt: ");
    Serial.println(pwmOutput);
    pulses = 0;    // Reset pulse count
    timeO = micros();    // Reset initial time again
     }
    
    myPID.Compute();
    if (pwmOutput <= 0) { //if handle is below center, go in reverse
    digitalWrite(dirA1, LOW); //Set motor direction to revese
    digitalWrite(dirA2, HIGH);
    analogWrite(enA, -pwmOutput); // Send PWM signal to L298N Enable pin
    //Serial.println(pwmOutput);
    }
    else { //if handle is above center
    digitalWrite(dirA1, HIGH); //Set motor direction forward
    digitalWrite(dirA2, LOW);
    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
    //Serial.println(pwmOutput);
    }
     
    delay(20);
}

// Interrupt function for detecting and counting pulses
void detectPulses() {
  if (digitalRead(phaseB) == LOW) { //When phase A is high (from interrupt) and phase B is low, motor is turning CW direction
    pulses++;    // Adding pulses
  }
  else { //Motor turning on CCW direction.
    pulses--;    // Substraing pulses
  }
}
