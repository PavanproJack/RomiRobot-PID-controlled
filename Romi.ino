// Include header files.
// These must be in your sketch folder.
#include "encoders.h"
#include "pid.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

// Experiment with your gains slowly, one by one.
float Kp_left = 0.0; //Proportional gain for position controller
float Kd_left = 0.0; //Derivative gain for position controller
float Ki_left = 0.0; //Integral gain for position controller
PID left_PID(Kp_left, Ki_left, Kd_left); //Position controller for left wheel position


float demand = 5; //Target speed


void setupMotorPins() {
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );
}

// put your setup code here, to run once:
void setup() {

  //Assign motor pins and set direction
  // These two function set up the pin
  // change interrupts for the encoders.
  // If you want to know more, find them
  // at the end of this file.  
  setupEncoder0();
  setupEncoder1();


  // Initialise the Serial communication
  // so that we can inspect the values of
  // our encoder using the Monitor.
  // Print something so we can see if we reset
  // the Romi accidentally.
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");

}

void loop() {

  //    output_signal <----PID-- demand, measurement
  // Note, e1_speed assumes you have done the time&speed
  // labsheet. 
  float output = left_PID.update(demand, e1_speed);

  Serial.print("Left wheel output signal is: ");
  Serial.println(output);

  // Once you think your error signal is correct
  // And your PID response is correct
  // Send output to motor

  // Consider switching this delay for a millis()
  // task-schedule block.
  delay(50);
}
