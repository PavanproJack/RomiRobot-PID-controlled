 
//#include "encoders.h"
#include "lineSensors.h"

#include <Romi32U4.h>

Romi32U4Encoders encoders;

float angleDeg, angleRad;
float xProj, yProj;
float theta = 0.0;
float distancePerCount = 0.015;
int wheelDistance = 14;
float meanDistance;
float finalX = 0.0;
float finalY = 0.0;
int turn = 0;
int16_t count_left_e;
int16_t count_right_e;


#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define BUZZER_PIN  6
#define LINE_THRE 300
#define POWER_MAX 30
#define LINE_STR 5


#define LINE_LEFT_PIN A4 //LEFT PIN
#define LINE_CENTRE_PIN A3 //CENTER PIN
#define LINE_RIGHT_PIN A2 //RIGHT PIN

LineSensor line_left(LINE_LEFT_PIN); //Line sensor object for the left sensor
LineSensor line_centre(LINE_CENTRE_PIN); //Line sensor object for the centre sensor
LineSensor line_right(LINE_RIGHT_PIN); //Line sensor object for the right sensor

unsigned long last_timestamp;
unsigned long time_now;
unsigned long elapsed_time;
int l_value; // left sensor
int c_value; // centre sensor
int r_value; // right sensor
volatile int left_speed;
volatile int right_speed;
int end_of_line_cnt=0;
float Itot;
float Pl;
float Pr;
float M;

int flag=0;


void play_tone()
{
for (int i=0; i<=100; i++)
{
analogWrite(BUZZER_PIN, 50);
delay(5);
analogWrite(BUZZER_PIN, 0);
}
delay(100);
for (int i=0; i<=100; i++)
{
analogWrite(BUZZER_PIN, 50);
delay(5);
analogWrite(BUZZER_PIN, 0);
}
}

void motor_left(signed power){
if (abs(power)>255){
Serial.println("ERROR LEFT MOTOR");   
}
else if (power>0){
digitalWrite( L_DIR_PIN, LOW);
analogWrite( L_PWM_PIN,power);
}
else
{
digitalWrite( L_DIR_PIN, HIGH);
analogWrite( L_PWM_PIN,-power);
}
}

void motor_right(signed power)
{
if (abs(power)>255)
{
Serial.println("ERROR RIGHT MOTOR");  
}
else if (power>0)
{
 digitalWrite( R_DIR_PIN, LOW);
 analogWrite( R_PWM_PIN,power);
}
else
{
  digitalWrite( R_DIR_PIN, HIGH);
  analogWrite( R_PWM_PIN,-power);
  }
}

void setupMotorPins() {
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );
  pinMode( LINE_LEFT_PIN, INPUT );
  pinMode( LINE_CENTRE_PIN, INPUT );
  pinMode( LINE_RIGHT_PIN, INPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?

}

void all_stop()
{
  motor_right(0);
  motor_left(0);
  delay(200); 
}

void move_forward(int power)
{
  motor_right(power);
  motor_left(power);
}

void turn_left(int power_turn_left)
{
  motor_right(power_turn_left);
  motor_left(-power_turn_left);
}

void turn_right(int power_turn_right)
{
  motor_left(power_turn_right);
  motor_right(-power_turn_right); 
}

//this function run the 

void Drive_Romi(float Mag)
{
  if (Mag >= 0.3)
  {
    turn_left(40);
  }
  else if (Mag <= -0.3)
  {
    turn_right(40);
  }
  else
  {
    left_speed=(-POWER_MAX)*Mag+30;
    right_speed= POWER_MAX*Mag+30;
    motor_left(left_speed);
    motor_right(right_speed);
    
}
    
}

// Remember, setup only runs once.
void setup() {

  // change interrupts for the encoders.
  //setupLeftEncoder();
  //setupRightEncoder();

  // Initialise your other globals variables
  // and devices.

//t_timestamp = millis();

  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
  
  // Calibrate the three line sensors.
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();
   
  //move forward until meeting the line
  
  while(line_left.readCalibrated()<LINE_THRE and line_centre.readCalibrated()<LINE_THRE and line_right.readCalibrated()<LINE_THRE)
  {
    move_forward(30);
    calcOdometry(encoders.getCountsLeft(), encoders.getCountsRight(), 0);
    delay(600);
  }
  
  all_stop();
  delay(500);
}



void loop() {

//Serial.println("turn is : " + String(turn) );
//
// Read analog voltages
  l_value=line_left.readCalibrated();
  c_value=line_centre.readCalibrated();
  r_value=line_right.readCalibrated();

  count_left_e = encoders.getCountsLeft();
  count_right_e = encoders.getCountsRight();
  

  calcOdometry(count_left_e, count_right_e, turn);
  
  if(turn == 0){
      Itot= l_value+c_value+r_value;
      Pl=l_value/Itot;
      Pr=r_value/Itot;
      M=Pl-Pr;
      
      //Drive_Romi(M);
      delay(1);
      int LINE_THREb=50;
      if(l_value<LINE_THREb and c_value<LINE_THREb and r_value<LINE_THREb)
      {
         
        end_of_line_cnt++;
        move_forward(30);
        if (end_of_line_cnt++<=10 )
        {
          all_stop();
          Serial.println("Inside");
          if (flag==0){
            flag=1;
            play_tone();
            delay(700);
            turn = 1;
          }
        }
      }
      else
      {
        Drive_Romi(M);
      }
  }
  
}

float finalTheta = 0.0;

void calcOdometry(int16_t encoderLPos, int16_t encoderRPos, int turn){

  //Serial.println("Encoder values are" + String(encoderLPos) + ",,,," + String(encoderRPos));

    static int encoderRPosPrev = 0;
    static int encoderLPosPrev = 0;

    float SR = distancePerCount * (encoderRPos - encoderRPosPrev);
    float SL = distancePerCount * (encoderLPos - encoderLPosPrev);

    encoderRPosPrev = encoderRPos;
    encoderLPosPrev = encoderLPos;
    

    theta += (SR - SL) / wheelDistance;

    if(theta > 6.28)
      theta -= 6.28;
    else if(theta < -6.28)
      theta += 6.28;
   
     // Serial.println("Deg : " + String(theta * 57.2958) );
        meanDistance = (SL + SR)/2;
        xProj = xProj + meanDistance*cos(theta);
        yProj = yProj + meanDistance*sin(theta);

      if(turn == 0){
           finalX = xProj;
           finalY = yProj;
           finalTheta = theta;
           Serial.println(
                      "Final Theta : " + String(finalTheta * 57.2958) + "\t" + 
                      "final X : " + String(xProj) + "\t" + 
                      "final Y : " + String(yProj)
                      );
        }
      else if(turn == 1){   // Turn 180 degrees opposite to start location
        
           float t = theta * 57.2958;
           int difference = -180 - t;
           Serial.println("Difference Angle is : " + String(difference));
           
              if(difference > 0){
                      motor_drive(HIGH, LOW);
                      Serial.println("Moving Left");
                  }else if(difference < 0){
                      motor_drive(LOW, HIGH);
                      Serial.println("Moving Right");
                    }else{
                      Serial.println("Stopping");
                       //all_stop();
                       resetEncoders();
                           encoderRPosPrev = 0;
                           encoderLPosPrev = 0;
                           xProj = 0;
                           yProj = 0;
                           theta = 0;
                         //count_left_e = encoders.getCountsLeft();
                         //count_right_e = encoders.getCountsRight();
                       
                       stopAll();
                       delay(2000);
                       
                    }
                     
        }else if(turn == 2){   // Now move forward towards Start point.
            
          Serial.println("Difference X is : " + String(float(finalX - xProj)));
          Serial.println(
                      "final X : " + String(xProj) + "\t" + 
                      "final Y : " + String(yProj)
                      );
          
          if(int(finalX - xProj) == 0){
              all_stop();
            }else{   
              Serial.println("Moving Forward");
                move_forward(30);
                Serial.println(
                      "New X : " + String(xProj) + "\t" + 
                      "NEW Y : " + String(yProj)
                      );
                      delay(1700);
              }
          }
  }

  void stopAll(){
      motor_right(0);
      motor_left(0);
      //delay(200); 
      turn = 2;
    }

 

void resetEncoders(){
    Serial.println("Resetting Encoders");
    int RC = encoders.getCountsAndResetRight();
    int LC = encoders.getCountsAndResetLeft();
    Serial.println(LC);Serial.println(RC);
  }

void motor_drive(boolean ls, boolean rs){ // Drive motors according to the calculated values for a turn
  digitalWrite( L_DIR_PIN, ls );
  digitalWrite( R_DIR_PIN, rs );
  analogWrite(L_PWM_PIN, 20);
  analogWrite(R_PWM_PIN, 20);
  //delay(2000);
}