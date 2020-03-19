#include <Romi32U4.h>

Romi32U4Encoders encoders;

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15 

int weightsArray[] = {1000, 2000, 3000};
float setPoint = 1996;
int maxSpeed = 30;
int right_speed = 0;
int left_speed = 0;

int Kp = 1.0;
int Ki = 0.0002;
int Kd = 1.0;
float proportional = 0.0;
float errorIntegral = 0.0;
float errorDerivative = 0.0;
float lastError = 0.0;


float angleDeg, angleRad;
float xProj, yProj;
float theta = 0.0;
float distancePerCount = 0.015;
int wheelDistance = 14;
float meanDistance;

float d = 0.0;
float finalRightMotor = 0.0;

void setup() {

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  Serial.begin(9600);
  delay(1500);
  Serial.println("Calibrating the set point ....");


 /* while(1){
      digitalWrite( L_DIR_PIN, HIGH );
      digitalWrite( R_DIR_PIN, HIGH );
      analogWrite(L_PWM_PIN, 20);
      analogWrite(R_PWM_PIN, 18);
      delay(1500);
  }*/
}
 
 
void loop() {
  

      
      //float error = calcWeightedMean() - setPoint;
      //Serial.println("Error is : " + String(error)); 
      //int pidErr = calcPidGain(error);
      //calc_turn(pidErr);
     // motor_drive(20,20);
  
      int th = calcOdometry();
           //moveforward(th);
      
      delay(700);
      //Serial.println("Position is : " + String(calcWeightedMean()));
      //Serial.println("Error Val is : " + String(pidErr));
      //Serial.println("PID err is : " + String(pidErr));
}


  void moveforward(int t){
      int difference = 0 - t;
      if(difference > 0){
                      motor_drive(HIGH, LOW);
                      Serial.println("Moving Left");
                  }else if(difference < 0){
                      motor_drive(LOW, HIGH);
                      Serial.println("Moving Right");
                  }else{
                      motor_drive(HIGH, HIGH);
                      delay(700);
                      }
      
    }

void motor_drive(boolean ls, boolean rs){ // Drive motors according to the calculated values for a turn
  digitalWrite( L_DIR_PIN, ls );
  digitalWrite( R_DIR_PIN, rs );
  analogWrite(L_PWM_PIN, 20);
  analogWrite(R_PWM_PIN, 20);
  //delay(2000);
}


int calcOdometry(){
  
    int16_t encoderLPos = encoders.getCountsLeft();
    int16_t encoderRPos = encoders.getCountsRight();

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

    meanDistance = (SL + SR)/2;
    xProj = xProj + meanDistance*cos(theta);
    yProj = yProj + meanDistance*sin(theta);

    Serial.println(
                  "DEg : " + String(theta * 57.2958) + "\t"  
                 
                  );
    return int(theta);
  }

 int calcPidGain(float currentErr){
   
    //Serial.println("Current Error is : " + String(currentErr));
    //Serial.println("Error integral is : " + String(errorIntegral));
    //Serial.println("Error derivative is : " + String(errorDerivative));
    proportional = currentErr;
    errorIntegral =  currentErr + errorIntegral;
    errorDerivative =  currentErr - lastError;
    lastError = currentErr;
           // delay(1500);
  return int(proportional * Kp + errorIntegral * Ki + errorDerivative * Kd);
 }


float calcWeightedMean(){
   float lS =  analogRead(A4);  // left sensor towards pin 5
   float mS =  analogRead(A3); // middle sensor
   float rS =  analogRead(A2); // right sensor towards ground 

   float pos = ( weightsArray[0] * lS + weightsArray[1] * mS + weightsArray[2] * rS ) / (lS + mS + rS);
   return pos;
  }


void calc_turn(int error_value){ 
  if (error_value< (-1 * maxSpeed) ){
      error_value = -1 * maxSpeed;
   }
  if (error_value>= maxSpeed){
      error_value = maxSpeed;
   }
   // If error_value is less than zero calculate right turn speed values
  if (error_value< 0){
      left_speed = maxSpeed + error_value;
      right_speed = maxSpeed;
   }
   // Iferror_value is greater than zero calculate left turn values
  else{
      left_speed = maxSpeed; 
      right_speed = maxSpeed - error_value;
   }

   motor_drive(right_speed, left_speed);
}

void motor_drive(int rs, int ls){ // Drive motors according to the calculated values for a turn
  analogWrite(L_PWM_PIN, ls);
  analogWrite(R_PWM_PIN, rs);
  //delay(50); // Optional
}

void turnAngle(float t){

  Serial.println(t);

  int difference = -180 - t;

 
  Serial.println(difference);
  
  if(difference > 0){
     Serial.println("LEFT");
    }else if(difference < 0){
 
      while(1){
          motor_drive(LOW, HIGH);
          int val = ( encoders.getCountsRight() - encoders.getCountsLeft() ) / 0.063;
          Serial.println(String(val));
          delay(500);
        }
      //Serial.println("RIGHT");
      }else{
        Serial.println("0 angle");
        }
}