#define BAUD_RATE 9600

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15


// the setup function runs once when you press reset or power the board
void setup()
{  

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );
  
  
  Serial.begin(BAUD_RATE);
  //delay(1000);
  Serial.println(" ***Reset*** ");

}



void loop()
{

  float lS =  analogRead(A4);  // left sensor towards pin 5
  float mS =  analogRead(A3); // middle sensor
  float rS =  analogRead(A2); // right sensor towards ground 
 

 /*
  int lS =  digitalRead(A4);  // left sensor towards pin 5
  int mS =  digitalRead(A3); // middle sensor
  int rS =  digitalRead(A2); // right sensor towards ground
  */
  
  Serial.println( "(Left : " + String(lS) + "  \n Midd : " + String(mS) + "  \n Rght : " + String(rS)  + ")" );
  //Serial.println("\n  \n" );
  int threshold = 600;
  if(mS > threshold){
     moveForward();
  }else if(rS > threshold){
    moveLeft();
    }else if(lS > threshold){
    moveRight();
    }else{
      Stop();
      }
    
  
  
  
  delay(50);

}

void mySpeed(){
     analogWrite(L_PWM_PIN, 20);
     analogWrite(R_PWM_PIN, 20);
}


void moveForward(){
  Serial.println("Forward");
     mySpeed();
     digitalWrite( L_DIR_PIN, LOW  );
     digitalWrite( R_DIR_PIN, LOW );
  }
  void moveLeft(){
     Serial.println("Moving Left");
     mySpeed();
     digitalWrite( L_DIR_PIN, HIGH  );
     digitalWrite( R_DIR_PIN, LOW );
  }
  void moveRight(){
    Serial.println("Moving Right");
     mySpeed();
     digitalWrite( L_DIR_PIN, LOW  );
     digitalWrite( R_DIR_PIN, HIGH );
  }
  void Stop(){
    Serial.println("Stopping");
     analogWrite(L_PWM_PIN, 0);
     analogWrite(R_PWM_PIN, 0);
     //digitalWrite( L_DIR_PIN, LOW  );
     //digitalWrite( R_DIR_PIN, LOW );
  }
