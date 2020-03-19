#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
class LineSensor {
  public:

    // Required function.
    LineSensor(int pin);   //Constructor

    // Suggested functions.
    void calibrate();         //Calibrate
    int readRaw();            //Return the uncalibrated value from the sensor
    float readCalibrated();   //Return the calibrated value from the sensor
    bool isOverLine();

  private:
  
    int pin;
    int calibrate_count;
    int threshold;
    float bias_compensation;
    
};


// Class Constructor: 
// Sets pin passed in as argument to input
LineSensor::LineSensor(int Line_pin) {
  calibrate_count = 50;
  bias_compensation = 0.0;
  threshold = 100;

  pin = Line_pin;
  pinMode(pin, INPUT);
}

// Returns unmodified reading.
int LineSensor::readRaw() {
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void LineSensor::calibrate() {
  int running_total = 0;

  for (int i = 0; i < calibrate_count; i++)
  {
    int value = readRaw();
    running_total += value;
  }

  bias_compensation = (float)running_total/(float)calibrate_count;
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
float LineSensor::readCalibrated() {
  float calibrated_reading = (float)readRaw() - bias_compensation;
  return calibrated_reading;
}

// Check if sensor is over a line
bool LineSensor::isOverLine() {
  if (readCalibrated() > threshold) {
    return true;
  }
  return false;
}



#endif