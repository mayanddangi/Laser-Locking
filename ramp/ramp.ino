#include "analogShield.h"

double period = 1000000/1;
long ramp_reset_time = 0;
long current_time = 0;
long ramp_time = 0;
int ramp_amp = 3.0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}


float ToBits(float voltage) {
  return voltage*6553.6+32768;
}

float RampOut(long ramp_time) {
  float ramp;
  float amp = ramp_amp;
  float offset = 2.5-amp/2;
  offset = 0;
  if(ramp_time<=(period/2))
      ramp = (amp/(period/2))*ramp_time + offset;
    else 
      ramp = -(amp/(period/2))*ramp_time + 2*amp + offset;
    return ramp;
}

void loop() {
  float ramp_start_time = micros();
  ramp_time = 0;
  while(ramp_time <= period){
    current_time = micros();
    //----------------Generate Ramp----------------
    ramp_time = current_time - ramp_start_time;
    analog.write(0, ToBits(RampOut(ramp_time)));
    Serial.println(RampOut(ramp_time));
  }
}
