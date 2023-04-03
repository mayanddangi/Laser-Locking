/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : laserlocking.ino
  * @brief          : Code for generating rampMain program body
  * @author         : Deepshikha, Mayand
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Queue.h"
//#include <SPI.h>
#include "analogShield.h"

typedef struct {
  String param_name;
  float param_value;
} Parameter ;

/* Measured quantities */
Parameter measured_sig_amp = {"Desired lock-point amplitude [mV]",500};	//the amplitude of the discriminator slope
Parameter loop_speed = {"Running loop speed (locking mode) [kHz]",12.8};

/* Input Initial PID Control ParameteParameterrs */
Parameter pterm_piezo = {"P (piezo)",0.01};								// proportional gain term on piezo
// Parameter pterm_current = {"P (current)",2};							// proportional gain term on current
Parameter itime = {"Integration time constant [us]",500};				// integration time constant in microseconds
Parameter stime = {"Second integration time constant [us]",1000};		// second integration time (i squared) in microseconds
Parameter dtime = {"Derivative time constant [us]",2};					// derivative time constant in microseconds
// Parameter fterm = {"Feedforward to current",0.1};						//feedforward scaling term
Parameter alpha = {"Low-pass filter constant alpha",0.9};				//proportional gain low-pass filter constant

/* Adjust Initial Ramp Parameters */
Parameter freq = {"Scan frequency [Hz]",10};							//in Hz
Parameter ramp_amp = {"Scan amplitude [V]", 3};							// in V
/* Private variables ---------------------------------------------------------*/
byte byte_read;
bool lock_state = 0;  													// 0 is scanning, 1 is locked
int trigger_auto_relock = 0;

// unsigned long loop_counter = 0;

// float zerov = 32768.0; //Zero volts
float threshold = 0.01;

//set point param
float set_point;
float set_point_time;
float set_point_offset;

float lock_point;
float lock_point_time;
float lock_point_offset;
// PID params
// float PI_out = 0;
// float PIs_out = 0;
float PIID_out = 0;
float P_out = 0;
float error;
float error_previous = 0;
float d_error;
float d_error_previous = 0;
float accumulator = 0;
float accumulator_squared;

// ramp
double period = 1000000/freq.param_value;
long ramp_reset_time = 0;
long current_time = 0;
long ramp_time = 0;

// signal filter
int sf_size = 10;
// Queue signal_filter(sf_size);
  
// derivative_filter
int df_size = 10;
// Queue deri_filter(df_size);

// test queue
// int test_size = 10;
// Queue d(test_size);

/* Private function prototypes -----------------------------------------------*/
float ToBits(float voltage);
float ToVoltage(float bits);
float RampOut(long ramp_time);
void print_queue(Queue q);
float getSetPoint();
void startPID();
float floatIn();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Set line-ending to 'no line-ending'");
  Serial.println("");
  Serial.println("To toggle between scan/lock, type [y]."); 
  Serial.println("");
  Serial.println("Scan amplitude: " + String(ramp_amp.param_value,3) + " - to change, type [a]."); 
  Serial.println("Proportional gain on piezo channel: " + String(pterm_piezo.param_value, 4) + " - to change, type [p].");
  // Serial.println("Proportional gain on current channel: " + String(pterm_current.param_value, 4) + " - to change, type [c].");
  Serial.println("Integral gain time: " + String(itime.param_value, 2) + " us - to change, type [i].");
  Serial.println("Integral-squared gain time: " + String(stime.param_value, 2) + " us - to change, type [s].");
  // Serial.println("Feedforward scaling: " + String(fterm.param_value, 6) + " - to change, type [f].");
  Serial.println("Proportional gain low-pass filter constant: " + String(alpha.param_value, 6) + " - to change, type [l].");
  Serial.println("**Desired signal amplitude (approx) set to: " + String(measured_sig_amp.param_value,3) + " mV - to change, type [m].**");
  Serial.println("");
  Serial.println("scanning mode");
  Serial.println("");
  // queue initialization
  // for(int i=0; i<sf_size; i++)  signal_filter.enQueue(0);
  // for(int i=0; i<df_size; i++)  deri_filter.enQueue(0);
  // for(int i=0; i<test_size; i++)  d.enQueue(0);
}

void loop () { 
  display_freeram();
  byte_read = Serial.read();
  
  /* Listen for parameter adjust commands */
  if(byte_read == 'a') {ramp_amp = UpdateParameter(ramp_amp); }
  if(byte_read == 'r') {freq = UpdateParameter(freq); }
  // if(byte_read == 'f') {fterm = UpdateParameter(fterm); }
  if(byte_read == 'l') {alpha = UpdateParameter(alpha); }
  if(byte_read == 'p') {pterm_piezo = UpdateParameter(pterm_piezo); }
  // if(byte_read == 'c') {pterm_current = UpdateParameter(pterm_current); }
  if(byte_read == 'i') {itime = UpdateParameter(itime); }
  if(byte_read == 's') {stime = UpdateParameter(stime); }
  if(byte_read == 'd') {dtime = UpdateParameter(dtime); }
  if(byte_read == 'm') {measured_sig_amp = UpdateParameter(measured_sig_amp); }
  if(byte_read == 'o') {loop_speed = UpdateParameter(loop_speed); }
  
  period = 1000000/freq.param_value; //in micros
  
  if(byte_read == 'y') {
    lock_state = !lock_state; //toggle the lock state
    if(lock_state == 0) {
      Serial.println("Scanning mode");
    }
    if(lock_state == 1) {
      Serial.println("Locking mode");
	  // reset params
      // loop_counter = 0; //reset a bunch of parameters
	  
	  // PID params
      accumulator = 0;
      accumulator_squared = 0;
      error_previous = 0;
      d_error_previous = 0;
	  
	  // obtain set point
      set_point_time = getSetPoint();

      // Serial.println("hello");
      ramp_reset_time = micros();
      do{
        current_time = micros();
        ramp_time = current_time-ramp_reset_time;
		// analogWrite(3, RampOut(ramp_time));
        analog.write(0, ToBits(RampOut(ramp_time)));
      }while(ramp_time<set_point_time);
    }
    
  }
 
  /* -------------- Scanning Mode ----------------------
   * Generate Ramp
  */
  // Serial.println(lock_state);
  if(lock_state == 0) {
    // Serial.println("Scanning Start");
    ramp_reset_time = micros(); 
    do {
      current_time = micros(); //time in mircoseconds
      ramp_time = current_time-ramp_reset_time;
	  // analogWrite(3, RampOut(ramp_time));
      analog.write(0, ToBits(RampOut(ramp_time)));
    } while(ramp_time<period);  
  }
  /* -------------- Lock Mode ----------------------
   * obtain setpoint
   * start PID
  */
  if(lock_state == 1) {
    // loop_counter ++;
    startPID();
	set_point_offset = RampOut(set_point_time);
	  // analogWrite(3, PIID_out+set_point_offset);
    analog.write(0, ToBits(PIID_out+set_point_offset));  
    Serial.println("inside lock point");
  // AUTO-RELOCK
    if(abs(accumulator)>50) {
      Serial.println("Out of lock!");
      // loop_counter = 0;
      accumulator = 0;
      accumulator_squared = 0;
      set_point_time = getSetPoint();
      // Scan once more until lock_point_time so system is in the capture range:
      ramp_reset_time = micros();
      do {
        current_time = micros(); //time in mircoseconds
        ramp_time = current_time-ramp_reset_time;
		// analogWrite(3, RampOut(ramp_time));
        analog.write(0, ToBits(RampOut(ramp_time)));
      } while(ramp_time<set_point_time);
    }

  }
}

float ToVoltage(float bits) {
  return (bits-32768)/6553.6;
}

float ToBits(float voltage) {
  return voltage*6553.6+32768;
}

float RampOut(long ramp_time) {
  float ramp;
  float amp = ramp_amp.param_value;
  float offset = 2.5-amp/2;
  if(ramp_time<=(period/2))
      ramp = (amp/(period/2))*ramp_time + offset;
    else 
      ramp = -(amp/(period/2))*ramp_time + 2*amp + offset;
    return ramp;
}

void print_queue(Queue q){
  String q_arr = "[";
  int len = q.get_length();
  float *ptr;
  int front = q.get_first_index();
  ptr = q.getQueue();
  int i =0;
  for (; i<len-1; i++){
    if(i==q.get_size()-1) break;
    q_arr+=String(*(ptr+(i+front)%len));
    q_arr+=",";
  }
  q_arr+=String(*(ptr+(i+front)%len));
  q_arr+= "]";

  Serial.println(q_arr);
}

// float getSetPoint(void) {
//   float max_val = -5;
//   float min_val = 5;
//   float max_val_time;
//   float min_val_time;
//   float slope;
//   float sig_amp;
//   int lock_point_found = 0;
//   float lock_point_slope;

//   sig_amp = measured_sig_amp.param_value/1000; // in V
//   min_val = 5;
//   max_val = -5;

//   /* Start scan */
//   ramp_reset_time = micros();
//   do {
//     current_time = micros(); //time in mircoseconds
//     ramp_time = current_time-ramp_reset_time;
//     analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
//     float sig_read = (ToVoltage(analog.read(0)-analog.read(1)));
//     // float sig_read = ToVoltage(analog.read(0));
//     // float sig_read = (ToVoltage(analog.read(0)));
//     // Serial.println(sig_read);
//     /* Find the minimum */
//     if(sig_read<min_val && ramp_time<period/2) {
//       min_val = sig_read;
//       min_val_time = ramp_time; 
//       max_val = -5;
//     }
//     /* Find the maximum that follows the minimum - if the difference between the two is close to the 
//        signal amplitude, the correct peak has been found */
//     if(ramp_time>min_val_time && sig_read>max_val && ramp_time<period/2) {
//       max_val = sig_read;
//       max_val_time = ramp_time;
//       slope = max_val - min_val;
//       if(abs(slope-sig_amp)<0.25*sig_amp)
//        {
//         lock_point = (max_val+min_val)/2;
//         //  lock_point =0;
//         lock_point_time = (((max_val_time + min_val_time))/2.0);
//        lock_point_offset = RampOut(lock_point_time);
//               // lock_point_offset = RampOut(lock_point_time-lock_point_time*1000);

//         // lock_point_offset = 0;
//         lock_point_found = 1;
//         lock_point_slope = slope;
//       }
//     }      
  
//   } while(ramp_time<period);
    
//   Serial.print("Lock-point signal amplitude: ");
//   Serial.print(lock_point_slope*1000,2);
//   Serial.print(" mV; Lock-point time (from start of scan): ");
//   Serial.print(lock_point_time/1000.);
//   Serial.println(" ms");
  
//   if(lock_point_found==0) {
//     accumulator = 100; //triggers relocking
//     Serial.println("Lock-point not found");
//   }
//   if(lock_point_found==1);
//     Serial.println("Relocked");
//   return lock_point_time;
// }

float getSetPoint(){
  // signal filter
  int sf_size = 10;
  Queue signal_filter(sf_size);
  for(int i=0; i<sf_size; i++)  signal_filter.enQueue(0);
  
  // derivative_filter
  int df_size = 10;
  Queue deri_filter(df_size);  
  for(int i=0; i<df_size; i++)  deri_filter.enQueue(0); 

  float max_val = -5;
  float min_val = 5;
  float max_val_time = 0;
  float min_val_time = 0;

  Queue minima_time(10);
  Queue maxima_time(10);

  float err_sig = 0;
  float err_sig_prev = 0;
  float err_sig_avg = 0;
  float err_sig_avg_prev = 0;

  float deri_avg = 0;
  float deri_avg_prev = 0;
  
  bool find_max = false;
  bool find_min = true;
  bool lockpoint_found = false;
  
  float ramp_start_time = micros();
  
  while(ramp_time <= period){
    current_time = micros();
    err_sig_prev = err_sig;
    //----------------Generate Ramp----------------
    ramp_time = current_time - ramp_start_time;
	// analogWrite(3, RampOut(ramp_time));
    analog.write(0, ToBits(RampOut(ramp_time)));


    
    //-----------------Read Signal-----------------
   err_sig = ToVoltage(analog.read(0) - analog.read(1));
    // err_sig = 0;
	// err_sig = (analogRead(0) - analogRead(1))/1024*5;

    signal_filter.deQueue();
    signal_filter.enQueue(err_sig);

    err_sig_avg_prev = err_sig_avg;
    err_sig_avg = signal_filter.avg_arr();

    //---------------Update Signal-----------------
    deri_filter.deQueue();
    deri_filter.enQueue(err_sig - err_sig_prev);

    deri_avg_prev = deri_avg;
    deri_avg = deri_filter.avg_arr();

    if(ramp_time > period/8 && ramp_time < period/3){
      //find maximum if find_max = true
      if(err_sig_avg > max_val && find_max){
        max_val = err_sig_avg;
        max_val_time = ramp_time;
      }
      //find minimum if find_min = true
      if(err_sig_avg < min_val && find_min){
        min_val = err_sig_avg;
        min_val_time = ramp_time;
      }
  
      if(deri_avg_prev * deri_avg < 0){
        // find local minima
        if(min_val < err_sig_avg){
          minima_time.enQueue(min_val_time);
          min_val = 5;
          find_min = false;
          find_max = true;
        }
        // find local maxima
        if(max_val >err_sig_avg){
          maxima_time.enQueue(max_val_time);
          max_val = -5;
          find_max = false;
          find_min = true;
        }
      }
    }
    
  }
  // finding lockpoint
  int len = maxima_time.get_length();
  float sig_amp = 0;
  
  float *max_ptr;
  max_ptr = maxima_time.getQueue();
  int max_front = maxima_time.get_first_index();
  
  float *min_ptr;  
  int min_front = minima_time.get_first_index();
  min_ptr = minima_time.getQueue();
  
  threshold = measured_sig_amp.param_value;
  for (int i=0; i<len; i++){
    if(i == maxima_time.get_size()-1) break;
    sig_amp = *(max_ptr+(i+max_front)%len) - *(min_ptr+(i+min_front)%len);
    if(abs(threshold-sig_amp)<0.25*sig_amp){
      lockpoint_found = true;
      set_point_time = (max_front + min_front)/2;
    }
  }



  if(lockpoint_found){
    set_point = sig_amp;
    set_point_time = (max_front + min_front)/2;
    Serial.print("Lock-point signal amplitude: ");
    Serial.print(sig_amp);
    Serial.print(" mV; Lock-point time (from start of scan): ");
    Serial.print((max_front+min_front)/2/1000.);
    Serial.println(" ms");
  }

  if(lockpoint_found==false) {
      accumulator = 100; //triggers relocking
      Serial.println("Lock-point not found");
    }

  return set_point_time;

}

////////////////////////////PID///////////////////////////////////////////////////
void startPID(){
  Serial.println("In PID loop");
  // float err_sig = (analogRead(0) - analogRead(1))/1024*5; 
 float error_sig = ToVoltage(analog.read(0) - analog.read(1));
   // float error_sig = 0;
  Serial.println("Hello");
  float dt = 1000/loop_speed.param_value;
  error = set_point - error_sig; 
  error = (alpha.param_value*error_previous) + (1-alpha.param_value)*error;
  P_out = pterm_piezo.param_value*(error);

  if(P_out>=4)
    P_out = 4;
  if(P_out<=-4)
    P_out = -4;
    
  accumulator += error;  // accumulator is sum of errors (for integral gain term)
  accumulator_squared += error + (1/stime.param_value)*accumulator*dt;

  d_error = error-error_previous;
  d_error = 0.9*d_error_previous + 0.1*d_error;
  
  PIID_out = P_out+((pterm_piezo.param_value*(1/itime.param_value)*accumulator_squared*dt)+(dtime.param_value/dt)*(d_error));

  error_previous = error;
  d_error_previous = d_error;
}

/* Accept a serial input float */
float floatIn() {
  while(!Serial.available()){} //Wait for serial input
  return Serial.parseFloat(); //parse the next float
}

Parameter UpdateParameter(Parameter param) {
  Serial.print(param.param_name + " = ");
  Serial.print(param.param_value,6);
  Serial.println(" - enter a new value for " + param.param_name + ":");
  float new_value = floatIn();
  Serial.print(param.param_name + " = ");
  Serial.println(new_value,6);
  Parameter new_param = {param.param_name,new_value};
  return new_param;
}

void display_freeram() {
  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}
