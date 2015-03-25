#define CAN_BAUD 100000

// Control Unit IDs
enum CONTROLLER_ID { PLACEHOLDER, CENTRAL_ID, CONTROL_PANEL_ID, GENERATOR_ID, RIGHT_MOTOR_ID, LEFT_MOTOR_ID, FRONT_MOTOR_ID, BATTERY_ID};
// Message IDs
enum MSG_ID {GENERIC,  DASHBOARD_OUTPUT, DASHBOARD_INPUT, SET_TORQUE, REPORT_VELOCITY, REPORT_PEDAL, REPORT_BATTERY, DRIVER_CONTROL, REAR_SYNCH};


//Low pass filter
class LPF {

private:

float * array;
int curr;
int size; 
int max_size;
float output = 0;

public:

LPF(int max){
  array = NULL;
  array = new float[max];
  for (int i = 0; i < max; i++) array[i]= 0;
  curr = 0;
  size = 0;
  max_size = max;
};

float addPoint (float value){
  array[curr] = value;
  size++;
  curr++;
  curr %= max_size;
  if (size > max_size) size = max_size;
  
  float sum = 0;
  if (size != 0){
    for (int i = 0; i < size; i++) sum += array[i];
    output = sum/size;
  }
  return output;
}

float getValue (){
  if (size == 0) return 0;
  return output;
}

};  
/*
class LE {
//input array [input output]
private:

  float * p;
  int size;

public:

  LE(float * point_array, int cnt){
    p = point_array
    size = cnt;
  };

  float lin_ext (float value){
    if (value < p[0][0]){
      return p[0][1] + (p[1][1] - p[0][1])(value - p[0][0])/(p[1][0] - p[1][0])
    }
    for (int i = 0; i < cnt; i++){
      
    }
  }


};  */


