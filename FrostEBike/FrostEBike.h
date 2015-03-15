// Control Unit IDs
enum CONTROLLER_ID { CENTRAL_ID, CONTROL_PANEL_ID, GENERATOR_ID, RIGHT_MOTOR_ID, LEFT_MOTOR_ID, FRONT_MOTOR_ID, BATTERY_ID};
// Message IDs
enum MSG_ID {GENERIC,  DASHBOARD_OUTPUT, DASHBOARD_INPUT, SET_TORQUE, REPORT_VELOCITY, REPORT_PEDAL, REPORT_BATTERY, DRIVER_CONTROL, REAR_SYNCH};

//Low pass filter
class LPF {

private:

float array[10];
int curr;
int size; 
float output = 0;

public:

LPF(){
  for (int i = 0; i < 10; i++) array[i]= 0;
  curr = 0;
  size = 0;
};

float addPoint (float value){
  array[curr] = value;
  size++;
  curr++;
  curr %= 10;
  if (size > 10) size = 10;
  
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
