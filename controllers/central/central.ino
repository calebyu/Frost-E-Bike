#define THERM1 A9
#define THERM2 A8
#define RAIL_SEN A1
#define BACKUP_RAIL_SEN A0
#define PWR_SHUNT_ON 8

#define VIS_ON 5
#define BRAKE_ON 6
#define BRAKE_L 9
#define HEAD_ON 10
#define SIG1 11
#define SIG2 12
#define LED 13

#include <FlexCAN.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <FrostEBike.h>


// Components
FlexCAN CANbus(500000);
CAN_message_t msg,rxmsg;

long previousTime = 0;
long interval = 10; //ms  

//Pin parameters


//State variables
int spdL = 0;
int spdR = 0;
int spdF = 0;
int spd = 0;
int bat = 0;
int bat_low = 0;
int pedal_in = 0;
uint8_t ABS_state = 0; //Front Right Left
uint8_t TRC_state = 0; //FRL
int throttle = 0;
int steering_angle = 0; // 0 is forward
int break_sig = 0; //0: off 1: on

//Control Settings
int ABS_on = 1;
int TRC_on = 1;
int pedal_wheel_ratio = 100; //FIND A GOOD DEFAULT FOR THIS SETTING
int front_light_on = 0;
int motor_drive_mode = 0; //0: Pedal Assist 1: Throttle
//Target torques
int torL = 0;
int torR = 0;
int torF = 0;

//for fun variables
int cnt = 0;
int spinFlag = 0;

void setup() {
  
  // initializes i/o stuff
  Serial.begin(9600);
  
  //Setup CANBus
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 0;
  mask.id = 0xFF;  
  CANbus.begin(mask);
  //Set filters - specs says to set all the filters 
  CAN_filter_t filter;
  filter.rtr = 0;
  filter.ext = 0;
  filter.id = CENTRAL_ID;
  for (int i = 0; i<8;i++)
    CANbus.setFilter(filter,i);
  
  //LED
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH); 
  
  bat = 100;
}

void loop() { unsigned long currentTime = millis();
  //wrap case
  if (currentTime < previousTime) previousTime = currentTime;
   
  //time limited loop
  if(currentTime - previousTime > interval) {
  
  digitalWrite(LED,HIGH);
  previousTime = currentTime;   
  
  //Read CANBus
  while (CANbus.read(rxmsg)){
    //Serial.print("MSG RCV: ");
    switch (rxmsg.buf[0]){
      case GENERIC:{
        Serial.println(String(rxmsg.buf[1]));
        break; 
      }
      case DASHBOARD_INPUT:{
        ABS_on = rxmsg.buf[1];
        TRC_on = rxmsg.buf[2];
        pedal_wheel_ratio = rxmsg.buf[3]; //FIND A GOOD DEFAULT FOR THIS SETTING
        front_light_on = rxmsg.buf[4];
        motor_drive_mode = rxmsg.buf[5]; //0: Pedal Assist 1: Throttle
        break;
      }
	  case DRIVER_CONTROL:{
	    throttle = rxmsg.buf[1];
        steering_angle = rxmsg.buf[2];
        break_sig = rxmsg.buf[3]; 
	    break;
	  }
      case REPORT_VELOCITY:{
        switch (rxmsg.buf[4]){
          case RIGHT_MOTOR_ID:
          {
            spdR = rxmsg.buf[1];
            ABS_state = (ABS_state & B0101) + rxmsg.buf[2] << 1;
            TRC_state = (TRC_state & B0101) + rxmsg.buf[2] << 1;
            break; 
          }
          case LEFT_MOTOR_ID:
          {
            spdL = rxmsg.buf[1];
            ABS_state = (ABS_state & B0110) + rxmsg.buf[2];
            TRC_state = (TRC_state & B0110) + rxmsg.buf[2];
            break;
          }
          case FRONT_MOTOR_ID:
          {
            spdF = rxmsg.buf[1];
            ABS_state = (ABS_state & B0011) + rxmsg.buf[2] << 2 ;
            TRC_state = (TRC_state & B0011) + rxmsg.buf[2] << 2;
            break;
          }
          break;
        } 

      }
	  case REPORT_PEDAL:{
		pedal_in = rxmsg.buf[1];
        break;
      }
	  case REPORT_BATTERY:{
		bat = rxmsg.buf[1];
		bat_low = rxmsg.buf[2];
        break;
      }
    } 
  }
  
  //Processing
  
  //{ Update Dashboard 
  msg.len = 8;
  msg.id = CONTROL_PANEL_ID;
  for( int idx=0; idx<8; ++idx ) {
      msg.buf[idx] = 0;
  }
  msg.buf[0] = DASHBOARD_OUTPUT;
  msg.buf[1] = (int)(spdF + spdR + spdL)/3;
  msg.buf[2] = bat;
  msg.buf[3] = bat_low;
  msg.buf[4] = (ABS_state)?1:0;
  msg.buf[5] = (TRC_state)?1:0;
  CANbus.write(msg);
  //}
  
  //Speed Processing
  // input throttle/pedal
  // steering angle compensation
  // use torF, torR, torL
  
  //{ Update Torque/Current
  //perhaps add checking if target tor has changed first before resending
  
  msg.id = FRONT_MOTOR_ID;
  for( int idx=0; idx<8; ++idx ) {
    msg.buf[idx] = 0;
  }
  msg.len = 4;
  msg.buf[0] = SET_TORQUE;
  msg.buf[1] = torF;
  msg.buf[2] = ABS_on;
  msg.buf[3] = TRC_on;
  CANbus.write(msg);
  
  msg.id = RIGHT_MOTOR_ID;
  msg.buf[1] = torR;
  CANbus.write(msg);
  
  msg.id = LEFT_MOTOR_ID;
  msg.buf[1] = torL;
  CANbus.write(msg);
  //}
 
  digitalWrite(LED,LOW);
  }
}

