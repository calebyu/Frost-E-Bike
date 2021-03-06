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

#define CANS 2
#define LED 13

#include <FlexCAN.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <FrostEBike.h>


// Components
FlexCAN CANbus(CAN_BAUD);
CAN_message_t msg,rxmsg;

long previousTime = 0;
long interval = 5; //ms

int signal_timer_cnt = 0;

//Pin parameters


//State variables
int spdL = 0;
int spdR = 0;
int spdF = 0;
int spd = 0;
int bat = 0;
int bat_low = 0;
int pedal_curr = 0;
uint8_t ABS_state = 0; //Front Right Left
uint8_t TRC_state = 0; //FRL
int throttle = 0;
int steering_angle = 0; // 0 is forward
int brake_sig = 0; //0: off 1: on
int turn_sig= 0; //0: off 1: right 2: left
LPF rail_read = NULL;

//Control Settings
int ABS_on = 1;
int TRC_on = 1;
int pedal_wheel_ratio = 100; //FIND A GOOD DEFAULT FOR THIS SETTING
int light_on = 0;
int motor_drive_mode = 0; //0: Pedal Assist 1: Throttle
int cadence_set = 20;
int torque_set = 0;

//Target torques
int torL = 0;
int torR = 0;
int torF = 0;

//for fun variables
int cnt = 0;
int spinFlag = 0;
int light_state = 0;

void setup() {
  pinMode(CANS, OUTPUT);
  pinMode(THERM1, INPUT);
  pinMode(THERM2, INPUT);
  pinMode(RAIL_SEN, INPUT);
  pinMode(BACKUP_RAIL_SEN, INPUT);
  pinMode(PWR_SHUNT_ON, INPUT);
  
  pinMode(VIS_ON, OUTPUT);
  pinMode(BRAKE_ON, OUTPUT);
  pinMode(BRAKE_L, OUTPUT);
  pinMode(HEAD_ON, OUTPUT);
  pinMode(SIG1, OUTPUT);
  pinMode(SIG2, OUTPUT);

  digitalWrite(VIS_ON, LOW);
  digitalWrite(BRAKE_ON, LOW);
  digitalWrite(BRAKE_L, LOW);
  digitalWrite(HEAD_ON, LOW);
  digitalWrite(SIG1, LOW);
  digitalWrite(SIG2, LOW);
  digitalWrite(CANS, LOW);
  
  // initializes i/o stuff
  Serial.begin(9600);
  
  //Setup CANBus
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 0;
  mask.id = 0x0F;  
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
  rail_read = LPF(20);
}

void loop() { unsigned long currentTime = millis();
  //wrap case
  if (currentTime < previousTime) previousTime = currentTime;
   
  //time limited loop
  if(currentTime - previousTime > interval) {
  Serial.println("I am central");
  digitalWrite(LED,HIGH);
  previousTime = currentTime;   
  
  //Read CANBus
  while (CANbus.read(rxmsg)){
    Serial.print("MSG RCV: ");
    Serial.println(String(rxmsg.buf[0]));
    switch (rxmsg.buf[0]){
      case GENERIC:{
        Serial.println(String(rxmsg.buf[1]));
        break; 
      }
      case DASHBOARD_INPUT:{
        ABS_on = rxmsg.buf[1];
        TRC_on = rxmsg.buf[2];
        pedal_wheel_ratio = rxmsg.buf[3]; //FIND A GOOD DEFAULT FOR THIS SETTING
        light_on = rxmsg.buf[4];
        motor_drive_mode = rxmsg.buf[5]; //0: Pedal Assist 1: Throttle
        cadence_set = rxmsg.buf[6];
        torque_set = rxmsg.buf[7];
        break;
      }
      case DRIVER_CONTROL:{
        Serial.println("DRIVE CONTROL MSG: ");
        throttle = rxmsg.buf[1];
        steering_angle = rxmsg.buf[2];
        brake_sig = rxmsg.buf[3]; 
        turn_sig = rxmsg.buf[4]; 
        break;
	  }
      case REPORT_VELOCITY:{
        Serial.println("REPORT_VELOCITY");
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
        break;
      }
      case REPORT_PEDAL:{
        Serial.print("PEDAL MSG: ");
        Serial.println(pedal_curr);
        if (rxmsg.buf[1] > 15)
          pedal_curr = rxmsg.buf[1];
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
  //err = CANbus.write(msg);
//}
  
  //{Speed Processing
  // input throttle/pedal
  // steering angle compensation
  // use torF, torR, torL
  int torq_vec = 0;
  if (steering_angle == 1) // left turn
    torq_vec = -5;
  else if (steering_angle == -1) // right turn
    torq_vec = 5;
  
  if (motor_drive_mode == CADENCE_MODE || motor_drive_mode == TORQUE_MODE){
    torF = pedal_curr/3;
    torR = torF - torq_vec;
    torL = torF + torq_vec;
  }
  else if (motor_drive_mode == THROTTLE_MODE)
  {
    torF = throttle;
    torR = torF - torq_vec;
    torL = torF + torq_vec;  
  }
    
  //}
  
  //{ Update Torque/Current
  //perhaps add checking if target tor has changed first before resending
  
  rail_read.addPoint(analogRead(RAIL_SEN));
  if (rail_read.getValue() < 270){// In no battery condition, wait until bus is 15V before applying motors
    torF = 0;
    torR = 0;
    torL = 0;
  }
  Serial.print("Brake Sig: ");
  Serial.println(brake_sig, DEC);
  Serial.print("Target Torque: ");
  Serial.println(torF, DEC);
  msg.id = FRONT_MOTOR_ID;
  for( int idx=0; idx<8; ++idx ) {
    msg.buf[idx] = 0;
  }
  msg.len = 5;
  msg.buf[0] = SET_TORQUE;
  msg.buf[1] = torF;
  msg.buf[2] = ABS_on;
  msg.buf[3] = TRC_on;
  msg.buf[4] = brake_sig;
  CANbus.write(msg);
  
  msg.id = RIGHT_MOTOR_ID;
  msg.buf[1] = torR;
  CANbus.write(msg);
  
  msg.id = LEFT_MOTOR_ID;
  msg.buf[1] = torL;
  CANbus.write(msg);
  
  // Drive mode msg to generator
  msg.id = GENERATOR_ID;
  for( int idx=0; idx<8; ++idx ) {
    msg.buf[idx] = 0;
  }
  msg.len = 4;
  msg.buf[0] = DRIVE_MODE;
  msg.buf[1] = motor_drive_mode;
  msg.buf[2] = cadence_set;
  msg.buf[3] = torque_set;
  CANbus.write(msg);
        
  //}
  
  //{ Signaling code 

  int sig_pin;
  if (turn_sig == 1) 
    sig_pin = SIG1;
  else if (turn_sig == 2) 
    sig_pin = SIG2;
  else {
    sig_pin = 0;
  }
  if (light_on){
    digitalWrite(SIG1, HIGH);
    digitalWrite(SIG2, HIGH);
  }
  else {
    digitalWrite(SIG1, LOW);
    digitalWrite(SIG2, LOW);
  }
  
  if (turn_sig){
    if ( signal_timer_cnt > 50 ){
      signal_timer_cnt = -50;
      digitalWrite(sig_pin, HIGH);
    }
    else if ( signal_timer_cnt > 0 ){
      signal_timer_cnt++;
      digitalWrite(sig_pin, LOW);
    }
    else {
      signal_timer_cnt ++;
    }
  }
  //}

  digitalWrite(LED,LOW);
  }
}

