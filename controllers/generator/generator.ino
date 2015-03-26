#define THERM1 A1
#define THERM2 A0

#define RAIL_SEN A9
#define CUR_SEN A8

#define GATES 5
#define HALL1 10
#define HALL2 11
#define HALL3 12

#define CANS 2

#include <FlexCAN.h>
#include <FrostEBike.h>
#include <math.h>

//State
LPF spd = NULL;
LPF rail = NULL;
int curr_in = 0;
volatile int spd_cnt = 0;
int prevtime = 0;
int curr;
int mode;
float torque_set;
int cadence_set;
int drive_mode;
int cadence;
int cadence_thres;
int rail_pedal_max;
  
void isr_spdcnt(){
  cli();
  spd_cnt++;
  sei();
} 

//CANBus
FlexCAN CANbus(CAN_BAUD);
CAN_message_t msg,rxmsg;

void setup(){
  
  // Safety variables and system managment
  pinMode(THERM1, INPUT);
  pinMode(THERM2, INPUT);
  pinMode(RAIL_SEN, INPUT);
  pinMode(CUR_SEN, INPUT);
  
  
  // Initialize gates for boost converter
  pinMode(GATES, OUTPUT);
  analogWriteFrequency(GATES, 23437);
  digitalWrite(GATES, LOW);
  
  // LED for hardwared debug
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // Initialize Serial
  Serial.begin(9600);
  Serial.println("Hello World!");
  
  // Setup CAN
  pinMode(CANS, OUTPUT);
  digitalWrite(CANS, LOW); // Wake up
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 0;
  mask.id = 0xFF;  
  CANbus.begin(mask);
  //Set filters - specs says to set all the filters 
  CAN_filter_t filter;
  filter.rtr = 0;
  filter.ext = 0;
  filter.id = GENERATOR_ID;
  for (int i = 0; i<8;i++)
  CANbus.setFilter(filter,i);
  
  // Initialize for cadence calculations
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);
  attachInterrupt(HALL1, isr_spdcnt, RISING);
  attachInterrupt(HALL2, isr_spdcnt, RISING);
  attachInterrupt(HALL3, isr_spdcnt, RISING);
  prevtime = millis();
  spd = LPF(15);
  delay(100);
  
  // Control variables
  mode = 0;
  torque_set = 1;
  cadence_set = 20;
  cadence_thres = 3;
  rail_pedal_max = 725; //30: 500, 25: 420, 20: 350, 15: 270
  rail = LPF(5);
}

void loop(){
  // Identify self over serial for sanity check
  Serial.println("I am generator ");
    
  // CANBus Read
  if (CANbus.read(rxmsg))
  {
    Serial.print("MSG RCV: ");
    switch (rxmsg.buf[0])
    {
      case GENERIC:
      {
        Serial.println(String(rxmsg.buf[1]));
        break; 
      }
      case DRIVE_MODE:
      {
          drive_mode = rxmsg.buf[1] ;
          cadence_set = rxmsg.buf[2];
          torque_set = rxmsg.buf[3];
        break;  
      }
    } 
  }
  
  // Analog and digital values read
  rail.addPoint( analogRead(RAIL_SEN) );
  
  
  // Detect Speed
  cli();
    spd.addPoint( (float) spd_cnt/ (millis() - prevtime ));
    
      Serial.print("SPD_CNT: ");
  Serial.println((String)spd_cnt);
  
    spd_cnt = 0;
  sei();
  prevtime = millis();
  
  Serial.print("SPD: ");
  Serial.println((String)spd.getValue());
  
  // Process
  // Set torque based control mode
  int ecadence;
  if (drive_mode == CADENCE_MODE){
    cadence = spd.getValue()/(100+torque_set)*100;
    ecadence = cadence - cadence_set;
    float ki = 1;
    if ( mode == 0 ){ // Constant cadence control
      if ( abs(ecadence) > cadence_thres/2 ){
        torque_set += ecadence*ki;
      }
    }
    else {
      torque_set = 1;
    }
    if ( torque_set > 200 ) torque_set = 200;
    if ( torque_set < 1 ) torque_set = 1;
    if ( rail.getValue() >= rail_pedal_max ) torque_set = 0;
  }
  else if (drive_mode == THROTTLE_MODE) torque_set = 150;
  
  analogWrite(GATES, (int)torque_set);
  Serial.print("Torque: ");
  Serial.println(torque_set,DEC);
  Serial.print("Cadence: ");
  Serial.println(cadence,DEC);
  Serial.print("Err-Cadence: ");
  Serial.println(ecadence,DEC);
  
  // CANBus Write
  // current processing to amps
  curr = analogRead(CUR_SEN);
  
  msg.len = 8;
  msg.id = msg.id = GENERATOR_ID << 4 | CENTRAL_ID;
  for( int idx=0; idx< msg.len; ++idx ) {
      msg.buf[idx] = 0;
  }
  msg.buf[0] = REPORT_PEDAL;
  msg.buf[1] = curr - 16;
  int err = 0;
  err = CANbus.write(msg);
  Serial.print("ERR: ");
  Serial.println(err);

  // Serial output for debug
  Serial.print("CURR: ");
  Serial.println((String)curr);
  
  
  Serial.print("RAIL: ");
  Serial.println(rail.getValue(), DEC);
  
  delay(100);
}
