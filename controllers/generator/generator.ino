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

//CANBus
FlexCAN CANbus(50000);
CAN_message_t msg,rxmsg;

void setup(){
  pinMode(CANS, OUTPUT);
  pinMode(THERM1, INPUT);
  pinMode(THERM2, INPUT);
  pinMode(RAIL_SEN, INPUT);
  pinMode(CUR_SEN, INPUT);
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);
  pinMode(GATES, OUTPUT);
  
  digitalWrite(GATES, LOW);
  digitalWrite(CANS, LOW);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Serial.begin(9600);
  Serial.println("Hello World!");
  
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
  
    
}

void loop(){
  Serial.println("I am generator ");
    
  //CANBus
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
    } 
  }
    //CANBus
  msg.len = 3;
  msg.id = GENERATOR_ID;
  for( int idx=0; idx< msg.len; ++idx ) {
      msg.buf[idx] = 0;
  }
  msg.buf[0] = GENERIC;
  msg.buf[1] = 'h';
  msg.buf[2] = 'w';
  int err = 0;
  err = CANbus.write(msg);
  Serial.println(String(err));
  
  int curr, rail;
  curr = analogRead(CUR_SEN);
  rail = analogRead(RAIL_SEN);
  
  Serial.print("CURR: ");
  Serial.println(curr, DEC);
  /*Serial.print("RAIL: ");
  Serial.println(rail, DEC);*/
  
  analogWrite(GATES, 128);
  
  delay(1000);
}
